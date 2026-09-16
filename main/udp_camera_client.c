#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netdb.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_app_format.h"
#include "esp_http_client.h"
#include "esp_ota_ops.h"
#include "lwip/inet.h"
#include "led.h"

#include "udp_camera_client.h"
#include "posture_model.h"
#include "posture_sched.h"
#include "time_sync.h"
#include "app_version.h"

static const char* TAG = "UDP_CAMERA";

// 目标PC的IP地址和端口
#define UDP_SERVER_IP "192.168.31.126"  // 编译期默认值；运行时会被自动发现机制覆盖
#define UDP_SERVER_PORT 20000

// UDP 自动发现（PC IP 变化免改代码）：PC 周期广播发现包到 20003，
// ESP32 学习其源 IP 作为发送目标并回 ACK；IP 变化后下一个广播周期(2s)内自动跟随
#define UDP_DISCOVERY_PORT 20003
#define DISCOVERY_REQ "ESPCAM_DISCOVER"
#define DISCOVERY_ACK "ESPCAM_ACK"

// UDP相关参数
#define MAX_UDP_PACKET_SIZE 1400  // MTU限制

// 图像分包传输结构
typedef struct
{
    uint32_t chunk_id;                                         // 包序号
    uint32_t total_chunks;                                     // 总包数
    uint32_t image_size;                                       // 图像总大小
    uint8_t data[MAX_UDP_PACKET_SIZE - sizeof(uint32_t) * 3];  // 数据区域
} udp_image_chunk_t;

// 帧率统计相关变量
static uint32_t frame_count = 0;
static uint32_t last_fps_time = 0;
static float current_fps = 0.0f;

// UDP socket
static int s_udp_socket = -1;
static struct sockaddr_in s_dest_addr;
static struct sockaddr_in s_posture_dest_addr;  // 姿态结果目标地址
static bool s_socket_initialized = false;

// 运行时目标 IP（网络序；0=未学习，用编译期默认）。32 位对齐写天然原子无需加锁；
// 极端竞态（与 init 同时写）最坏丢一次更新，下个广播周期即恢复
static in_addr_t s_server_ip = 0;

// 发现/调度 socket（20003；任务外可见，供 posture_sched 主动推送状态）
static int s_disc_socket = -1;

// 更新发送目标 IP（发现任务调用；无变化时静默）
static void set_server_ip(in_addr_t ip)
{
    if (ip == 0 || ip == s_server_ip) {
        return;
    }
    struct in_addr new_addr;
    new_addr.s_addr = ip;
    if (s_server_ip != 0) {
        struct in_addr old_addr;
        old_addr.s_addr = s_server_ip;
        ESP_LOGW(TAG, "UDP 目标 IP 更新: %s -> %s", inet_ntoa(old_addr), inet_ntoa(new_addr));
    } else {
        ESP_LOGI(TAG, "UDP 目标 IP 由发现机制设定: %s", inet_ntoa(new_addr));
    }
    s_server_ip = ip;
    s_dest_addr.sin_addr.s_addr = ip;
    s_posture_dest_addr.sin_addr.s_addr = ip;
}

// 任务控制标志
static TaskHandle_t s_udp_task_handle = NULL;
static volatile bool s_udp_task_running = false;

/**
 * @brief 初始化UDP socket连接（只初始化一次）
 */
static esp_err_t init_udp_socket_once(void)
{
    if (s_socket_initialized && s_udp_socket >= 0) {
        return ESP_OK;
    }

    s_udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_udp_socket < 0) {
        ESP_LOGE(TAG, "创建UDP socket失败: errno %d", errno);
        return ESP_FAIL;
    }

    // 设置发送超时
    struct timeval timeout;
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;
    setsockopt(s_udp_socket, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    // 设置目标地址（端口固定；IP 优先用发现机制学到的，未学习时用编译期默认）
    memset(&s_dest_addr, 0, sizeof(struct sockaddr_in));
    s_dest_addr.sin_family = AF_INET;
    s_dest_addr.sin_port = htons(UDP_SERVER_PORT);

    // 设置姿态结果目标地址（同一IP，不同端口）
    memset(&s_posture_dest_addr, 0, sizeof(struct sockaddr_in));
    s_posture_dest_addr.sin_family = AF_INET;
    s_posture_dest_addr.sin_port = htons(20002);  // 姿态结果端口

    if (s_server_ip != 0) {
        s_dest_addr.sin_addr.s_addr = s_server_ip;
        s_posture_dest_addr.sin_addr.s_addr = s_server_ip;
    } else {
        inet_aton(UDP_SERVER_IP, &s_dest_addr.sin_addr);
        inet_aton(UDP_SERVER_IP, &s_posture_dest_addr.sin_addr);
    }

    s_socket_initialized = true;
    ESP_LOGI(TAG, "UDP socket初始化成功，目标地址: %s:%d", inet_ntoa(s_dest_addr.sin_addr), UDP_SERVER_PORT);

    return ESP_OK;
}

/**
 * @brief 关闭UDP socket
 */
static void close_udp_socket(void)
{
    if (s_udp_socket >= 0) {
        close(s_udp_socket);
        s_udp_socket = -1;
        s_socket_initialized = false;
        ESP_LOGI(TAG, "UDP socket已关闭");
    }
}

/**
 * @brief 发送图像通过UDP
 */
esp_err_t send_image_via_udp(camera_fb_t* fb)
{
    if (init_udp_socket_once() != ESP_OK) {
        return ESP_FAIL;
    }

    size_t total_size = fb->len;
    size_t bytes_sent = 0;
    uint32_t chunk_idx = 0;
    size_t data_size = sizeof(((udp_image_chunk_t*)0)->data);
    uint32_t total_chunks = (total_size + data_size - 1) / data_size;

    ESP_LOGI(TAG, "开始发送图像，大小: %lu bytes, 分 %lu 包",
             (unsigned long)total_size, (unsigned long)total_chunks);

    static udp_image_chunk_t chunk;

    while (bytes_sent < total_size) {
        chunk.chunk_id = htonl(chunk_idx);
        chunk.total_chunks = htonl(total_chunks);
        chunk.image_size = htonl(total_size);

        size_t remaining = total_size - bytes_sent;
        size_t copy_size = (remaining > data_size) ? data_size : remaining;

        memcpy(chunk.data, fb->buf + bytes_sent, copy_size);

        ssize_t sent = sendto(s_udp_socket, &chunk, sizeof(uint32_t) * 3 + copy_size, 0,
                              (struct sockaddr*)&s_dest_addr, sizeof(struct sockaddr_in));

        if (sent < 0) {
            ESP_LOGE(TAG, "发送UDP包失败: errno %d", errno);
            close_udp_socket();
            return ESP_FAIL;
        }

        bytes_sent += copy_size;
        chunk_idx++;

        // 添加小延迟，减少网络拥塞导致的包乱序
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    ESP_LOGI(TAG, "图像发送完成，共 %lu bytes", (unsigned long)total_size);
    return ESP_OK;
}

/**
 * @brief 捕获图像并通过UDP发送
 */
esp_err_t capture_and_send_udp(void)
{
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "获取相机帧失败");
        return ESP_FAIL;
    }

    esp_err_t result = send_image_via_udp(fb);
    esp_camera_fb_return(fb);

    return result;
}

/**
 * @brief 计算并打印帧率
 */
void update_and_print_fps(void)
{
    uint32_t current_time = esp_timer_get_time() / 1000;

    frame_count++;

    if (current_time - last_fps_time >= 1000) {
        current_fps = (float)frame_count * 1000.0f / (current_time - last_fps_time);
        ESP_LOGI(TAG, "帧率: %f FPS", current_fps);

        frame_count = 0;
        last_fps_time = current_time;
    }
}

/**
 * @brief UDP图像传输任务
 */
void udp_camera_task(void* pvParameters)
{
    const uint32_t capture_interval_ms = 500;  // 500ms间隔，约2 FPS

    s_udp_task_running = true;

    while (s_udp_task_running) {
        uint64_t start_time = esp_timer_get_time();

        esp_err_t result = capture_and_send_udp();
        if (result == ESP_OK) {
            ESP_LOGD(TAG, "图像发送成功");
        } else {
            ESP_LOGE(TAG, "图像发送失败");
        }

        uint64_t capture_time = esp_timer_get_time() - start_time;
        ESP_LOGI(TAG, "图像捕获耗时: %llu 微秒", capture_time);

        update_and_print_fps();

        vTaskDelay(pdMS_TO_TICKS(capture_interval_ms));
    }

    s_udp_task_running = false;
    s_udp_task_handle = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief 停止UDP图像传输
 */
void stop_udp_camera(void)
{
    ESP_LOGI(TAG, "停止UDP图像传输");
    s_udp_task_running = false;
    close_udp_socket();
}

/**
 * @brief 重启UDP图像传输
 */
void restart_udp_camera(void)
{
    stop_udp_camera();
    vTaskDelay(pdMS_TO_TICKS(100));
    start_udp_camera();
}

/**
 * @brief 启动UDP图像传输
 */
void start_udp_camera(void)
{
    frame_count = 0;
    last_fps_time = esp_timer_get_time() / 1000;
    current_fps = 0.0f;

    led_set_state(LED_STATE_BREATH);
    xTaskCreate(udp_camera_task, "udp_camera_task", 8192, NULL, 5, &s_udp_task_handle);
}

/**
 * @brief 获取当前帧率
 */
float get_current_fps(void)
{
    return current_fps;
}

/**
 * @brief 获取总帧数
 */
uint32_t get_total_frames(void)
{
    return frame_count;
}

/**
 * @brief 发送姿态检测结果通过UDP
 */
void send_posture_result_via_udp(const posture_output_t* output)
{
    if (init_udp_socket_once() != ESP_OK) {
        return;
    }

    // 姿态结果 UDP 包格式：
    // - 包类型: 0x02 (姿态结果)
    // - result: 1字节
    // - ratio: 4字节 float
    // - keypoints: 4 * (4*float + 4*float + 4*float) = 4 * 12 = 48字节
    // 总共: 1 + 4 + 48 = 53字节

    static uint8_t posture_buf[128];
    int offset = 0;

    // 包类型
    posture_buf[offset++] = 0x02;

    // result
    posture_buf[offset++] = (uint8_t)output->result;

    // ratio (float)
    float ratio = output->ratio;
    memcpy(&posture_buf[offset], &ratio, sizeof(float));
    offset += sizeof(float);

    // keypoints (4个关键点，每个: x, y, score 各4字节)
    for (int i = 0; i < KEYPOINT_COUNT; i++) {
        memcpy(&posture_buf[offset], &output->keypoints[i].x, sizeof(float));
        offset += sizeof(float);
        memcpy(&posture_buf[offset], &output->keypoints[i].y, sizeof(float));
        offset += sizeof(float);
        memcpy(&posture_buf[offset], &output->keypoints[i].score, sizeof(float));
        offset += sizeof(float);
    }

    // 发送到姿态结果专用端口
    ssize_t sent = sendto(s_udp_socket, posture_buf, offset, 0,
                          (struct sockaddr*)&s_posture_dest_addr, sizeof(struct sockaddr_in));
    if (sent < 0) {
        ESP_LOGE(TAG, "发送姿态结果UDP失败: errno %d", errno);
    } else {
        ESP_LOGI(TAG, "发送姿态结果: result=%d, ratio=%.3f", output->result, output->ratio);
    }
}

// 供 posture_sched 主动推送：把调度状态文本发到学到的 PC IP:20003
// （SNTP 同步完成时调用；无 socket 或未学到 PC 地址则跳过）
void udp_sched_push_state(void)
{
    if (s_disc_socket < 0 || s_server_ip == 0) {
        return;
    }
    char buf[160];
    int n = posture_sched_build_state(buf, sizeof(buf));
    struct sockaddr_in to;
    memset(&to, 0, sizeof(to));
    to.sin_family = AF_INET;
    to.sin_port = htons(UDP_DISCOVERY_PORT);
    to.sin_addr.s_addr = s_server_ip;
    sendto(s_disc_socket, buf, n, 0, (struct sockaddr*)&to, sizeof(to));
}

// ---------- OTA 固件升级（局域网 HTTP，地址由 PC 广播动态下发）----------

static volatile bool s_ota_running = false;

// 经 20003 通道向已学习 IP 推送一行文本（OTA 进度/结果）
static void push_disc_text(const char* text)
{
    if (s_disc_socket < 0 || s_server_ip == 0) {
        return;
    }
    struct sockaddr_in to;
    memset(&to, 0, sizeof(to));
    to.sin_family = AF_INET;
    to.sin_port = htons(UDP_DISCOVERY_PORT);
    to.sin_addr.s_addr = s_server_ip;
    sendto(s_disc_socket, text, strlen(text), 0, (struct sockaddr*)&to, sizeof(to));
}

bool udp_ota_in_progress(void)
{
    return s_ota_running;
}

/**
 * @brief OTA 下载任务：HTTP 流式下载 → 写入另一 app 分区 → 校验 → 切换启动分区 → 重启。
 *        URL 由 PC 经 20003 广播下发（IP 动态，无需固定）；期间推理照常（共享带宽，下载稍慢）。
 */
static void ota_update_task(void* arg)
{
    char* url = (char*)arg;
    // 注意：s_ota_running 已在命令处理分支置位（xTaskCreate 之前），任务体内不再置位
    static char rbuf[4096];  // 下载缓冲（放 BSS，避免大栈）
    esp_http_client_handle_t client = NULL;
    esp_ota_handle_t ota_handle = 0;
    esp_err_t err = ESP_FAIL;
    int total = 0;
    int64_t clen = 0;

    ESP_LOGI(TAG, "OTA: 开始下载 %s", url);
    // SoftAP+STA 并发共存会拖累 STA 下行吞吐（AP beacon/管理帧占空口、驱动收发竞争）：
    // 下载前切纯 STA。重启进新固件后 app_main 会重新建 APSTA（完整恢复）；
    // 下载失败也保持纯 STA 继续检测（热点暂缺，重启即恢复）
    if (esp_wifi_stop() == ESP_OK) {
        if (esp_wifi_set_mode(WIFI_MODE_STA) == ESP_OK) {
            esp_wifi_start();
            esp_wifi_set_ps(WIFI_PS_NONE);  // 重启 WiFi 后再设一次（防回默认省电）
            vTaskDelay(pdMS_TO_TICKS(3000));  // 等 STA 重连拿到 IP
            ESP_LOGI(TAG, "OTA: 已切纯 STA（SoftAP 暂停，重启后恢复）");
        }
        else {
            esp_wifi_start();  // 切换失败：按原 APSTA 启动，不阻塞 OTA
        }
    }
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        // RSSI 诊断：> -60 很好，-60~-70 良好，-70~-80 一般，< -80 弱（低速调制，吞吐骤降）
        ESP_LOGI(TAG, "OTA: WiFi 信号 %d dBm（> -70 正常）", ap_info.rssi);
    }
    do {
        esp_http_client_config_t cfg = {
            .url = url,
            .timeout_ms = 15000,
        };
        client = esp_http_client_init(&cfg);
        if (client == NULL) {
            ESP_LOGE(TAG, "OTA: http client 初始化失败");
            break;
        }
        err = esp_http_client_open(client, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "OTA: 连接失败: %s", esp_err_to_name(err));
            break;
        }
        clen = esp_http_client_fetch_headers(client);
        int status = esp_http_client_get_status_code(client);
        if (status != 200 || clen <= 0) {
            ESP_LOGE(TAG, "OTA: HTTP 状态 %d，长度 %lld（固件不存在？）", status, (long long)clen);
            err = ESP_FAIL;
            break;
        }
        const esp_partition_t* part = esp_ota_get_next_update_partition(NULL);
        if (part == NULL) {
            ESP_LOGE(TAG, "OTA: 找不到可写的 app 分区（分区表无 ota 槽？）");
            err = ESP_FAIL;
            break;
        }
        err = esp_ota_begin(part, OTA_SIZE_UNKNOWN, &ota_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "OTA: esp_ota_begin 失败: %s", esp_err_to_name(err));
            break;
        }
        int last_log = 0;
        int64_t net_us = 0, flash_us = 0;  // 网络 read / flash 写累计耗时（诊断下载慢在哪一侧）
        while (true) {
            int64_t t0 = esp_timer_get_time();
            int n = esp_http_client_read(client, rbuf, sizeof(rbuf));
            net_us += esp_timer_get_time() - t0;
            if (n == 0) {
                break;  // 下完（content-length 已满）
            }
            if (n < 0) {
                ESP_LOGE(TAG, "OTA: 下载中断（超时/网络断开）");
                err = ESP_FAIL;
                break;
            }
            t0 = esp_timer_get_time();
            err = esp_ota_write(ota_handle, rbuf, n);  // 首块会校验镜像头（magic）
            flash_us += esp_timer_get_time() - t0;
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "OTA: esp_ota_write 失败: %s", esp_err_to_name(err));
                break;
            }
            total += n;
            if (total - last_log >= 256 * 1024) {  // 每 256KB：串口日志 + 推送 PC 显示进度
                ESP_LOGI(TAG, "OTA: 进度 %d/%lld KB（累计等待网络 %lld ms / 写 flash %lld ms）",
                         total / 1024, (long long)(clen / 1024), (long long)(net_us / 1000), (long long)(flash_us / 1000));
                char prog[56];
                int plen = snprintf(prog, sizeof(prog), "ESPCAM_OTA_PROGRESS %d %lld", total, (long long)clen);
                push_disc_text(prog);
                last_log = total;
            }
        }
        if (err != ESP_OK) {
            break;
        }
        if (total != (int)clen) {
            ESP_LOGE(TAG, "OTA: 长度不符 已收 %d 应为 %lld", total, (long long)clen);
            err = ESP_FAIL;
            break;
        }
        err = esp_ota_end(ota_handle);  // 校验镜像完整性
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "OTA: esp_ota_end 失败: %s（镜像校验未过）", esp_err_to_name(err));
            esp_ota_abort(ota_handle);  // end 失败后须显式 abort 释放
            ota_handle = 0;
            break;
        }
        ota_handle = 0;  // 成功：handle 已被消费，失败路径不可再 abort
        err = esp_ota_set_boot_partition(part);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "OTA: 切换启动分区失败: %s", esp_err_to_name(err));
            break;
        }
        ESP_LOGI(TAG, "OTA: 完成，共 %d KB，600ms 后重启进入新固件（重启后 ACK 带新版本号）", total / 1024);
        push_disc_text("ESPCAM_OTA_DONE");
        if (client != NULL) {
            esp_http_client_cleanup(client);
        }
        free(url);
        vTaskDelay(pdMS_TO_TICKS(600));  // 等 UDP 发出再重启
        esp_restart();
        return;  // 不会执行到这
    } while (0);

    // 失败路径：清理并通知 PC（当前固件继续运行，不受影响）
    if (ota_handle != 0 && err != ESP_OK) {
        esp_ota_abort(ota_handle);
    }
    if (client != NULL) {
        esp_http_client_cleanup(client);
    }
    char emsg[64];
    snprintf(emsg, sizeof(emsg), "ESPCAM_OTA_ERR %s", esp_err_to_name(err));
    push_disc_text(emsg);
    free(url);
    s_ota_running = false;
    vTaskDelete(NULL);
}

/**
 * @brief UDP 20003 服务任务：自动发现 + 检测时间段设置/查询 + OTA 触发
 *  - ESPCAM_DISCOVER              → 学习源 IP 为发送目标 + 回 "ESPCAM_ACK <版本号>"
 *  - ESPCAM_SCHED_GET             → 回当前调度状态
 *  - ESPCAM_SCHED_SET n HH:MM …   → 设置时间段（写 NVS）+ 回状态；解析失败回 ESPCAM_SCHED_ERR 原因
 *  - ESPCAM_OTA_START http://…    → 回 ESPCAM_OTA_STARTED 并启动下载任务（PC 的 IP 动态）
 */
static void udp_discovery_task(void* pvParameters)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "发现 socket 创建失败: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);  // 任意接口（STA/AP 均可收到广播）
    bind_addr.sin_port = htons(UDP_DISCOVERY_PORT);
    if (bind(sock, (struct sockaddr*)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "发现端口 %d 绑定失败: errno %d", UDP_DISCOVERY_PORT, errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    s_disc_socket = sock;
    // 接收超时：无 PC 流量时循环也能 1s 一转，轮询对时事件（见下 time_sync_pop_event）
    struct timeval rto = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &rto, sizeof(rto));
    ESP_LOGI(TAG, "UDP 发现/调度监听已启动 :%d（等待 PC 广播，自动学习目标 IP）", UDP_DISCOVERY_PORT);

    char rx[160];
    while (1) {
        // SNTP 对时回调运行在 lwIP tcpip_thread 上下文，不能直接 sendto（自等待死锁，
        // 会挂起之后所有 socket 操作）——回调只置标志，由本任务在此取走并推送状态到 PC
        if (time_sync_pop_event()) {
            udp_sched_push_state();
        }
        struct sockaddr_in src;
        socklen_t src_len = sizeof(src);
        ssize_t n = recvfrom(sock, rx, sizeof(rx) - 1, 0, (struct sockaddr*)&src, &src_len);
        if (n <= 0) {
            continue;  // 1s 接收超时（已含等待），转回循环顶部继续轮询
        }
        rx[n] = '\0';
        if (strcmp(rx, DISCOVERY_REQ) == 0) {
            set_server_ip(src.sin_addr.s_addr);  // 学习 PC 的 IP（已相同则静默）
            // ACK 附带设备 ID（WiFi MAC 后 3 字节 hex，多设备区分）+ 版本号 + ELF SHA256 前 8 位
            // （PC 维护设备表：逐台对比 ota/ 固件 sha 决定是否升级）
            char ack[80];
            char sha8[9];
            uint8_t mac[6];
            esp_read_mac(mac, ESP_MAC_WIFI_STA);
            esp_app_get_elf_sha256(sha8, sizeof(sha8));
            int alen = snprintf(ack, sizeof(ack), "ESPCAM_ACK %02x%02x%02x %s %s",
                                mac[3], mac[4], mac[5], APP_VERSION, sha8);
            sendto(sock, ack, alen, 0, (struct sockaddr*)&src, sizeof(src));
        }
        else if (strncmp(rx, "ESPCAM_SCHED_GET", 16) == 0) {
            char buf[160];
            int len = posture_sched_build_state(buf, sizeof(buf));
            sendto(sock, buf, len, 0, (struct sockaddr*)&src, sizeof(src));
        }
        else if (strncmp(rx, "ESPCAM_OTA_START ", 17) == 0) {
            const char* url = rx + 17;
            if (strlen(url) < 10) {
                sendto(sock, "ESPCAM_OTA_ERR bad url", 22, 0, (struct sockaddr*)&src, sizeof(src));
            }
            else if (s_ota_running) {
                sendto(sock, "ESPCAM_OTA_ERR busy", 19, 0, (struct sockaddr*)&src, sizeof(src));
            }
            else {
                set_server_ip(src.sin_addr.s_addr);  // 升级期间推送进度用
                char* u = malloc(strlen(url) + 1);  // 任务自行 free
                if (u == NULL) {
                    sendto(sock, "ESPCAM_OTA_ERR oom", 19, 0, (struct sockaddr*)&src, sizeof(src));
                }
                else {
                    strcpy(u, url);
                    // 置位必须在 xTaskCreate 之前：本任务串行处理 20003 命令，双份广播包
                    // 第二份到达时必然看到 true → 回 busy。若等任务体第一行才置位，任务
                    // 未及调度、第二份已通过检查 → 两个下载任务并发写同一分区 → 镜像损坏
                    s_ota_running = true;
                    if (xTaskCreate(ota_update_task, "ota_update", 8192, u, 4, NULL) != pdPASS) {
                        s_ota_running = false;
                        free(u);
                        sendto(sock, "ESPCAM_OTA_ERR oom", 19, 0, (struct sockaddr*)&src, sizeof(src));
                    }
                    else {
                        sendto(sock, "ESPCAM_OTA_STARTED", 18, 0, (struct sockaddr*)&src, sizeof(src));
                    }
                }
            }
        }
        else if (strncmp(rx, "ESPCAM_SCHED_SET", 16) == 0) {
            char err[48] = "";
            if (!posture_sched_parse_set(rx + 16, err, sizeof(err))) {
                // 失败：显式回错误原因（设置未生效、NVS 未动），PC 端能明确感知
                ESP_LOGW(TAG, "SCHED_SET 解析失败(%s): %s", err, rx);
                char ebuf[80];
                int elen = snprintf(ebuf, sizeof(ebuf), "ESPCAM_SCHED_ERR %s", err);
                sendto(sock, ebuf, elen, 0, (struct sockaddr*)&src, sizeof(src));
            }
            else {
                char buf[160];
                int len = posture_sched_build_state(buf, sizeof(buf));
                sendto(sock, buf, len, 0, (struct sockaddr*)&src, sizeof(src));  // 回状态即确认
            }
        }
    }
}

/**
 * @brief 启动 UDP 自动发现监听（幂等，重复调用无副作用）
 */
void udp_discovery_start(void)
{
    static bool started = false;
    if (started) {
        return;
    }
    started = true;
    xTaskCreate(udp_discovery_task, "udp_discovery", 4096, NULL, 4, NULL);
}