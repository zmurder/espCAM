#include <string.h>
#include <sys/socket.h>
#include <netdb.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "lwip/inet.h"
#include "led.h"

#include "udp_camera_client.h"
#include "posture_model.h"

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

/**
 * @brief UDP 自动发现监听任务：PC 端周期广播 DISCOVERY_REQ 到 20003，
 *        学习其源 IP 为图像/结果发送目标（变化时自动切换），并回复 ACK 供 PC 确认
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
    ESP_LOGI(TAG, "UDP 发现监听已启动 :%d（等待 PC 广播，自动学习目标 IP）", UDP_DISCOVERY_PORT);

    char rx[32];
    while (1) {
        struct sockaddr_in src;
        socklen_t src_len = sizeof(src);
        ssize_t n = recvfrom(sock, rx, sizeof(rx) - 1, 0, (struct sockaddr*)&src, &src_len);
        if (n <= 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        rx[n] = '\0';
        if (strcmp(rx, DISCOVERY_REQ) == 0) {
            set_server_ip(src.sin_addr.s_addr);  // 学习 PC 的 IP（已相同则静默）
            sendto(sock, DISCOVERY_ACK, strlen(DISCOVERY_ACK), 0,
                   (struct sockaddr*)&src, sizeof(src));
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