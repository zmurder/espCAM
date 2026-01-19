#include <string.h>
#include <sys/socket.h>
#include <netdb.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "lwip/inet.h"
#include "led.h"
#include "driver/i2s.h"

#include "udp_camera_client.h"
#include "audio_player.h"  // 添加音频播放模块

static const char* TAG = "UDP_CAMERA";

// 目标PC的IP地址和端口
#define UDP_SERVER_IP "192.168.5.3"  // 替换为你的PC的IP地址，请根据实际情况修改
#define UDP_SERVER_PORT 8080

// PC发送语音的端口
#define UDP_AUDIO_PORT 8081

// UDP相关参数
#define MAX_UDP_PACKET_SIZE 1400  // MTU限制
#define RECV_TIMEOUT_MS 5000

// 图像分包传输结构
typedef struct
{
    uint32_t chunk_id;                                         // 包序号
    uint32_t total_chunks;                                     // 总包数
    uint32_t image_size;                                       // 图像总大小
    uint8_t data[MAX_UDP_PACKET_SIZE - sizeof(uint32_t) * 3];  // 数据区域
} udp_image_chunk_t;

// 音频数据包结构
typedef struct
{
    uint32_t packet_id;                                        // 音频包序号
    uint32_t total_packets;                                    // 音频总包数
    uint32_t audio_size;                                       // 音频总大小
    uint8_t data[MAX_UDP_PACKET_SIZE - sizeof(uint32_t) * 3];  // 音频数据区域
} udp_audio_chunk_t;

// 帧率统计相关变量
static uint32_t frame_count = 0;
static uint32_t last_fps_time = 0;
static float current_fps = 0.0f;

// UDP socket 复用，避免重复创建
static int s_udp_socket = -1;
static int s_audio_socket = -1;  // 新增音频接收socket
static struct sockaddr_in s_dest_addr;
static bool s_socket_initialized = false;
static bool s_audio_socket_initialized = false;

// 任务控制标志
static TaskHandle_t s_udp_task_handle = NULL;
static volatile bool s_udp_task_running = false;

/**
 * @brief 初始化UDP socket连接（只初始化一次）
 *
 * @return esp_err_t
 */
static esp_err_t init_udp_socket_once(void)
{
    if (s_socket_initialized && s_udp_socket >= 0) {
        return ESP_OK;  // Socket已初始化，直接返回
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

    // 设置接收超时
    timeout.tv_sec = RECV_TIMEOUT_MS / 1000;
    timeout.tv_usec = (RECV_TIMEOUT_MS % 1000) * 1000;
    setsockopt(s_udp_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    // 设置目标地址
    memset(&s_dest_addr, 0, sizeof(struct sockaddr_in));
    s_dest_addr.sin_family = AF_INET;
    s_dest_addr.sin_port = htons(UDP_SERVER_PORT);
    inet_aton(UDP_SERVER_IP, &s_dest_addr.sin_addr);

    s_socket_initialized = true;
    ESP_LOGI(TAG, "UDP socket初始化成功，目标地址: %s:%d", UDP_SERVER_IP, UDP_SERVER_PORT);

    return ESP_OK;
}

/**
 * @brief 初始化音频接收socket
 *
 * @return esp_err_t
 */
static esp_err_t init_audio_socket(void)
{
    if (s_audio_socket_initialized && s_audio_socket >= 0) {
        return ESP_OK;  // Socket已初始化，直接返回
    }

    s_audio_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_audio_socket < 0) {
        ESP_LOGE(TAG, "创建音频接收socket失败: errno %d", errno);
        return ESP_FAIL;
    }

    // 设置接收超时
    struct timeval timeout;
    timeout.tv_sec = 5;  // 5秒超时
    timeout.tv_usec = 0;
    setsockopt(s_audio_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    // 绑定到本地端口
    struct sockaddr_in local_addr;
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(UDP_AUDIO_PORT);
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(s_audio_socket, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        ESP_LOGE(TAG, "绑定音频端口失败: errno %d", errno);
        close(s_audio_socket);
        s_audio_socket = -1;
        return ESP_FAIL;
    }

    s_audio_socket_initialized = true;
    ESP_LOGI(TAG, "音频接收socket初始化成功，监听端口: %d", UDP_AUDIO_PORT);

    return ESP_OK;
}

/**
 * @brief 处理音频数据包
 *
 * @param audio_packet 音频数据包
 * @param packet_size 数据包大小
 */
static void handle_audio_packet(udp_audio_chunk_t* audio_packet, size_t packet_size)
{
    if (audio_packet == NULL) {
        return;
    }

    // 解析音频包信息
    uint32_t packet_id = ntohl(audio_packet->packet_id);
    uint32_t total_packets = ntohl(audio_packet->total_packets);
    uint32_t audio_size = ntohl(audio_packet->audio_size);

    ESP_LOGI(TAG, "收到音频包，ID: %lu/%lu, 音频大小: %lu bytes",
             (unsigned long)packet_id,
             (unsigned long)total_packets,
             (unsigned long)audio_size);

    // 播放音频数据
    size_t actual_data_size = packet_size - sizeof(uint32_t) * 3;
    if (actual_data_size > 0) {
        esp_err_t ret = audio_player_play_stream(audio_packet->data, actual_data_size);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "播放音频数据失败: %s", esp_err_to_name(ret));
        }
    }
}

/**
 * @brief 音频接收任务
 *
 * @param pvParameters 参数
 */
static void audio_receive_task(void* pvParameters)
{
    // 从堆中分配接收缓冲区，减少栈使用
    uint8_t* recv_buffer = heap_caps_malloc(MAX_UDP_PACKET_SIZE, MALLOC_CAP_DMA);
    if (recv_buffer == NULL) {
        ESP_LOGE(TAG, "无法分配音频接收缓冲区");
        return;
    }

    struct sockaddr_in source_addr;
    socklen_t addr_len = sizeof(source_addr);

    ESP_LOGI(TAG, "音频接收任务启动，监听端口: %d", UDP_AUDIO_PORT);

    while (s_udp_task_running) {
        int len = recvfrom(s_audio_socket, recv_buffer, MAX_UDP_PACKET_SIZE, 0,
                           (struct sockaddr*)&source_addr, &addr_len);

        if (len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;  // 超时，继续循环
            }
            ESP_LOGE(TAG, "音频接收错误: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(100));  // 错误后短暂延迟
            continue;
        }

        // 尝试解析为音频数据包
        if (len >= sizeof(udp_audio_chunk_t)) {
            udp_audio_chunk_t* audio_pkt = (udp_audio_chunk_t*)recv_buffer;
            handle_audio_packet(audio_pkt, len);
        } else {
            // 收到的数据包太小，可能是原始音频数据
            ESP_LOGD(TAG, "收到原始音频数据: %d bytes", len);
            esp_err_t ret = audio_player_play_stream(recv_buffer, len);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "播放原始音频数据失败: %s", esp_err_to_name(ret));
            }
        }
    }

    ESP_LOGI(TAG, "音频接收任务结束");
    heap_caps_free(recv_buffer);  // 释放接收缓冲区
    vTaskDelete(NULL);
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
 * @brief 关闭音频socket
 */
static void close_audio_socket(void)
{
    if (s_audio_socket >= 0) {
        close(s_audio_socket);
        s_audio_socket = -1;
        s_audio_socket_initialized = false;
        ESP_LOGI(TAG, "音频socket已关闭");
    }
}

/**
 * @brief 发送图像通过UDP（复用socket）
 *
 * @param fb 相机帧缓冲
 * @return esp_err_t
 */
esp_err_t send_image_via_udp(camera_fb_t* fb)
{
    // 初始化socket（如果尚未初始化）
    if (init_udp_socket_once() != ESP_OK) {
        return ESP_FAIL;
    }

    size_t total_size = fb->len;
    size_t bytes_sent = 0;
    uint32_t chunk_idx = 0;
    uint32_t total_chunks = (total_size + sizeof(((udp_image_chunk_t*)0)->data) - 1) / sizeof(((udp_image_chunk_t*)0)->data);

    ESP_LOGI(TAG, "开始发送图像，大小: %lu bytes, 分 %lu 包", (unsigned long)total_size, (unsigned long)total_chunks);

    // 使用静态变量避免栈上分配大数组
    static udp_image_chunk_t chunk;

    while (bytes_sent < total_size) {
        chunk.chunk_id = htonl(chunk_idx);
        chunk.total_chunks = htonl(total_chunks);
        chunk.image_size = htonl(total_size);

        // 计算当前包的数据大小
        size_t remaining = total_size - bytes_sent;
        size_t copy_size = (remaining > sizeof(chunk.data)) ? sizeof(chunk.data) : remaining;

        memcpy(chunk.data, fb->buf + bytes_sent, copy_size);

        // 发送包（使用复用的socket和目标地址）
        ssize_t sent = sendto(s_udp_socket, &chunk, sizeof(uint32_t) * 3 + copy_size, 0, (struct sockaddr*)&s_dest_addr, sizeof(struct sockaddr_in));

        if (sent < 0) {
            ESP_LOGE(TAG, "发送UDP包失败: errno %d", errno);
            // 发送失败时关闭socket，下次重新初始化
            close_udp_socket();
            return ESP_FAIL;
        }

        bytes_sent += copy_size;
        chunk_idx++;

        // 优化：减少延迟，提高传输速度
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    ESP_LOGI(TAG, "图像发送完成，共 %lu bytes", (unsigned long)total_size);

    return ESP_OK;
}

/**
 * @brief 捕获图像并通过UDP发送
 *
 * @return esp_err_t
 */
esp_err_t capture_and_send_udp()
{
    // 捕获图像
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "获取相机帧失败");
        return ESP_FAIL;
    }

    esp_err_t result = send_image_via_udp(fb);

    // 释放帧缓冲
    esp_camera_fb_return(fb);

    return result;
}

/**
 * @brief 计算并打印帧率
 */
void update_and_print_fps()
{
    uint32_t current_time = esp_timer_get_time() / 1000;  // 转换为毫秒

    frame_count++;

    // 每秒计算一次帧率
    if (current_time - last_fps_time >= 1000) {
        current_fps = (float)frame_count * 1000.0f / (current_time - last_fps_time);
        ESP_LOGI(TAG, "帧率: %f FPS, 总帧数: %lu", current_fps, (unsigned long)frame_count);

        frame_count = 0;
        last_fps_time = current_time;
    }
}

/**
 * @brief 获取当前帧率
 * @return 当前帧率 (FPS)
 */
float get_current_fps(void)
{
    return current_fps;
}

/**
 * @brief 获取总帧数
 * @return 总帧数
 */
uint32_t get_total_frames(void)
{
    return frame_count;
}

/**
 * @brief UDP图像传输任务
 *
 * @param pvParameters 参数
 */
void udp_camera_task(void* pvParameters)
{
    // 优化：减少延迟到1秒，提高帧率
    const uint32_t capture_interval_ms = 1000;  // 改为1秒间隔

    s_udp_task_running = true;

    // 初始化音频接收socket
    if (init_audio_socket() != ESP_OK) {
        ESP_LOGE(TAG, "音频socket初始化失败");
    } else {
        // 启动音频接收任务
        xTaskCreate(audio_receive_task, "audio_receive_task", 4096, NULL, 5, NULL);
    }

    while (s_udp_task_running) {
        uint64_t start_time = esp_timer_get_time();

        ESP_LOGI(TAG, "捕获并发送图像...");
        esp_err_t result = capture_and_send_udp();
        if (result == ESP_OK) {
            ESP_LOGI(TAG, "图像发送成功");
        }
        else {
            ESP_LOGE(TAG, "图像发送失败");
        }

        uint64_t end_time = esp_timer_get_time();
        uint64_t capture_time = end_time - start_time;

        // 打印捕获时间
        ESP_LOGI(TAG, "图像捕获耗时: %llu 微秒", capture_time);

        // 更新并打印帧率
        update_and_print_fps();

        // 优化：减少延迟时间
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
    close_audio_socket();
}

/**
 * @brief 重启UDP图像传输（在WiFi重置后调用）
 */
void restart_udp_camera(void)
{
    ESP_LOGI(TAG, "重启UDP图像传输");
    // 停止当前任务
    stop_udp_camera();

    // 等待任务完全停止
    vTaskDelay(pdMS_TO_TICKS(100));

    // 重新启动
    start_udp_camera();
}

/**
 * @brief 启动UDP图像传输
 */
void start_udp_camera()
{
    // 初始化帧率统计
    frame_count = 0;
    last_fps_time = esp_timer_get_time() / 1000;
    current_fps = 0.0f;
    // 启动呼吸灯表示正常图像发送
    led_set_state(LED_STATE_BREATH);
    // 增加任务栈大小以处理图像数据
    xTaskCreate(udp_camera_task, "udp_camera_task", 8192, NULL, 5, &s_udp_task_handle);
}