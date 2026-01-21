#include "audio_player.h"
#include "driver/i2s.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

static const char* TAG = "AUDIO_PLAYER";

#define I2S_PORT I2S_NUM_0
#define SAMPLE_RATE (8000)  // 8kHz采样率（MAX98357需要16位数据，降低采样率使播放速度正常）
#define CHANNELS (1)  // 单声道
#define BITS_PER_SAMPLE (I2S_BITS_PER_SAMPLE_16BIT)  // 改回 16 位，MAX98357 需要

// ============================================================================
// 音频数据（存储在 Flash 中，运行时加载到 PSRAM）
// ============================================================================
// 音频格式: 8位PCM, 16000Hz, 单声道

// "WiFi已连接" - 音频数据
#include "wifi_connect.c"

// "WiFi已断开" (WiFi蜂鸣声) - 音频数据
#include "wifi_beak.c"

// "WiFi已重置" - 音频数据
#include "wifi_reset.c"

// 运行时 PSRAM 缓冲区指针
static uint8_t* wifi_connect_psram = NULL;
static uint8_t* wifi_beak_psram = NULL;
static uint8_t* wifi_reset_psram = NULL;

// I2S 初始化状态标志
static bool i2s_initialized = false;

// I2S配置
static i2s_config_t i2s_config = {
    .mode = I2S_MODE_MASTER | I2S_MODE_TX,
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,  // 改回 16 位，MAX98357 需要
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,  // 禁用中断，使用轮询模式（避免与相机中断冲突）
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
};

// I2S引脚配置 - 使用 GPIO 14, 12, 15 避免与相机冲突
static i2s_pin_config_t i2s_pin_config = {
    .bck_io_num = 14,      // I2S 位时钟 (BCLK)
    .ws_io_num = 13,       // I2S 字选择/左右声道时钟 (LRCK/WS) - 改为 GPIO 13
    .data_out_num = 15,    // I2S 数据输出 (DOUT)
    .data_in_num = -1      // 不使用数据输入
};

esp_err_t audio_player_init(void)
{
    esp_err_t ret = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2S driver: %s", esp_err_to_name(ret));
        i2s_initialized = false;
        return ret;
    }

    ret = i2s_set_pin(I2S_PORT, &i2s_pin_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set I2S pin: %s", esp_err_to_name(ret));
        i2s_driver_uninstall(I2S_PORT);
        i2s_initialized = false;
        return ret;
    }

    // 将音频数据从 Flash 加载到 PSRAM
    ESP_LOGI(TAG, "Loading audio data to PSRAM...");

    wifi_connect_psram = (uint8_t*)heap_caps_malloc(WIFI_CONNECT_AUDIO_LEN, MALLOC_CAP_SPIRAM);
    if (wifi_connect_psram) {
        memcpy(wifi_connect_psram, wifi_connect_audio, WIFI_CONNECT_AUDIO_LEN);
        ESP_LOGI(TAG, "  WiFi connect audio: %d bytes loaded to PSRAM", WIFI_CONNECT_AUDIO_LEN);
    } else {
        ESP_LOGW(TAG, "  Failed to allocate PSRAM for WiFi connect audio, will use Flash directly");
    }

    wifi_beak_psram = (uint8_t*)heap_caps_malloc(WIFI_BEAK_AUDIO_LEN, MALLOC_CAP_SPIRAM);
    if (wifi_beak_psram) {
        memcpy(wifi_beak_psram, wifi_beak_audio, WIFI_BEAK_AUDIO_LEN);
        ESP_LOGI(TAG, "  WiFi beak audio: %d bytes loaded to PSRAM", WIFI_BEAK_AUDIO_LEN);
    } else {
        ESP_LOGW(TAG, "  Failed to allocate PSRAM for WiFi beak audio, will use Flash directly");
    }

    wifi_reset_psram = (uint8_t*)heap_caps_malloc(WIFI_RESET_AUDIO_LEN, MALLOC_CAP_SPIRAM);
    if (wifi_reset_psram) {
        memcpy(wifi_reset_psram, wifi_reset_audio, WIFI_RESET_AUDIO_LEN);
        ESP_LOGI(TAG, "  WiFi reset audio: %d bytes loaded to PSRAM", WIFI_RESET_AUDIO_LEN);
    } else {
        ESP_LOGW(TAG, "  Failed to allocate PSRAM for WiFi reset audio, will use Flash directly");
    }

    i2s_initialized = true;
    ESP_LOGI(TAG, "Audio player initialized successfully");
    return ESP_OK;
}

esp_err_t audio_player_play_data(uint8_t* audio_data, size_t data_size)
{
    if (audio_data == NULL || data_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // 将音频数据发送到I2S接口
    size_t bytes_written;
    size_t offset = 0;
    
    while (offset < data_size) {
        size_t chunk_size = (data_size - offset > 1024) ? 1024 : (data_size - offset);
        
        i2s_write(I2S_PORT, audio_data + offset, chunk_size, &bytes_written, portMAX_DELAY);
        offset += bytes_written;
    }

    return ESP_OK;
}

/**
 * @brief 播放8位PCM音频数据（扩展为16位）
 * @param audio_data 8位PCM音频数据
 * @param data_size 音频数据大小
 * @return esp_err_t
 */
static esp_err_t play_8bit_pcm(const uint8_t* audio_data, size_t data_size)
{
    if (audio_data == NULL || data_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // 分配缓冲区用于8位到16位扩展
    uint16_t* buffer = (uint16_t*)malloc(2048 * sizeof(uint16_t));
    if (buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate audio buffer");
        return ESP_ERR_NO_MEM;
    }

    size_t offset = 0;
    const size_t chunk_size = 2048;

    while (offset < data_size) {
        size_t current_chunk = (data_size - offset > chunk_size) ? chunk_size : (data_size - offset);

        // 将8位PCM扩展为16位PCM（每个字节重复两次）
        for (size_t i = 0; i < current_chunk; i++) {
            // 8位无符号转16位有符号：(value - 128) * 256
            int16_t sample = ((int16_t)audio_data[offset + i] - 128) * 256;
            buffer[i] = (uint16_t)sample;
        }

        // 播放扩展后的数据
        size_t bytes_written;
        esp_err_t ret = i2s_write(I2S_PORT, buffer, current_chunk * 2, &bytes_written, portMAX_DELAY);
        if (ret != ESP_OK) {
            free(buffer);
            ESP_LOGE(TAG, "Failed to write audio data: %s", esp_err_to_name(ret));
            return ret;
        }

        offset += current_chunk;
    }

    free(buffer);
    return ESP_OK;
}

esp_err_t audio_player_play_wifi_status(int status)
{
    // 检查 I2S 是否已初始化
    if (!i2s_initialized) {
        ESP_LOGW(TAG, "I2S not initialized, skipping audio playback for status %d", status);
        return ESP_ERR_INVALID_STATE;
    }

    const uint8_t* audio_data = NULL;
    size_t audio_len = 0;
    const char* audio_name = NULL;

    switch(status) {
        case 0: // WiFi已连接
            audio_data = wifi_connect_psram ? wifi_connect_psram : wifi_connect_audio;
            audio_len = WIFI_CONNECT_AUDIO_LEN;
            audio_name = "WiFi已连接";
            break;

        case 1: // WiFi已断开 (蜂鸣声)
            audio_data = wifi_beak_psram ? wifi_beak_psram : wifi_beak_audio;
            audio_len = WIFI_BEAK_AUDIO_LEN;
            audio_name = "WiFi蜂鸣声";
            break;

        case 2: // WiFi已重置
            audio_data = wifi_reset_psram ? wifi_reset_psram : wifi_reset_audio;
            audio_len = WIFI_RESET_AUDIO_LEN;
            audio_name = "WiFi已重置";
            break;

        default:
            ESP_LOGE(TAG, "Invalid WiFi status: %d", status);
            return ESP_FAIL;
    }

    if (audio_data == NULL || audio_len == 0) {
        ESP_LOGE(TAG, "Audio data not available for status: %d", status);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Playing audio: %s (%zu bytes, from %s)", audio_name, audio_len,
             (status == 0 && wifi_connect_psram) || (status == 1 && wifi_beak_psram) || (status == 2 && wifi_reset_psram) ? "PSRAM" : "Flash");

    // 播放8位PCM音频数据
    return play_8bit_pcm(audio_data, audio_len);
}

esp_err_t audio_player_stop(void)
{
    return i2s_zero_dma_buffer(I2S_PORT);
}

esp_err_t audio_player_play_stream(uint8_t* audio_data, size_t data_size)
{
    if (audio_data == NULL || data_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // 将音频数据流式发送到I2S接口
    // 使用更大的块大小以提高性能
    size_t bytes_written;
    size_t offset = 0;
    const size_t chunk_size = 2048;  // 使用2KB的块大小

    while (offset < data_size) {
        size_t current_chunk = (data_size - offset > chunk_size) ? chunk_size : (data_size - offset);

        esp_err_t ret = i2s_write(I2S_PORT, audio_data + offset, current_chunk, &bytes_written, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write audio data: %s", esp_err_to_name(ret));
            return ret;
        }

        offset += bytes_written;
    }

    return ESP_OK;
}

esp_err_t audio_player_deinit(void)
{
    // 释放 PSRAM 中的音频数据
    if (wifi_connect_psram) {
        heap_caps_free(wifi_connect_psram);
        wifi_connect_psram = NULL;
    }
    if (wifi_beak_psram) {
        heap_caps_free(wifi_beak_psram);
        wifi_beak_psram = NULL;
    }
    if (wifi_reset_psram) {
        heap_caps_free(wifi_reset_psram);
        wifi_reset_psram = NULL;
    }

    esp_err_t ret = i2s_driver_uninstall(I2S_PORT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to uninstall I2S driver: %s", esp_err_to_name(ret));
        return ret;
    }

    i2s_initialized = false;
    ESP_LOGI(TAG, "Audio player deinitialized successfully");
    return ESP_OK;
}