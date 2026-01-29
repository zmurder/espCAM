/*
 * audio_player.c
 * 音频播放器实现 - 使用 I2S 驱动 MAX98357 DAC
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "audio_player.h"

#include "wifi_connect.h"
#include "wifi_beak.h"
#include "wifi_reset.h"

static const char* TAG = "AUDIO_PLAYER";

// I2S 配置
#define I2S_NUM I2S_NUM_0
#define SAMPLE_RATE 16000
#define I2S_BCLK_IO 19
#define I2S_WS_IO 20
#define I2S_DOUT_IO 47
#define I2S_MCLK_IO -1

static i2s_chan_handle_t tx_chan = NULL;
static bool i2s_initialized = false;

// 音频数据数组 - 定义在 res/wifi_connect_audio.c, res/wifi_beak_audio.c, res/wifi_reset_audio.c
extern const uint8_t wifi_connect_audio[];
extern const uint8_t wifi_beak_audio[];
extern const uint8_t wifi_reset_audio[];

esp_err_t audio_player_init(void)
{
    if (i2s_initialized) {
        ESP_LOGW(TAG, "Audio player already initialized");
        return ESP_OK;
    }

    esp_err_t ret;

    // Step 1: 创建 I2S 通道
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ret = i2s_new_channel(&chan_cfg, &tx_chan, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Step 2: 配置标准 I2S 模式
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg =
            {
                .mclk = I2S_MCLK_IO,
                .bclk = I2S_BCLK_IO,
                .ws = I2S_WS_IO,
                .dout = I2S_DOUT_IO,
                .din = I2S_GPIO_UNUSED,
                .invert_flags =
                    {
                        .mclk_inv = false,
                        .bclk_inv = false,
                        .ws_inv = false,
                    },
            },
    };

    // Step 3: 初始化 I2S 通道
    ret = i2s_channel_init_std_mode(tx_chan, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2S std mode: %s", esp_err_to_name(ret));
        return ret;
    }

    // Step 4: 启用 I2S 通道
    ret = i2s_channel_enable(tx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }

    i2s_initialized = true;
    ESP_LOGI(TAG, "Audio player initialized successfully");
    return ESP_OK;
}

esp_err_t audio_player_deinit(void)
{
    if (!i2s_initialized) {
        ESP_LOGW(TAG, "Audio player not initialized");
        return ESP_OK;
    }

    esp_err_t ret;

    // 禁用 I2S 通道
    ret = i2s_channel_disable(tx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // 删除 I2S 通道
    ret = i2s_del_channel(tx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }

    tx_chan = NULL;
    i2s_initialized = false;
    ESP_LOGI(TAG, "Audio player deinitialized successfully");
    return ESP_OK;
}

esp_err_t audio_player_play(uint8_t* audio_data, size_t data_size)
{
    if (audio_data == NULL || data_size == 0) {
        ESP_LOGE(TAG, "Invalid audio data");
        return ESP_ERR_INVALID_ARG;
    }

    if (!i2s_initialized) {
        ESP_LOGE(TAG, "Audio player not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // 将 8-bit PCM 转换为 16-bit PCM
    uint16_t* buffer = malloc(data_size * 2);
    if (buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate audio buffer");
        return ESP_ERR_NO_MEM;
    }

    for (size_t i = 0; i < data_size; i++) {
        // 将 8-bit unsigned 转换为 16-bit signed
        int16_t sample = ((int16_t)audio_data[i] - 128) * 256;
        buffer[i] = (uint16_t)sample;
    }

    // 播放转换后的数据
    size_t bytes_written;
    esp_err_t ret = i2s_channel_write(tx_chan, buffer, data_size * 2, &bytes_written, portMAX_DELAY);
    if (ret != ESP_OK) {
        free(buffer);
        ESP_LOGE(TAG, "Failed to write audio data: %s", esp_err_to_name(ret));
        return ret;
    }

    free(buffer);
    return ESP_OK;
}

esp_err_t audio_player_stop(void)
{
    // 新驱动中不需要清零 DMA 缓冲区，直接返回成功
    return ESP_OK;
}

esp_err_t audio_player_play_stream(uint8_t* audio_data, size_t data_size)
{
    if (audio_data == NULL || data_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!i2s_initialized) {
        ESP_LOGE(TAG, "Audio player not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // 将音频数据流式发送到I2S接口
    // 使用更大的块大小以提高性能
    size_t bytes_written;
    size_t offset = 0;
    const size_t chunk_size = 2048;  // 使用2KB的块大小

    while (offset < data_size) {
        size_t current_chunk = (data_size - offset > chunk_size) ? chunk_size : (data_size - offset);

        // 将 8-bit PCM 转换为 16-bit PCM
        uint16_t* buffer = malloc(current_chunk * 2);
        if (buffer == NULL) {
            ESP_LOGE(TAG, "Failed to allocate audio buffer");
            return ESP_ERR_NO_MEM;
        }

        for (size_t i = 0; i < current_chunk; i++) {
            // 将 8-bit unsigned 转换为 16-bit signed
            int16_t sample = ((int16_t)audio_data[offset + i] - 128) * 256;
            buffer[i] = (uint16_t)sample;
        }

        // 播放扩展后的数据
        esp_err_t ret = i2s_channel_write(tx_chan, buffer, current_chunk * 2, &bytes_written, portMAX_DELAY);
        if (ret != ESP_OK) {
            free(buffer);
            ESP_LOGE(TAG, "Failed to write audio data: %s", esp_err_to_name(ret));
            return ret;
        }

        offset += current_chunk;
        free(buffer);
    }

    return ESP_OK;
}

esp_err_t audio_player_play_wifi_status(int status)
{
    if (!i2s_initialized) {
        ESP_LOGE(TAG, "Audio player not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    const uint8_t* audio_data = NULL;
    size_t data_size = 0;

    switch (status) {
        case 0:  // WiFi 连接成功
            audio_data = wifi_connect_audio;
            data_size = WIFI_CONNECT_AUDIO_LEN;
            break;
        case 1:  // WiFi 连接失败
            audio_data = wifi_beak_audio;
            data_size = WIFI_BEAK_AUDIO_LEN;
            break;
        case 2:  // WiFi 重置
            audio_data = wifi_reset_audio;
            data_size = WIFI_RESET_AUDIO_LEN;
            break;
        default:
            ESP_LOGE(TAG, "Invalid WiFi status: %d", status);
            return ESP_ERR_INVALID_ARG;
    }

    if (audio_data == NULL || data_size == 0) {
        ESP_LOGE(TAG, "Audio data not available for status: %d", status);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Playing WiFi status audio: %d", status);
    return audio_player_play_stream((uint8_t*)audio_data, data_size);
}