#ifndef AUDIO_PLAYER_H
#define AUDIO_PLAYER_H

#include "esp_err.h"

/**
 * @brief 初始化音频播放功能
 * @return esp_err_t
 */
esp_err_t audio_player_init(void);

/**
 * @brief 播放音频数据
 * @param audio_data 音频数据
 * @param data_size 音频数据大小
 * @return esp_err_t
 */
esp_err_t audio_player_play_data(uint8_t* audio_data, size_t data_size);

/**
 * @brief 播放WiFi状态语音提示
 * @param status 0: 连接成功, 1: 连接失败, 2: WiFi重置
 * @return esp_err_t
 */
esp_err_t audio_player_play_wifi_status(int status);

/**
 * @brief 停止音频播放并清除缓冲区
 * @return esp_err_t
 */
esp_err_t audio_player_stop(void);

/**
 * @brief 播放连续的音频流（用于接收PC发送的语音数据）
 * @param audio_data 音频数据
 * @param data_size 音频数据大小
 * @return esp_err_t
 */
esp_err_t audio_player_play_stream(uint8_t* audio_data, size_t data_size);

/**
 * @brief 去初始化音频播放功能
 * @return esp_err_t
 */
esp_err_t audio_player_deinit(void);

#endif /* AUDIO_PLAYER_H */