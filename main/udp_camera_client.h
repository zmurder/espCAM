#ifndef UDP_CAMERA_CLIENT_H
#define UDP_CAMERA_CLIENT_H

#include "posture_model.h"

/**
 * @brief 发送图像通过UDP
 * @param fb 相机帧缓冲
 * @return esp_err_t
 */
esp_err_t send_image_via_udp(camera_fb_t* fb);

/**
 * @brief 启动UDP图像传输
 */
void start_udp_camera(void);

/**
 * @brief 停止UDP图像传输
 */
void stop_udp_camera(void);

/**
 * @brief 重启UDP图像传输（在WiFi重置后调用）
 */
void restart_udp_camera(void);

/**
 * @brief 获取当前帧率
 * @return 当前帧率 (FPS)
 */
float get_current_fps(void);

/**
 * @brief 获取总帧数
 * @return 总帧数
 */
uint32_t get_total_frames(void);

/**
 * @brief 发送姿态检测结果通过UDP
 * @param output 姿态检测结果
 */
void send_posture_result_via_udp(const posture_output_t* output);

#endif /* UDP_CAMERA_CLIENT_H */