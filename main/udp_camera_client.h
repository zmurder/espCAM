#ifndef UDP_CAMERA_CLIENT_H
#define UDP_CAMERA_CLIENT_H

#include "esp_err.h"
#include "esp_camera.h"
#include "posture_model.h"

/**
 * @brief 发送图像通过UDP
 */
esp_err_t send_image_via_udp(camera_fb_t* fb);

/**
 * @brief 启动 UDP 自动发现监听（PC 广播发现包 → 自动学习/更新目标 IP，幂等）
 */
void udp_discovery_start(void);

/**
 * @brief 主动推送检测时间段状态到 PC（posture_sched 在 SNTP 同步后调用）
 */
void udp_sched_push_state(void);

/**
 * @brief 启动UDP图像传输
 */
void start_udp_camera(void);

/**
 * @brief 停止UDP图像传输
 */
void stop_udp_camera(void);

/**
 * @brief 重启UDP图像传输
 */
void restart_udp_camera(void);

/**
 * @brief 获取当前帧率
 */
float get_current_fps(void);

/**
 * @brief 获取总帧数
 */
uint32_t get_total_frames(void);

/**
 * @brief 发送姿态检测结果通过UDP
 */
void send_posture_result_via_udp(const posture_output_t* output);

#endif /* UDP_CAMERA_CLIENT_H */