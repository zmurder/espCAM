#ifndef UDP_CAMERA_CLIENT_H
#define UDP_CAMERA_CLIENT_H

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

#endif /* UDP_CAMERA_CLIENT_H */