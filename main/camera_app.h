#ifndef CAMERA_APP_H
#define CAMERA_APP_H

#include "esp_err.h"

/**
 * @brief 初始化摄像头
 * 
 * @return esp_err_t 
 */
esp_err_t camera_init(void);

/**
 * @brief 捕获图像
 * 
 * @return esp_err_t 
 */
esp_err_t camera_capture(void);

#endif /* CAMERA_APP_H */