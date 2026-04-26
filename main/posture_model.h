/**
 * posture_model.h
 * 坐姿检测模型接口
 */

#ifndef __POSTURE_MODEL_H__
#define __POSTURE_MODEL_H__

#include "esp_err.h"
#include "esp_camera.h"

// 坐姿检测测试模式：1=仅运行模型测试(test), 0=运行实际推理
#define POSTURE_TEST_MODE 1

#ifdef __cplusplus
extern "C" {
#endif

// 7个关键点定义（与模型输出顺序一致）
typedef enum
{
    KEYPOINT_LEFT_EYE = 0,
    KEYPOINT_RIGHT_EYE = 1,
    KEYPOINT_LEFT_EAR = 2,
    KEYPOINT_RIGHT_EAR = 3,
    KEYPOINT_NOSE = 4,
    KEYPOINT_LEFT_SHOULDER = 5,
    KEYPOINT_RIGHT_SHOULDER = 6,
    KEYPOINT_COUNT = 7
} posture_keypoint_enum_t;

// 关键点坐标
typedef struct
{
    float x;  // 归一化坐标 [0, 1]
    float y;
    float score;  // 置信度 [0, 1]
} posture_keypoint_data_t;

// 姿态判断结果
typedef enum
{
    POSTURE_OK = 0,            // 坐姿正常
    POSTURE_BAD_NECK = 1,      // 头部前倾/后仰
    POSTURE_BAD_SHOULDER = 2,  // 肩膀不平
    POSTURE_NOT_DETECTED = 3,  // 关键点检测失败
} posture_result_t;

// 姿态检测结果
typedef struct
{
    posture_result_t result;
    posture_keypoint_data_t keypoints[KEYPOINT_COUNT];
    float ratio;  // 眼睛距离/肩膀距离 比值
} posture_output_t;

/**
 * @brief 初始化坐姿检测模型
 * @return ESP_OK 成功
 * @return ESP_FAIL 失败
 */
esp_err_t posture_model_init(void);

/**
 * @brief 运行姿态检测推理
 * @param fb 摄像头帧缓冲（JPEG 格式）
 * @param output 推理输出结果
 * @return ESP_OK 成功
 * @return ESP_FAIL 失败
 */
esp_err_t posture_model_run_inference(camera_fb_t* fb, posture_output_t* output);

/**
 * @brief 释放模型资源
 */
void posture_model_deinit(void);

/**
 * @brief 获取上一帧的推理延迟（微秒）
 */
uint32_t posture_model_get_last_latency_us(void);

#ifdef __cplusplus
}
#endif

#endif  // __POSTURE_MODEL_H__
