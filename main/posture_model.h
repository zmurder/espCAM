/**
 * posture_model.h
 * 坐姿检测模型接口（4 关键点 COCO Pose：左眼/右眼/左肩/右肩）
 */

#ifndef __POSTURE_MODEL_H__
#define __POSTURE_MODEL_H__

#include "esp_err.h"
#include "esp_camera.h"
#include <stdbool.h>

// 坐姿检测测试模式：1=仅运行模型 test(), 0=运行实际推理
#define POSTURE_TEST_MODE 0

// 静态测试图模式：1=用嵌入的 model/320240.jpg 推理(不依赖摄像头实时帧)，0=用摄像头帧
// 用于排查"摄像头输入/处理"问题：固定输入图，对比 PC onnx 结果
#define POSTURE_TEST_IMAGE_MODE 0

#ifdef __cplusplus
extern "C" {
#endif

// 4 个关键点定义（与模型输出通道顺序一致）
typedef enum
{
    KEYPOINT_LEFT_EYE = 0,
    KEYPOINT_RIGHT_EYE = 1,
    KEYPOINT_LEFT_SHOULDER = 2,
    KEYPOINT_RIGHT_SHOULDER = 3,
    KEYPOINT_COUNT = 4
} posture_keypoint_enum_t;

// 关键点坐标
typedef struct
{
    float x;       // 归一化坐标 [0, 1]（相对 320×240 模型输入）
    float y;
    float score;   // 置信度 [0, 1]（conf = max_int8 × 2^exponent）
    bool valid;    // score >= CONF_THRESH
} posture_keypoint_data_t;

// 姿态判断结果
typedef enum
{
    POSTURE_OK = 0,            // 坐姿正常
    POSTURE_BAD_NECK = 1,      // 头部前倾（眼距/肩距比值超阈）
    POSTURE_BAD_SHOULDER = 2,  // 肩膀歪斜（双肩倾斜角超阈）
    POSTURE_NOT_DETECTED = 3,  // 关键点检测失败（conf 不足）
} posture_result_t;

// 姿态检测结果
typedef struct
{
    posture_result_t result;
    posture_keypoint_data_t keypoints[KEYPOINT_COUNT];
    float ratio;              // 眼睛距离 / 肩膀距离 比值
    float shoulder_tilt_deg;  // 双肩连线倾斜角（度）
    float eye_tilt_deg;       // 双眼连线倾斜角（度）
} posture_output_t;

/**
 * @brief 初始化坐姿检测模型
 */
esp_err_t posture_model_init(void);

/**
 * @brief 运行姿态检测推理（fb 为 JPEG 帧）
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
