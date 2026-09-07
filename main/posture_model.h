/**
 * posture_model.h
 * 坐姿检测模型接口（6 关键点：左眼/右眼/左耳/右耳/左肩/右肩）
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

// 6 个关键点定义（与模型输出通道顺序一致）
typedef enum
{
    KEYPOINT_LEFT_EYE = 0,
    KEYPOINT_RIGHT_EYE = 1,
    KEYPOINT_LEFT_EAR = 2,
    KEYPOINT_RIGHT_EAR = 3,
    KEYPOINT_LEFT_SHOULDER = 4,
    KEYPOINT_RIGHT_SHOULDER = 5,
    KEYPOINT_COUNT = 6
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
    POSTURE_BAD_NECK = 1,      // 头部前倾（眼肩垂直距离/肩宽 比值偏低）
    POSTURE_BAD_SHOULDER = 2,  // 肩膀歪斜（双肩倾斜角超阈）
    POSTURE_NOT_DETECTED = 3,  // 关键点检测失败（不可信：双肩不可见或头部关键点全不可见）
    POSTURE_UNRELIABLE = 4,    // 检测几何不合理（同组连线近垂直等明显误检），本帧不判断
} posture_result_t;

// 头部定位来源（双眼不可见时用双耳兜底）
typedef enum
{
    HEAD_SRC_NONE = -1,  // 双眼双耳都不可见
    HEAD_SRC_EYES = 0,   // 用双眼
    HEAD_SRC_EARS = 1,   // 用双耳（低头遮挡眼时兜底）
} posture_head_source_t;

// 姿态检测结果
typedef struct
{
    posture_result_t result;
    posture_keypoint_data_t keypoints[KEYPOINT_COUNT];
    float ratio;              // 前倾指标：头(眼/耳)-肩 垂直距离 / 肩宽（归一化）
    float shoulder_tilt_deg;  // 双肩连线倾斜角（度；核心歪斜指标）
    float head_tilt_deg;      // 头部连线倾斜角（度；眼优先，眼不可见用耳）
    int head_source;          // 头部定位来源，见 posture_head_source_t
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
