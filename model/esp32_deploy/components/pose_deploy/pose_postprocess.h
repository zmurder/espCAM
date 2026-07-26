#pragma once
#include <cstdint>
#include <cmath>

namespace pose {

// 6 关键点索引（与训练 kp_config 一致）
// 0=left_eye, 1=right_eye, 2=left_ear, 3=right_ear,
// 4=left_shoulder, 5=right_shoulder
constexpr int NUM_KP = 6;
constexpr int IDX_LEFT_EYE       = 0;
constexpr int IDX_RIGHT_EYE      = 1;
constexpr int IDX_LEFT_EAR       = 2;
constexpr int IDX_RIGHT_EAR      = 3;
constexpr int IDX_LEFT_SHOULDER  = 4;
constexpr int IDX_RIGHT_SHOULDER = 5;

constexpr int HEATMAP_H = 120;
constexpr int HEATMAP_W = 160;

// 置信度阈值：cam 实测 conf>=0.6 时 PCK 高（眼睛满分、肩膀 94-96%）
constexpr float CONF_THRESH = 0.6f;

// 坐姿判断阈值
constexpr float TILT_WARN_DEG = 10.0f;  // 双肩倾斜 >10° 判为坐姿歪斜

struct Keypoint {
    float x;       // 归一化 [0,1]（相对 320×240 模型输入）
    float y;
    float conf;    // Sigmoid 输出 [0,1]
    bool valid;    // conf >= CONF_THRESH
};

struct PoseResult {
    Keypoint kps[NUM_KP];
    float shoulder_tilt_deg = 0;   // 双肩连线倾斜角（度；核心坐姿指标）
    float head_tilt_deg = 0;       // 头部连线倾斜（眼优先，眼不可见则用耳）
    float shoulder_width_px = 0;   // 双肩像素距离（320×240 尺度）
    float head_dist_px = 0;        // 双眼或双耳像素距离
    int  head_source = 0;          // 0=eyes, 1=ears, -1=双眼双耳都不够
    int n_valid = 0;
    bool reliable = false;         // >=3 点 valid 且含双肩才可信
};

/**
 * @brief heatmap 后处理：argmax + dequantize + conf 阈值 + 坐姿指标
 *
 * 纯 heatmap argmax（无 offset）。argmax 在 int8 上做（量化单调），峰值 conf 用 exponent dequantize。
 * 头部定位：眼睛优先（0/1），低头看不到眼时用双耳（2/3）兜底。
 *
 * @param heatmap_data int8 数据指针, layout [NUM_KP, HEATMAP_H, HEATMAP_W]
 * @param exponent     ESP-DL tensor exponent（dequantize: float = int8 * 2^exponent）
 * @return PoseResult  6 关键点 + 坐姿指标
 */
PoseResult postprocess(const int8_t* heatmap_data, int exponent);

} // namespace pose
