#pragma once
#include <cstdint>
#include <cmath>

namespace pose {

// 4 关键点索引（与训练 kp_config 一致）
// 0=left_eye, 1=right_eye, 2=left_shoulder, 3=right_shoulder
constexpr int NUM_KP = 4;
constexpr int HEATMAP_H = 120;
constexpr int HEATMAP_W = 160;

// 置信度阈值：cam 实测 conf>=0.6 时 PCK 97-100%（眼睛 100%，肩膀 97-99.7%）
// conf<0.6 的 13% 肩膀是难检样本（侧身/遮挡/边缘），argmax 会跑偏，必须过滤
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
    float shoulder_tilt_deg = 0;   // 双肩连线倾斜角（度；正=右肩低/头左歪）
    float eye_tilt_deg = 0;        // 双眼连线倾斜角
    float shoulder_width_px = 0;   // 双肩像素距离（320×240 尺度）
    float eye_dist_px = 0;         // 双眼像素距离
    int n_valid = 0;
    bool reliable = false;         // >=3 点 valid 才可信
};

/**
 * @brief heatmap 后处理：argmax + dequantize + conf 阈值 + 坐姿指标
 *
 * 纯 heatmap argmax 后处理（无 offset）。argmax 直接在 int8 上做（量化单调，
 * 结果不受影响），仅峰值 conf 用 exponent dequantize。
 *
 * @param heatmap_data int8 数据指针, layout [NUM_KP, HEATMAP_H, HEATMAP_W]
 * @param exponent     ESP-DL tensor exponent（dequantize: float = int8 * 2^exponent）
 * @return PoseResult  4 关键点 + 坐姿指标
 */
PoseResult postprocess(const int8_t* heatmap_data, int exponent);

} // namespace pose
