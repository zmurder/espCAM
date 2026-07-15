#include "pose_postprocess.h"

namespace pose {

PoseResult postprocess(const int8_t* heatmap_data, int exponent) {
    PoseResult r;
    const float scale = powf(2.0f, (float)exponent);  // int8 -> float (对称量化, zero_point=0)
    const int HW = HEATMAP_H * HEATMAP_W;

    // 逐关键点 argmax（在 int8 上，单调，等价于 float argmax）
    for (int k = 0; k < NUM_KP; k++) {
        const int8_t* hm = heatmap_data + k * HW;
        int8_t maxv = -128;
        int maxidx = 0;
        for (int i = 0; i < HW; i++) {
            if (hm[i] > maxv) {
                maxv = hm[i];
                maxidx = i;
            }
        }
        int row = maxidx / HEATMAP_W;
        int col = maxidx % HEATMAP_W;
        // 归一化到 [0,1]（相对 320×240 输入；heatmap 160×120 是 1/2，归一化值一致）
        r.kps[k].x = (float)col / (float)HEATMAP_W;
        r.kps[k].y = (float)row / (float)HEATMAP_H;
        r.kps[k].conf = (float)maxv * scale;   // dequantize，Sigmoid 输出 [0,1]
        r.kps[k].valid = r.kps[k].conf >= CONF_THRESH;
        if (r.kps[k].valid) r.n_valid++;
    }

    // 双肩连线倾斜角（left_shoulder[2] -> right_shoulder[3]）
    // 坐标系：x 向右，y 向下。双肩水平时 dx>0, dy=0 -> 0°
    if (r.kps[2].valid && r.kps[3].valid) {
        float dx = r.kps[3].x - r.kps[2].x;
        float dy = r.kps[3].y - r.kps[2].y;
        r.shoulder_tilt_deg = atan2f(dy, dx) * 180.0f / (float)M_PI;
        r.shoulder_width_px = sqrtf((dx * 320.0f) * (dx * 320.0f)
                                  + (dy * 240.0f) * (dy * 240.0f));
    }
    // 双眼连线倾斜角
    if (r.kps[0].valid && r.kps[1].valid) {
        float dx = r.kps[1].x - r.kps[0].x;
        float dy = r.kps[1].y - r.kps[0].y;
        r.eye_tilt_deg = atan2f(dy, dx) * 180.0f / (float)M_PI;
        r.eye_dist_px = sqrtf((dx * 320.0f) * (dx * 320.0f)
                            + (dy * 240.0f) * (dy * 240.0f));
    }

    r.reliable = r.n_valid >= 3;
    return r;
}

} // namespace pose
