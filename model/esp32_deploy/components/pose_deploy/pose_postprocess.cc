#include "pose_postprocess.h"

namespace pose {

PoseResult postprocess(const int8_t* heatmap_data, int exponent) {
    PoseResult r;
    const float scale = powf(2.0f, (float)exponent);  // int8 -> float (对称量化, zero_point=0)
    const int HW = HEATMAP_H * HEATMAP_W;

    // 逐关键点 argmax（int8 单调，等价 float argmax）
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
        r.kps[k].x = (float)col / (float)HEATMAP_W;
        r.kps[k].y = (float)row / (float)HEATMAP_H;
        r.kps[k].conf = (float)maxv * scale;   // dequantize，Sigmoid 输出 [0,1]
        r.kps[k].valid = r.kps[k].conf >= CONF_THRESH;
        if (r.kps[k].valid) r.n_valid++;
    }

    // 双肩连线倾斜（核心坐姿指标，索引 4/5）
    // 坐标系：x 向右，y 向下。双肩水平时 dx>0, dy=0 -> 0°
    if (r.kps[IDX_LEFT_SHOULDER].valid && r.kps[IDX_RIGHT_SHOULDER].valid) {
        float dx = r.kps[IDX_RIGHT_SHOULDER].x - r.kps[IDX_LEFT_SHOULDER].x;
        float dy = r.kps[IDX_RIGHT_SHOULDER].y - r.kps[IDX_LEFT_SHOULDER].y;
        r.shoulder_tilt_deg = atan2f(dy, dx) * 180.0f / (float)M_PI;
        r.shoulder_width_px = sqrtf((dx * 320.0f) * (dx * 320.0f)
                                  + (dy * 240.0f) * (dy * 240.0f));
    }

    // 头部连线倾斜：眼(0/1)优先，眼不可见(低头遮挡)则用耳(2/3)
    bool eyes_ok = r.kps[IDX_LEFT_EYE].valid && r.kps[IDX_RIGHT_EYE].valid;
    bool ears_ok = r.kps[IDX_LEFT_EAR].valid && r.kps[IDX_RIGHT_EAR].valid;
    int a, b;
    if (eyes_ok)      { a = IDX_LEFT_EYE;  b = IDX_RIGHT_EYE;  r.head_source = 0; }
    else if (ears_ok) { a = IDX_LEFT_EAR;  b = IDX_RIGHT_EAR;  r.head_source = 1; }
    else              { r.head_source = -1; }
    if (r.head_source >= 0) {
        float dx = r.kps[b].x - r.kps[a].x;
        float dy = r.kps[b].y - r.kps[a].y;
        r.head_tilt_deg = atan2f(dy, dx) * 180.0f / (float)M_PI;
        r.head_dist_px  = sqrtf((dx * 320.0f) * (dx * 320.0f)
                              + (dy * 240.0f) * (dy * 240.0f));
    }

    // reliable: 至少 3 点可见且含双肩（坐姿判断依赖双肩）
    bool shoulders_ok = r.kps[IDX_LEFT_SHOULDER].valid && r.kps[IDX_RIGHT_SHOULDER].valid;
    r.reliable = (r.n_valid >= 3) && shoulders_ok;
    return r;
}

} // namespace pose
