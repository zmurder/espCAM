/**
 * posture_model.cpp
 * 坐姿检测模型 - 6 关键点（pose_model_6kp_v2.espdl）
 *
 * 前处理（与训练 dataset.py 对齐，见 model/esp32_deploy）:
 * 1. center crop（QVGA 320×240 已是 4:3，全图即可）
 * 2. resize 到 320×240（ImagePreprocessor 按模型 input shape 自动处理）
 * 3. ImageNet 归一化: mean=[123.675,116.28,103.53] std=[58.395,57.12,57.375]
 * 4. RGB（rgb_swap=false），HWC→CHW，按 input exponent 量化到 int8
 *
 * 后处理:
 * - 输出 (1,6,120,160) heatmap，6 通道: 左眼/右眼/左耳/右耳/左肩/右肩
 * - 每通道 int8 argmax，conf = max_int8 × 2^exponent（Sigmoid 输出，已在 [0,1]）
 * - 置信度阈值：眼/耳 conf<0.4、双肩 conf<0.2 视为不可信；layout 按 output_shape 自适应（NHWC/NCHW）
 * - 坐姿判断（4 条任意成立即不良，详见 judge_posture）：
 *   前倾(NECK) = 眼肩垂直距离/双眼距 或 耳肩垂直距离/双耳距 < FORWARD_RATIO 阈值（EYE/EAR_FORWARD_RATIO_MIN）；
 *   歪头(SHOULDER) = 双眼/双耳-双肩相对倾斜角 > HEAD_TILT_WARN 阈值（EYE/EAR_HEAD_TILT_WARN）
 * - 几何合理性预检：同组连线（双肩/双眼/双耳）近垂直属明显误检，超阈本帧不判断（POSTURE_UNRELIABLE）
 * - 距离门限：肩距过小（人太远）→ 定位误差占比过大，本帧不判断（POSTURE_TOO_FAR）
 */

#include <string.h>
#include <math.h>
#include <array>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "dl_image_jpeg.hpp"
#include "dl_image_preprocessor.hpp"
#include "dl_model_base.hpp"
#include "posture_model.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char* TAG = "POSTURE_MODEL";

// 模型嵌入 rodata（pose_model_6kp_v2.espdl，6 关键点）
extern const uint8_t pose_model_6kp_v2_espdl[] asm("_binary_pose_model_6kp_v2_espdl_start");

// 测试图嵌入 rodata（POSTURE_TEST_IMAGE_MODE=1 时用 model/320240.jpg 推理）
#if POSTURE_TEST_IMAGE_MODE
extern const uint8_t _binary_3202402_jpg_start[] asm("_binary_3202402_jpg_start");
extern const uint8_t _binary_3202402_jpg_end[] asm("_binary_3202402_jpg_end");
#endif

static dl::Model* s_model = NULL;
static dl::image::ImagePreprocessor* s_preprocessor = NULL;
static uint32_t s_last_latency_us = 0;

#define MODEL_INPUT_H 240
#define MODEL_INPUT_W 320

#define HEATMAP_H 120
#define HEATMAP_W 160

// 训练对齐：ImageNet 归一化（ESP-DL 要求 [0,255] 范围，已 ×255）
static const std::array<float, 3> IMAGENET_MEAN = {123.675f, 116.28f, 103.53f};
static const std::array<float, 3> IMAGENET_STD = {58.395f, 57.12f, 57.375f};

// 坐姿判断阈值（不良坐姿：下列 4 条任意成立即触发；详见 judge_posture）
#define CONF_THRESH 0.4f            // 关键点置信度阈值（眼/耳，<此值视为不可信）
#define SHOULDER_CONF_THRESH 0.3f   // 双肩单独阈值（更低；趴近时肩 conf 偏低但仍需作为参考基准）
#define EYE_FORWARD_RATIO_MIN 1.5f  // 条件1：眼肩垂直距离/双眼距离 < 此值 → 前倾（需据正常坐姿日志标定）
#define EAR_FORWARD_RATIO_MIN 1.5f  // 条件2：耳肩垂直距离/双耳距离 < 此值 → 前倾（需据正常坐姿日志标定）
#define EYE_HEAD_TILT_WARN 35.0f    // 条件3：双眼-双肩相对倾斜角 > 此值 → 歪头（度）
#define EAR_HEAD_TILT_WARN 35.0f    // 条件4：双耳-双肩相对倾斜角 > 此值 → 歪头（度）
// 几何合理性预检（防误检误报）：正常坐姿下同组连线接近水平，|倾斜角| 超阈（如接近垂直）视为明显误检
#define SHOULDER_LINE_TILT_MAX 40.0f  // 双肩连线 |倾斜角| > 此值 → 双肩误检，本帧不判断（度）
#define EYE_LINE_TILT_MAX 40.0f       // 双眼连线 |倾斜角| > 此值 → 该组误检，跳过眼条件（度）
#define EAR_LINE_TILT_MAX 40.0f       // 双耳连线 |倾斜角| > 此值 → 该组误检，跳过耳条件（度）
#define SHOULDER_DIST_MIN 0.19f       // 肩距(归一化) < 此值 → 人太远（heatmap 定位误差占比大，远距离误报根源），本帧不判断（需据远距离日志标定）

// 解析热力图：每关键点 int8 argmax + 亚像素质心 + dequantize conf
// chan_dim: 通道维在 4D shape [N,?, ?, ?] 中的位置 —— 1=NCHW(旧参考)，3=NHWC(ESP-DL 实测)

// 按 layout 取 heatmap (x, y, k) 处的 int8 值（chan_dim 语义同 parse_heatmaps）
static inline int8_t hm_at(const int8_t* hm, int chan_dim, int x, int y, int k)
{
    if (chan_dim == 1) {
        return hm[(k * HEATMAP_H + y) * HEATMAP_W + x];   // NCHW [1,C,H,W]
    }
    return hm[(y * HEATMAP_W + x) * KEYPOINT_COUNT + k];  // NHWC [1,H,W,C]
}
static void parse_heatmaps(const int8_t* quant_heatmaps, int exponent, int chan_dim, posture_keypoint_data_t* keypoints)
{
    const float scale = powf(2.0f, (float)exponent);  // 对称量化: float = int8 × 2^exp

    for (int k = 0; k < KEYPOINT_COUNT; k++) {
        int8_t maxv = -128;
        int max_row = 0, max_col = 0;
        for (int y = 0; y < HEATMAP_H; y++) {
            for (int x = 0; x < HEATMAP_W; x++) {
                int idx;
                if (chan_dim == 1) {
                    // NCHW [1, C, H, W]：index = ((k*H + y)*W + x)
                    idx = (k * HEATMAP_H + y) * HEATMAP_W + x;
                }
                else {
                    // NHWC [1, H, W, C]：index = ((y*W + x)*C + k)
                    idx = (y * HEATMAP_W + x) * KEYPOINT_COUNT + k;
                }
                int8_t v = quant_heatmaps[idx];
                if (v > maxv) {
                    maxv = v;
                    max_row = y;
                    max_col = x;
                }
            }
        }

        // 亚像素峰值定位：argmax 只有 ±1 格精度（≈输入 2px），远距离小目标误差占比过大
        //（双眼仅隔数格时 ±1 格 = 20% 噪声）。用峰及左右/上下邻居的正值做加权质心，
        // 把定位细化到格子内部（~±0.3 格）。邻居 ≤0（int8 背景为负/零）或峰贴边时不参与，退化为整数格
        float sub_x = 0.0f, sub_y = 0.0f;
        if (max_col > 0 && max_col < HEATMAP_W - 1) {
            float vl = hm_at(quant_heatmaps, chan_dim, max_col - 1, max_row, k);
            float vr = hm_at(quant_heatmaps, chan_dim, max_col + 1, max_row, k);
            if (vl > 0 && vr > 0)
                sub_x = (vr - vl) / (vl + (float)maxv + vr);
        }
        if (max_row > 0 && max_row < HEATMAP_H - 1) {
            float vu = hm_at(quant_heatmaps, chan_dim, max_col, max_row - 1, k);
            float vd = hm_at(quant_heatmaps, chan_dim, max_col, max_row + 1, k);
            if (vu > 0 && vd > 0)
                sub_y = (vd - vu) / (vu + (float)maxv + vd);
        }

        // 归一化到 [0,1]（相对 320×240 模型输入；heatmap 160×120 = 输入 1/2，归一化值等价）
        keypoints[k].x = ((float)max_col + sub_x) / (float)HEATMAP_W;
        keypoints[k].y = ((float)max_row + sub_y) / (float)HEATMAP_H;
        keypoints[k].score = (float)maxv * scale;  // dequantize（Sigmoid 输出，已在 [0,1]）
        keypoints[k].valid = keypoints[k].score >= CONF_THRESH;

        ESP_LOGI(TAG, "  kp[%d]: hm=(%d,%d) norm=(%.3f,%.3f) conf=%.3f", k, max_col, max_row, keypoints[k].x, keypoints[k].y, keypoints[k].score);
    }
}

static inline float dist_norm(float x0, float y0, float x1, float y1)
{
    float dx = x1 - x0;
    float dy = y1 - y0;
    return sqrtf(dx * dx + dy * dy);
}

// 把 atan2 角度归一化到 [-90,90]：面对镜头时连线 dx<0，水平时 atan2 返回 ±180°，
// 必须映射为 0°，否则"水平"会被误判为大倾斜（曾导致 eye_tilt≈160° 永远超阈）
static inline float normalize_tilt(float deg)
{
    if (deg > 90.0f)
        deg -= 180.0f;
    else if (deg < -90.0f)
        deg += 180.0f;
    return deg;
}

// 不良坐姿判断（6 关键点）：以下 4 条任意成立即判不良；某条所需关键点不可见时跳过该条。
//  可靠前提：双肩可见（垂直距离与相对角度的基准）且至少一组头部(眼或耳)可见，否则 POSTURE_NOT_DETECTED。
//  几何合理性预检（防误检误报）：正常坐姿下同组连线接近水平；双肩连线不合理（或眼+耳全不合理）时
//  关键点属明显误检（如连线近垂直）→ POSTURE_UNRELIABLE 本帧不判断；仅一组头部连线不合理则跳过该组条件。
//  距离门限：肩距（归一化）过小 → 人太远，heatmap 定位误差占比过大 → POSTURE_TOO_FAR 本帧不判断。
//  前倾(BAD_NECK)：
//   1) 眼肩垂直距离 / 双眼距离 < EYE_FORWARD_RATIO_MIN   （双眼可见时评估）
//   2) 耳肩垂直距离 / 双耳距离 < EAR_FORWARD_RATIO_MIN   （双耳可见时评估）
//  歪头(BAD_SHOULDER)：
//   3) 双眼-双肩相对倾斜角 > EYE_HEAD_TILT_WARN          （双眼可见时评估）
//   4) 双耳-双肩相对倾斜角 > EAR_HEAD_TILT_WARN          （双耳可见时评估）
static posture_result_t judge_posture(posture_keypoint_data_t* kp, float* out_ratio, float* out_shoulder_tilt, float* out_head_tilt, int* out_head_source)
{
    // 可见性（成对）。双肩用单独更低阈值：趴近时肩 conf 偏低，但仍需作为垂直距离/相对角度的基准
    bool shoulders_ok = (kp[KEYPOINT_LEFT_SHOULDER].score >= SHOULDER_CONF_THRESH) && (kp[KEYPOINT_RIGHT_SHOULDER].score >= SHOULDER_CONF_THRESH);
    bool eyes_ok = kp[KEYPOINT_LEFT_EYE].valid && kp[KEYPOINT_RIGHT_EYE].valid;
    bool ears_ok = kp[KEYPOINT_LEFT_EAR].valid && kp[KEYPOINT_RIGHT_EAR].valid;

    // 可靠前提：双肩可见 && (双眼或双耳可见)
    if (!shoulders_ok || (!eyes_ok && !ears_ok)) {
        int n_valid = 0;
        for (int i = 0; i < KEYPOINT_COUNT; i++)
            if (kp[i].valid) n_valid++;
        ESP_LOGI(TAG, "not reliable: shoulders_ok=%d eyes_ok=%d ears_ok=%d n_valid=%d", (int)shoulders_ok, (int)eyes_ok, (int)ears_ok, n_valid);
        return POSTURE_NOT_DETECTED;
    }

    // 双肩连线角度（归一化到[-90,90]）+ 肩 y 均值（垂直距离基准）
    float sdx = kp[KEYPOINT_RIGHT_SHOULDER].x - kp[KEYPOINT_LEFT_SHOULDER].x;
    float sdy = kp[KEYPOINT_RIGHT_SHOULDER].y - kp[KEYPOINT_LEFT_SHOULDER].y;
    *out_shoulder_tilt = normalize_tilt(atan2f(sdy, sdx) * 180.0f / (float)M_PI);
    float shoulder_y_avg = (kp[KEYPOINT_LEFT_SHOULDER].y + kp[KEYPOINT_RIGHT_SHOULDER].y) * 0.5f;

    // 预检⓪ 距离门限：肩距过小说明人太远，比值分母/角度分子都只有几个 heatmap 格子宽，
    // 定位噪声占比过大（远距离误报根源）→ 本帧不判断
    float shoulder_dist = sqrtf(sdx * sdx + sdy * sdy);
    ESP_LOGI(TAG, "  shoulder_dist=%.3f (min %.3f)", shoulder_dist, (double)SHOULDER_DIST_MIN);  // 每帧打印，供标定 SHOULDER_DIST_MIN
    if (shoulder_dist < SHOULDER_DIST_MIN) {
        ESP_LOGW(TAG, "shoulder_dist=%.3f < %.3f, subject too far, skip frame", shoulder_dist, (double)SHOULDER_DIST_MIN);
        return POSTURE_TOO_FAR;
    }

    // 预检①：双肩是全部判断的基准，连线近垂直属明显误检 → 本帧不判断
    if (fabsf(*out_shoulder_tilt) > SHOULDER_LINE_TILT_MAX) {
        ESP_LOGW(TAG, "shoulder line tilt=%.1f > %.1f, unreliable detection, skip frame", *out_shoulder_tilt, (double)SHOULDER_LINE_TILT_MAX);
        return POSTURE_UNRELIABLE;
    }

    // 预检②：眼/耳组连线近垂直 → 该组视为误检，跳过其条件（另一组可用则照常判断）
    float eye_tilt = 0.0f, ear_tilt = 0.0f;
    bool eyes_usable = eyes_ok;
    bool ears_usable = ears_ok;
    if (eyes_ok) {
        eye_tilt = normalize_tilt(atan2f(kp[KEYPOINT_RIGHT_EYE].y - kp[KEYPOINT_LEFT_EYE].y, kp[KEYPOINT_RIGHT_EYE].x - kp[KEYPOINT_LEFT_EYE].x) * 180.0f / (float)M_PI);
        if (fabsf(eye_tilt) > EYE_LINE_TILT_MAX) {
            ESP_LOGW(TAG, "eye line tilt=%.1f > %.1f, eye unreliable, skip eye conditions", eye_tilt, (double)EYE_LINE_TILT_MAX);
            eyes_usable = false;
        }
    }
    if (ears_ok) {
        ear_tilt = normalize_tilt(atan2f(kp[KEYPOINT_RIGHT_EAR].y - kp[KEYPOINT_LEFT_EAR].y, kp[KEYPOINT_RIGHT_EAR].x - kp[KEYPOINT_LEFT_EAR].x) * 180.0f / (float)M_PI);
        if (fabsf(ear_tilt) > EAR_LINE_TILT_MAX) {
            ESP_LOGW(TAG, "ear line tilt=%.1f > %.1f, ear unreliable, skip ear conditions", ear_tilt, (double)EAR_LINE_TILT_MAX);
            ears_usable = false;
        }
    }
    if (!eyes_usable && !ears_usable) {  // 头部两组连线全部不合理 → 检测不可信
        return POSTURE_UNRELIABLE;
    }

    // 默认输出（可靠前提下至少一组头部连线可用，会被下方覆盖）
    *out_head_source = HEAD_SRC_NONE;
    *out_head_tilt = 0.0f;
    *out_ratio = 0.0f;

    // 条件 1 & 3：双眼连线可用 → 前倾比 + 歪头角
    if (eyes_usable) {
        float edx = kp[KEYPOINT_RIGHT_EYE].x - kp[KEYPOINT_LEFT_EYE].x;
        float edy = kp[KEYPOINT_RIGHT_EYE].y - kp[KEYPOINT_LEFT_EYE].y;
        float eye_dist = sqrtf(edx * edx + edy * edy);
        float eye_y_avg = (kp[KEYPOINT_LEFT_EYE].y + kp[KEYPOINT_RIGHT_EYE].y) * 0.5f;
        float eye_forward_ratio = (eye_dist > 0.001f) ? ((shoulder_y_avg - eye_y_avg) / eye_dist) : 0.0f;  // 条件1
        float eye_head_rel = normalize_tilt(eye_tilt - *out_shoulder_tilt);                                // 条件3

        *out_head_source = HEAD_SRC_EYES;
        *out_head_tilt = eye_head_rel;
        *out_ratio = eye_forward_ratio;
        ESP_LOGI(TAG, "  [eye] forward_ratio=%.2f (thr<%.2f)  head_rel=%.1f (thr>%.1f)", eye_forward_ratio, (double)EYE_FORWARD_RATIO_MIN, eye_head_rel, (double)EYE_HEAD_TILT_WARN);

        if (eye_forward_ratio < EYE_FORWARD_RATIO_MIN) return POSTURE_BAD_NECK;     // 条件1
        if (fabsf(eye_head_rel) > EYE_HEAD_TILT_WARN) return POSTURE_BAD_SHOULDER;  // 条件3
    }

    // 条件 2 & 4：双耳连线可用 → 前倾比 + 歪头角（与双眼平行评估；眼不可用时兜底）
    if (ears_usable) {
        float edx = kp[KEYPOINT_RIGHT_EAR].x - kp[KEYPOINT_LEFT_EAR].x;
        float edy = kp[KEYPOINT_RIGHT_EAR].y - kp[KEYPOINT_LEFT_EAR].y;
        float ear_dist = sqrtf(edx * edx + edy * edy);
        float ear_y_avg = (kp[KEYPOINT_LEFT_EAR].y + kp[KEYPOINT_RIGHT_EAR].y) * 0.5f;
        float ear_forward_ratio = (ear_dist > 0.001f) ? ((shoulder_y_avg - ear_y_avg) / ear_dist) : 0.0f;  // 条件2
        float ear_head_rel = normalize_tilt(ear_tilt - *out_shoulder_tilt);                                // 条件4

        if (*out_head_source == HEAD_SRC_NONE) {  // 眼不可见时用耳填充输出
            *out_head_source = HEAD_SRC_EARS;
            *out_head_tilt = ear_head_rel;
            *out_ratio = ear_forward_ratio;
        }
        ESP_LOGI(TAG, "  [ear] forward_ratio=%.2f (thr<%.2f)  head_rel=%.1f (thr>%.1f)", ear_forward_ratio, (double)EAR_FORWARD_RATIO_MIN, ear_head_rel, (double)EAR_HEAD_TILT_WARN);

        if (ear_forward_ratio < EAR_FORWARD_RATIO_MIN) return POSTURE_BAD_NECK;     // 条件2
        if (fabsf(ear_head_rel) > EAR_HEAD_TILT_WARN) return POSTURE_BAD_SHOULDER;  // 条件4
    }

    const char* src = (*out_head_source == HEAD_SRC_EYES) ? "眼" : (*out_head_source == HEAD_SRC_EARS) ? "耳" : "无";
    ESP_LOGI(TAG, "[6 kp] OK shoulder_tilt=%.1f head_rel=%.1f(来自%s) forward_ratio=%.2f", *out_shoulder_tilt, *out_head_tilt, src, *out_ratio);
    return POSTURE_OK;
}

esp_err_t posture_model_init(void)
{
    ESP_LOGI(TAG, "Initializing posture model (pose_model_6kp_v2, 6 keypoints)...");

    // 创建模型：param_copy=true 把参数拷到 PSRAM（8MB PSRAM 充裕），
    // 否则 param_copy=false 每个卷积都要从 flash 读权重，推理会慢到 40s+ 触发 watchdog
    s_model = new dl::Model((const char*)pose_model_6kp_v2_espdl,
                            fbs::MODEL_LOCATION_IN_FLASH_RODATA,
                            0,                          // max_internal_size
                            dl::MEMORY_MANAGER_GREEDY,  // mm_type
                            nullptr,                    // key
                            true);                      // param_copy=true
    if (s_model == nullptr) {
        ESP_LOGE(TAG, "Failed to create model instance");
        return ESP_FAIL;
    }

    // ImagePreprocessor：ImageNet 归一化 + RGB（rgb_swap=false），不启用 letterbox
    s_preprocessor = new dl::image::ImagePreprocessor(s_model, IMAGENET_MEAN, IMAGENET_STD, false);

    // 打印模型输入信息
    std::map<std::string, dl::TensorBase*> model_inputs = s_model->get_inputs();
    dl::TensorBase* model_input = model_inputs.begin()->second;
    ESP_LOGI(TAG, "Model input: shape=[%d,%d,%d,%d], exponent=%d", model_input->shape[0], model_input->shape[1], model_input->shape[2], model_input->shape[3], (int)model_input->exponent);

    ESP_LOGI(TAG, "Free heap: %lu bytes, Free PSRAM: %lu bytes", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DEFAULT), (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    ESP_LOGI(TAG, "Model initialized successfully");
    return ESP_OK;
}

esp_err_t posture_model_run_inference(camera_fb_t* fb, posture_output_t* output)
{
    if (s_model == NULL || output == NULL) {
        return ESP_FAIL;
    }

    memset(output, 0, sizeof(posture_output_t));

    uint64_t start_time = esp_timer_get_time();

    // 图像来源：测试图模式用嵌入的 320240.jpg，否则用摄像头帧
    const void* img_data;
    size_t img_len;
#if POSTURE_TEST_IMAGE_MODE
    img_data = (const void*)_binary_3202402_jpg_start;
    img_len = (size_t)(_binary_3202402_jpg_end - _binary_3202402_jpg_start);
    ESP_LOGI(TAG, "Using TEST IMAGE, %d bytes", (int)img_len);
#else
    if (fb == NULL) {
        return ESP_FAIL;
    }
    img_data = (const void*)fb->buf;
    img_len = fb->len;
#endif

    // JPEG 解码为 RGB888
    dl::image::jpeg_img_t jpeg_img;
    jpeg_img.data = (void*)img_data;
    jpeg_img.data_len = img_len;

    auto img = dl::image::sw_decode_jpeg(jpeg_img, dl::image::DL_IMAGE_PIX_TYPE_RGB888);
    if (img.data == nullptr) {
        ESP_LOGE(TAG, "JPEG decode failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Image decoded: %dx%d", img.width, img.height);

    // 前处理: center crop(空=全图) + resize + ImageNet 归一化 + HWC→CHW + 量化
    s_preprocessor->preprocess(img);

    // 模型推理（单核：实测双核对 YOLO 串行模型无效——断 wifi 验证仍 ~10s）
    s_model->run();

    // 获取输出 heatmap (1,6,120,160)
    std::map<std::string, dl::TensorBase*> model_outputs = s_model->get_outputs();
    dl::TensorBase* model_output = model_outputs.begin()->second;

    std::vector<int> output_shape = model_output->get_shape();
    int8_t* quant_heatmaps = (int8_t*)model_output->data;
    int output_exp = model_output->exponent;

    ESP_LOGI(TAG, "Model output: shape=[%d,%d,%d,%d], exponent=%d", output_shape[0], output_shape[1], output_shape[2], output_shape[3], output_exp);

    // 定位通道维：H=120,W=160,C=6 互异，shape 中值为 KEYPOINT_COUNT(6) 的维度即通道维
    // chan_dim==1 → NCHW [1,C,H,W]；chan_dim==3 → NHWC [1,H,W,C]（ESP-DL 实测）
    int chan_dim = 3;  // 默认 NHWC（与旧 4 点模型一致）
    for (int d = 1; d < 4; d++) {
        if (output_shape[d] == KEYPOINT_COUNT) {
            chan_dim = d;
            break;
        }
    }
    ESP_LOGI(TAG, "heatmap layout: chan_dim=%d (%s)", chan_dim, chan_dim == 1 ? "NCHW" : "NHWC");

    // 解析热力图 → 6 关键点
    parse_heatmaps(quant_heatmaps, output_exp, chan_dim, output->keypoints);

    // 姿态判断
    output->result = judge_posture(output->keypoints, &output->ratio, &output->shoulder_tilt_deg, &output->head_tilt_deg, &output->head_source);

    uint64_t end_time = esp_timer_get_time();
    s_last_latency_us = (uint32_t)(end_time - start_time);

    ESP_LOGI(TAG, "Inference done in %lu us, result=%d", (unsigned long)s_last_latency_us, output->result);

    heap_caps_free(img.data);
    return ESP_OK;
}

void posture_model_deinit(void)
{
    if (s_preprocessor != nullptr) {
        delete s_preprocessor;
        s_preprocessor = nullptr;
    }
    if (s_model != nullptr) {
        delete s_model;
        s_model = nullptr;
    }
    ESP_LOGI(TAG, "Model deinitialized");
}

uint32_t posture_model_get_last_latency_us(void)
{
    return s_last_latency_us;
}
