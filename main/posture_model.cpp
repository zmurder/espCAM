/**
 * posture_model.cpp
 * 坐姿检测模型 - 4 关键点 COCO Pose（pose_model.espdl）
 *
 * 前处理（与训练 dataset.py 对齐，见 model/esp32_deploy）:
 * 1. center crop（QVGA 320×240 已是 4:3，全图即可）
 * 2. resize 到 320×240（ImagePreprocessor 按模型 input shape 自动处理）
 * 3. ImageNet 归一化: mean=[123.675,116.28,103.53] std=[58.395,57.12,57.375]
 * 4. RGB（rgb_swap=false），HWC→CHW，按 input exponent 量化到 int8
 *
 * 后处理:
 * - 输出 (1,4,120,160) heatmap，4 通道: 左眼/右眼/左肩/右肩
 * - 每通道 int8 argmax，conf = max_int8 × 2^exponent
 * - conf < 0.6 过滤；眼距/肩距比值判前倾(NECK)，双肩倾斜角判歪斜(SHOULDER)
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

// 模型嵌入 rodata（pose_model_int8_final.espdl）
extern const uint8_t pose_model_int8_final_espdl[] asm("_binary_pose_model_int8_final_espdl_start");

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
static const std::array<float, 3> IMAGENET_STD  = {58.395f, 57.12f, 57.375f};

// 坐姿判断阈值
#define CONF_THRESH          0.6f    // 置信度阈值（cam 实测 conf≥0.6 PCK 97-100%）
#define SHOULDER_TILT_WARN   10.0f   // 双肩倾斜角告警阈值（度）
#define EYE_TILT_WARN        25.0f   // 双眼倾斜角告警（缺肩时判头部歪斜），度
#define EYE_SHOULDER_DY_MIN  0.15f   // 眼肩垂直距离下限（<此值判前倾），归一化坐标
#define EYE_Y_FORWARD_MAX    0.50f   // 缺肩时眼睛 y 前倾阈值（眼睛过低=前倾）

// 解析热力图：每关键点 int8 argmax + dequantize conf
static void parse_heatmaps(const int8_t* quant_heatmaps, int exponent,
                           posture_keypoint_data_t* keypoints)
{
    // 输出 layout [1, 120, 160, 4] = (batch, H, W, keypoint)，NHWC
    // 每像素 4 个关键点通道连续存放：index = (y*W + x)*C + k
    const float scale = powf(2.0f, (float)exponent);  // 对称量化: float = int8 × 2^exp

    for (int k = 0; k < KEYPOINT_COUNT; k++) {
        int8_t maxv = -128;
        int max_row = 0, max_col = 0;
        for (int y = 0; y < HEATMAP_H; y++) {
            for (int x = 0; x < HEATMAP_W; x++) {
                int idx = (y * HEATMAP_W + x) * KEYPOINT_COUNT + k;  // NHWC
                int8_t v = quant_heatmaps[idx];
                if (v > maxv) {
                    maxv = v;
                    max_row = y;
                    max_col = x;
                }
            }
        }

        // 归一化到 [0,1]（相对 320×240 模型输入；heatmap 160×120 = 输入 1/2，归一化值等价）
        keypoints[k].x = (float)max_col / (float)HEATMAP_W;
        keypoints[k].y = (float)max_row / (float)HEATMAP_H;
        keypoints[k].score = (float)maxv * scale;   // dequantize（Sigmoid 输出，已在 [0,1]）
        keypoints[k].valid = keypoints[k].score >= CONF_THRESH;

        ESP_LOGI(TAG, "  kp[%d]: hm=(%d,%d) norm=(%.3f,%.3f) conf=%.3f",
                 k, max_col, max_row, keypoints[k].x, keypoints[k].y, keypoints[k].score);
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
    if (deg > 90.0f)       deg -= 180.0f;
    else if (deg < -90.0f) deg += 180.0f;
    return deg;
}

// 姿态判断：双肩可信时用 比值(前倾)+倾斜角(歪斜)；
// 缺一肩（左肩 espdl 量化峰值偏移、位置不可信）时退化用双眼判断
static posture_result_t judge_posture(posture_keypoint_data_t* kp,
                                      float* out_ratio, float* out_shoulder_tilt, float* out_eye_tilt)
{
    // 双眼必须可信（最可靠的关键点）
    if (!kp[KEYPOINT_LEFT_EYE].valid || !kp[KEYPOINT_RIGHT_EYE].valid) {
        ESP_LOGI(TAG, "eyes not reliable (conf<%.1f), skip", CONF_THRESH);
        return POSTURE_NOT_DETECTED;
    }

    // 双眼连线倾斜角（总能算，反映头部歪斜）+ 眼睛平均 y（前倾时下移）
    float edx = kp[KEYPOINT_RIGHT_EYE].x - kp[KEYPOINT_LEFT_EYE].x;
    float edy = kp[KEYPOINT_RIGHT_EYE].y - kp[KEYPOINT_LEFT_EYE].y;
    *out_eye_tilt = normalize_tilt(atan2f(edy, edx) * 180.0f / (float)M_PI);
    float eye_y_avg = (kp[KEYPOINT_LEFT_EYE].y + kp[KEYPOINT_RIGHT_EYE].y) * 0.5f;

    // 双肩都可信：完整判断（眼肩垂直距离判前倾 + 双肩倾斜判歪斜）
    if (kp[KEYPOINT_LEFT_SHOULDER].valid && kp[KEYPOINT_RIGHT_SHOULDER].valid) {
        // 眼肩垂直距离：正常眼睛在肩膀上方(dy>0)，前倾低头时眼睛下移、dy 变小
        float shoulder_y_avg = (kp[KEYPOINT_LEFT_SHOULDER].y + kp[KEYPOINT_RIGHT_SHOULDER].y) * 0.5f;
        float eye_shoulder_dy = shoulder_y_avg - eye_y_avg;

        // ratio 仅供日志参考（人体比例，不再作前倾判断依据）
        float eye_dist = dist_norm(kp[KEYPOINT_LEFT_EYE].x, kp[KEYPOINT_LEFT_EYE].y,
                                   kp[KEYPOINT_RIGHT_EYE].x, kp[KEYPOINT_RIGHT_EYE].y);
        float shoulder_dist = dist_norm(kp[KEYPOINT_LEFT_SHOULDER].x, kp[KEYPOINT_LEFT_SHOULDER].y,
                                        kp[KEYPOINT_RIGHT_SHOULDER].x, kp[KEYPOINT_RIGHT_SHOULDER].y);
        *out_ratio = (shoulder_dist > 0.001f) ? (eye_dist / shoulder_dist) : 0.0f;

        float sdx = kp[KEYPOINT_RIGHT_SHOULDER].x - kp[KEYPOINT_LEFT_SHOULDER].x;
        float sdy = kp[KEYPOINT_RIGHT_SHOULDER].y - kp[KEYPOINT_LEFT_SHOULDER].y;
        *out_shoulder_tilt = normalize_tilt(atan2f(sdy, sdx) * 180.0f / (float)M_PI);

        ESP_LOGI(TAG, "[both shoulders] eye_shoulder_dy=%.3f ratio=%.3f shoulder_tilt=%.1f eye_tilt=%.1f",
                 eye_shoulder_dy, *out_ratio, *out_shoulder_tilt, *out_eye_tilt);

        if (eye_shoulder_dy < EYE_SHOULDER_DY_MIN) return POSTURE_BAD_NECK;              // 前倾
        if (fabsf(*out_shoulder_tilt) > SHOULDER_TILT_WARN) return POSTURE_BAD_SHOULDER;  // 歪斜
        return POSTURE_OK;
    }

    // 缺一肩（左肩量化失效）：退化用双眼判断
    ESP_LOGI(TAG, "[single shoulder] L_sh=%.2f R_sh=%.2f -> 用双眼 eye_tilt=%.1f eye_y=%.2f",
             kp[KEYPOINT_LEFT_SHOULDER].score, kp[KEYPOINT_RIGHT_SHOULDER].score,
             *out_eye_tilt, eye_y_avg);

    if (fabsf(*out_eye_tilt) > EYE_TILT_WARN) return POSTURE_BAD_SHOULDER;  // 头部歪斜
    if (eye_y_avg > EYE_Y_FORWARD_MAX) return POSTURE_BAD_NECK;             // 低头/前倾
    return POSTURE_OK;
}

esp_err_t posture_model_init(void)
{
    ESP_LOGI(TAG, "Initializing posture model (pose_model, 4 keypoints)...");

    // 创建模型：param_copy=true 把参数拷到 PSRAM（8MB PSRAM 充裕），
    // 否则 param_copy=false 每个卷积都要从 flash 读权重，推理会慢到 40s+ 触发 watchdog
    s_model = new dl::Model((const char*)pose_model_int8_final_espdl,
                            fbs::MODEL_LOCATION_IN_FLASH_RODATA,
                            0,                            // max_internal_size
                            dl::MEMORY_MANAGER_GREEDY,    // mm_type
                            nullptr,                      // key
                            true);                        // param_copy=true
    if (s_model == nullptr) {
        ESP_LOGE(TAG, "Failed to create model instance");
        return ESP_FAIL;
    }

    // ImagePreprocessor：ImageNet 归一化 + RGB（rgb_swap=false），不启用 letterbox
    s_preprocessor = new dl::image::ImagePreprocessor(s_model, IMAGENET_MEAN, IMAGENET_STD, false);

    // 打印模型输入信息
    std::map<std::string, dl::TensorBase *> model_inputs = s_model->get_inputs();
    dl::TensorBase *model_input = model_inputs.begin()->second;
    ESP_LOGI(TAG, "Model input: shape=[%d,%d,%d,%d], exponent=%d",
             model_input->shape[0], model_input->shape[1], model_input->shape[2], model_input->shape[3],
             (int)model_input->exponent);

    ESP_LOGI(TAG, "Free heap: %lu bytes, Free PSRAM: %lu bytes",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

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

    // 获取输出 heatmap (1,4,120,160)
    std::map<std::string, dl::TensorBase *> model_outputs = s_model->get_outputs();
    dl::TensorBase *model_output = model_outputs.begin()->second;

    std::vector<int> output_shape = model_output->get_shape();
    int8_t* quant_heatmaps = (int8_t*)model_output->data;
    int output_exp = model_output->exponent;

    ESP_LOGI(TAG, "Model output: shape=[%d,%d,%d,%d], exponent=%d",
             output_shape[0], output_shape[1], output_shape[2], output_shape[3], output_exp);

    // 解析热力图 → 4 关键点
    parse_heatmaps(quant_heatmaps, output_exp, output->keypoints);

    // 姿态判断
    output->result = judge_posture(output->keypoints, &output->ratio,
                                   &output->shoulder_tilt_deg, &output->eye_tilt_deg);

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
