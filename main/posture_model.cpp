/**
 * posture_model.cpp
 * 坐姿检测模型 - 正确前处理（与 Python 测试脚本一致）
 *
 * 前处理必须与训练时一致：
 * 1. Letterbox resize 到 320x240
 * 2. ImageNet 归一化: (pixel/255 - mean) / std
 *    mean = [0.485, 0.456, 0.406], std = [0.229, 0.224, 0.225]
 * 3. 量化到 INT8 (exponent=-6)
 */

#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "dl_image_jpeg.hpp"
#include "dl_image_preprocessor.hpp"
#include "dl_model_base.hpp"
#include "posture_model.h"

static const char* TAG = "POSTURE_MODEL";

extern const uint8_t litepose_esp32s3_espdl[] asm("_binary_litepose_esp32s3_espdl_start");
extern const uint8_t _binary_320240_jpg_start[] asm("_binary_320240_jpg_start");
extern const uint8_t _binary_320240_jpg_end[] asm("_binary_320240_jpg_end");

static dl::Model* s_model = NULL;
static dl::image::ImagePreprocessor* s_preprocessor = NULL;
static uint32_t s_last_latency_us = 0;

#define MODEL_INPUT_H 240
#define MODEL_INPUT_W 320
#define MODEL_INPUT_C 3

#define HEATMAP_H 60
#define HEATMAP_W 80
#define KEYPOINT_COUNT 7

// 模型量化 exponent（从 .info 文件获取）
#define INPUT_EXPONENT  -6
#define OUTPUT_EXPONENT -7

#define POSTURE_RATIO_MIN 0.3f
#define POSTURE_RATIO_MAX 0.8f
#define HEATMAP_THRESHOLD 0.05f

// Letterbox padding 值
#define LETTERBOX_PAD 114

/**
 * @brief 反量化 INT8 → float
 */
static inline float dequantize(int8_t q, int exponent)
{
    return dl::dequantize(q, DL_SCALE(exponent));
}

/**
 * @brief 前处理: JPEG → Letterbox + 简单归一化(/255) + 量化 → NHWC
 *
 * 与官方 yolo11_pose 例子一致:
 * ImagePreprocessor with mean={0,0,0}, std={255,255,255}
 * 即简单 pixel / 255.0f
 */
static void preprocess(const uint8_t* rgb, int src_w, int src_h, int8_t* output_buf,
                      int* out_pad_left, int* out_pad_top, float* out_scale)
{
    // 计算 letterbox 缩放比例
    float scale_x = (float)src_w / MODEL_INPUT_W;
    float scale_y = (float)src_h / MODEL_INPUT_H;
    float scale = (scale_x < scale_y) ? scale_x : scale_y;

    int scaled_w = (int)(src_w / scale);
    int scaled_h = (int)(src_h / scale);
    int pad_left = (MODEL_INPUT_W - scaled_w) / 2;
    int pad_top = (MODEL_INPUT_H - scaled_h) / 2;

    *out_pad_left = pad_left;
    *out_pad_top = pad_top;
    *out_scale = scale;

    ESP_LOGI(TAG, "Preprocess: src=%dx%d, scaled=%dx%d, pad=(%d,%d), scale=%.4f",
             src_w, src_h, scaled_w, scaled_h, pad_left, pad_top, scale);

    // 清空输出 buffer（填充 letterbox 背景色 114）
    for (int h = 0; h < MODEL_INPUT_H; h++) {
        for (int w = 0; w < MODEL_INPUT_W; w++) {
            for (int c = 0; c < MODEL_INPUT_C; c++) {
                // 简单归一化: (114 - 0) / 255 = 0.447
                float normalized = 114.0f / 255.0f;
                int idx = h * MODEL_INPUT_W * MODEL_INPUT_C + w * MODEL_INPUT_C + c;
                output_buf[idx] = dl::quantize<int8_t>(normalized, DL_RESCALE(INPUT_EXPONENT));
            }
        }
    }

    // 处理实际图像区域（最近邻缩放 + 简单归一化 /255）
    for (int y = 0; y < scaled_h; y++) {
        for (int x = 0; x < scaled_w; x++) {
            int dst_y = pad_top + y;
            int dst_x = pad_left + x;
            if (dst_y >= MODEL_INPUT_H || dst_x >= MODEL_INPUT_W) continue;

            int src_x = (int)(x * scale);
            int src_y = (int)(y * scale);
            if (src_x >= src_w) src_x = src_w - 1;
            if (src_y >= src_h) src_y = src_h - 1;

            int src_idx = src_y * src_w * 3 + src_x * 3;

            for (int c = 0; c < MODEL_INPUT_C; c++) {
                // 简单归一化: pixel / 255.0f (与官方 yolo11_pose 一致)
                float pixel = (float)rgb[src_idx + c];
                float normalized = pixel / 255.0f;
                int idx = dst_y * MODEL_INPUT_W * MODEL_INPUT_C + dst_x * MODEL_INPUT_C + c;
                output_buf[idx] = dl::quantize<int8_t>(normalized, DL_RESCALE(INPUT_EXPONENT));
            }
        }
    }
}

/**
 * @brief 从热力图解析关键点（NHWC 布局）
 */
static void parse_heatmaps(const int8_t* quant_heatmaps, int exponent,
                         posture_keypoint_data_t* keypoints,
                         int pad_left, int pad_top, float scale,
                         int orig_w, int orig_h)
{
    for (int k = 0; k < KEYPOINT_COUNT; k++) {
        float max_val = -1e6f;
        int max_x = 0, max_y = 0;

        // NHWC: index = (y * W + x) * C + k
        for (int y = 0; y < HEATMAP_H; y++) {
            for (int x = 0; x < HEATMAP_W; x++) {
                int idx = (y * HEATMAP_W + x) * KEYPOINT_COUNT + k;
                float val = dequantize(quant_heatmaps[idx], exponent);
                if (val > max_val) {
                    max_val = val;
                    max_x = x;
                    max_y = y;
                }
            }
        }

        keypoints[k].score = max_val;

        // 坐标映射: 热力图 → letterbox → 原图 → 归一化
        // 热力图坐标 (max_x, max_y) 对应 letterbox 坐标系
        float letterbox_x = (float)max_x * MODEL_INPUT_W / HEATMAP_W;
        float letterbox_y = (float)max_y * MODEL_INPUT_H / HEATMAP_H;

        float img_x = (letterbox_x - pad_left) / scale;
        float img_y = (letterbox_y - pad_top) / scale;

        keypoints[k].x = img_x / orig_w;
        keypoints[k].y = img_y / orig_h;

        ESP_LOGI(TAG, "  kp[%d]: hm=(%d,%d) lb=(%.1f,%.1f) img=(%.1f,%.1f) norm=(%.3f,%.3f) score=%.4f",
                 k, max_x, max_y, letterbox_x, letterbox_y, img_x, img_y,
                 keypoints[k].x, keypoints[k].y, max_val);
    }
}

static float calc_distance(posture_keypoint_data_t* p1, posture_keypoint_data_t* p2)
{
    float dx = p1->x - p2->x;
    float dy = p1->y - p2->y;
    return sqrtf(dx * dx + dy * dy);
}

static posture_result_t judge_posture(posture_keypoint_data_t* keypoints, float* out_ratio)
{
    for (int i = 0; i < KEYPOINT_COUNT; i++) {
        if (keypoints[i].score < HEATMAP_THRESHOLD) {
            return POSTURE_NOT_DETECTED;
        }
    }

    float eye_dist = calc_distance(&keypoints[KEYPOINT_LEFT_EYE], &keypoints[KEYPOINT_RIGHT_EYE]);
    float shoulder_dist = calc_distance(&keypoints[KEYPOINT_LEFT_SHOULDER], &keypoints[KEYPOINT_RIGHT_SHOULDER]);

    if (shoulder_dist < 0.001f) {
        return POSTURE_NOT_DETECTED;
    }

    float ratio = eye_dist / shoulder_dist;
    *out_ratio = ratio;

    ESP_LOGI(TAG, "eye_dist=%.3f, shoulder_dist=%.3f, ratio=%.3f", eye_dist, shoulder_dist, ratio);

    if (ratio < POSTURE_RATIO_MIN || ratio > POSTURE_RATIO_MAX) {
        return POSTURE_BAD_NECK;
    }

    // 肩膀不平判定：只有当两个肩膀置信度都足够高时才判定
    // 避免因 INT8 量化导致低置信度检测错误而误判
    float left_shoulder_score = keypoints[KEYPOINT_LEFT_SHOULDER].score;
    float right_shoulder_score = keypoints[KEYPOINT_RIGHT_SHOULDER].score;
    float shoulder_score_threshold = 0.3f;  // 低于此置信度不判定肩膀问题

    if (left_shoulder_score > shoulder_score_threshold && right_shoulder_score > shoulder_score_threshold) {
        float shoulder_y_diff = fabsf(keypoints[KEYPOINT_LEFT_SHOULDER].y - keypoints[KEYPOINT_RIGHT_SHOULDER].y);
        if (shoulder_y_diff > 0.1f) {
            return POSTURE_BAD_SHOULDER;
        }
    }

    return POSTURE_OK;
}

esp_err_t posture_model_init(void)
{
    ESP_LOGI(TAG, "Initializing posture model...");

    s_model = new dl::Model((const char*)litepose_esp32s3_espdl, fbs::MODEL_LOCATION_IN_FLASH_RODATA);

    if (s_model == nullptr) {
        ESP_LOGE(TAG, "Failed to create model instance");
        return ESP_FAIL;
    }

    // 打印模型输入信息
    std::map<std::string, dl::TensorBase *> model_inputs = s_model->get_inputs();
    dl::TensorBase *model_input = model_inputs.begin()->second;
    ESP_LOGI(TAG, "Model input: shape=[%d,%d,%d,%d], exponent=%d, dtype=%d",
             model_input->shape[0], model_input->shape[1], model_input->shape[2], model_input->shape[3],
             (int)model_input->exponent, (int)model_input->dtype);

    ESP_LOGI(TAG, "Model initialized successfully");

#if POSTURE_TEST_MODE
    esp_err_t ret = s_model->test();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Model test FAILED!");
    } else {
        ESP_LOGI(TAG, "Model test PASSED!");
    }
#endif

    return ESP_OK;
}

esp_err_t posture_model_run_inference(camera_fb_t* fb, posture_output_t* output)
{
    if (s_model == NULL || output == NULL) {
        return ESP_FAIL;
    }

    memset(output, 0, sizeof(posture_output_t));

    uint64_t start_time = esp_timer_get_time();

    // 图像数据指针和大小
    void* img_data = NULL;
    size_t img_len = 0;

#if POSTURE_TEST_IMAGE_MODE
    // 静态图像测试模式：使用嵌入的测试图像
    img_data = (void*)_binary_320240_jpg_start;
    img_len = (size_t)(_binary_320240_jpg_end - _binary_320240_jpg_start);
    ESP_LOGI(TAG, "Using TEST IMAGE, size: %d bytes", img_len);
#else
    // 相机模式
    if (fb == NULL) {
        return ESP_FAIL;
    }
    img_data = (void*)fb->buf;
    img_len = fb->len;
#endif

    // 1. JPEG 解码
    dl::image::jpeg_img_t jpeg_img;
    jpeg_img.data = img_data;
    jpeg_img.data_len = img_len;

    auto img = dl::image::sw_decode_jpeg(jpeg_img, dl::image::DL_IMAGE_PIX_TYPE_RGB888);
    if (img.data == nullptr) {
        ESP_LOGE(TAG, "JPEG decode failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Image decoded: %dx%d", img.width, img.height);

    // 2. 前处理: Letterbox + ImageNet归一化 + 量化
    // 注意：输入缓冲区必须分配在 PSRAM，避免占用紧张的 DRAM
    int pad_left, pad_top;
    float scale;
    int8_t* input_buf = (int8_t*)heap_caps_malloc(1 * MODEL_INPUT_H * MODEL_INPUT_W * MODEL_INPUT_C, MALLOC_CAP_SPIRAM);
    if (input_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate input buffer in PSRAM");
        heap_caps_free(img.data);
        return ESP_FAIL;
    }
    preprocess((uint8_t*)img.data, img.width, img.height, input_buf, &pad_left, &pad_top, &scale);

    // 3. 获取模型输入 tensor 并填充数据
    std::map<std::string, dl::TensorBase *> model_inputs = s_model->get_inputs();
    dl::TensorBase *model_input = model_inputs.begin()->second;

    ESP_LOGI(TAG, "Model input buffer: shape=[%d,%d,%d,%d], exponent=%d",
             model_input->shape[0], model_input->shape[1], model_input->shape[2], model_input->shape[3],
             (int)model_input->exponent);

    // 复制数据到模型输入 memory
    memcpy(model_input->data, input_buf, 1 * MODEL_INPUT_H * MODEL_INPUT_W * MODEL_INPUT_C);

    // 4. 运行推理
    s_model->run();

    // 5. 获取输出
    std::map<std::string, dl::TensorBase *> model_outputs = s_model->get_outputs();
    dl::TensorBase *model_output = model_outputs.begin()->second;

    std::vector<int> output_shape = model_output->get_shape();
    int8_t* quant_heatmaps = (int8_t*)model_output->data;
    int output_exp = model_output->exponent;

    ESP_LOGI(TAG, "Model output: shape=[%d,%d,%d,%d], exponent=%d",
             output_shape[0], output_shape[1], output_shape[2], output_shape[3], output_exp);

    // 打印每个关键点热力图最大值
    ESP_LOGI(TAG, "Keypoint max heatmap values:");
    for (int k = 0; k < KEYPOINT_COUNT; k++) {
        float max_val = -1e6f;
        for (int y = 0; y < HEATMAP_H; y++) {
            for (int x = 0; x < HEATMAP_W; x++) {
                int idx = (y * HEATMAP_W + x) * KEYPOINT_COUNT + k;
                float val = dequantize(quant_heatmaps[idx], output_exp);
                if (val > max_val) max_val = val;
            }
        }
        ESP_LOGI(TAG, "  keypoint[%d] max=%.4f", k, max_val);
    }

    // 6. 解析热力图
    parse_heatmaps(quant_heatmaps, output_exp, output->keypoints,
                   pad_left, pad_top, scale, img.width, img.height);

    // 7. 姿态判断
    output->result = judge_posture(output->keypoints, &output->ratio);

    uint64_t end_time = esp_timer_get_time();
    s_last_latency_us = (uint32_t)(end_time - start_time);

    ESP_LOGI(TAG, "Inference done in %lu us, result=%d", (unsigned long)s_last_latency_us, output->result);

    heap_caps_free(img.data);
    heap_caps_free(input_buf);

    return ESP_OK;
}

void posture_model_deinit(void)
{
    if (s_model != nullptr) {
        delete s_model;
        s_model = nullptr;
    }
    if (s_preprocessor != nullptr) {
        delete s_preprocessor;
        s_preprocessor = nullptr;
    }
    ESP_LOGI(TAG, "Model deinitialized");
}

uint32_t posture_model_get_last_latency_us(void)
{
    return s_last_latency_us;
}