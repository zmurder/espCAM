/**
 * posture_model.cpp
 * 坐姿检测模型 - 参考官方 yolo11_pose 示例
 */

#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "dl_image_jpeg.hpp"
#include "dl_model_base.hpp"
#include "posture_model.h"

static const char* TAG = "POSTURE_MODEL";

// 模型符号声明（由链接器从嵌入的 .espdl 文件提供）
extern const uint8_t litepose_esp32s3_test_espdl[] asm("_binary_litepose_esp32s3_test_espdl_start");

// 全局模型指针
static dl::Model* s_model = NULL;
static uint32_t s_last_latency_us = 0;

// 模型输入尺寸
#define MODEL_INPUT_W 320
#define MODEL_INPUT_H 240
#define MODEL_INPUT_C 3

// 模型输出热力图尺寸
#define HEATMAP_W 80
#define HEATMAP_H 60

// 姿态阈值（眼睛距离/肩膀距离 比值）
#define POSTURE_RATIO_MIN 0.3f
#define POSTURE_RATIO_MAX 0.8f

// 热力图查找阈值
#define HEATMAP_THRESHOLD 0.2f

/**
 * @brief 从热力图提取关键点坐标
 */
static void parse_heatmaps(const float* heatmaps, posture_keypoint_data_t* keypoints)
{
    for (int k = 0; k < KEYPOINT_COUNT; k++) {
        float max_val = 0.0f;
        int max_x = 0, max_y = 0;

        for (int y = 0; y < HEATMAP_H; y++) {
            for (int x = 0; x < HEATMAP_W; x++) {
                float val = heatmaps[k * HEATMAP_H * HEATMAP_W + y * HEATMAP_W + x];
                if (val > max_val) {
                    max_val = val;
                    max_x = x;
                    max_y = y;
                }
            }
        }

        keypoints[k].x = (float)max_x / HEATMAP_W;
        keypoints[k].y = (float)max_y / HEATMAP_H;
        keypoints[k].score = max_val;
    }
}

/**
 * @brief 计算两点之间的欧氏距离
 */
static float calc_distance(posture_keypoint_data_t* p1, posture_keypoint_data_t* p2)
{
    float dx = p1->x - p2->x;
    float dy = p1->y - p2->y;
    return sqrtf(dx * dx + dy * dy);
}

/**
 * @brief 判断姿态是否正确
 */
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

    float shoulder_y_diff = fabsf(keypoints[KEYPOINT_LEFT_SHOULDER].y - keypoints[KEYPOINT_RIGHT_SHOULDER].y);
    if (shoulder_y_diff > 0.1f) {
        return POSTURE_BAD_SHOULDER;
    }

    return POSTURE_OK;
}

esp_err_t posture_model_init(void)
{
    ESP_LOGI(TAG, "Initializing posture model...");

    s_model = new dl::Model((const char*)litepose_esp32s3_test_espdl, fbs::MODEL_LOCATION_IN_FLASH_RODATA);

    if (s_model == nullptr) {
        ESP_LOGE(TAG, "Failed to create model instance");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Model initialized successfully");

#if POSTURE_TEST_MODE
    // 运行 test() 验证模型正确性
    esp_err_t ret = s_model->test();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Model test FAILED!");
    }
    else {
        ESP_LOGI(TAG, "Model test PASSED!");
    }
#endif

    return ESP_OK;
}

esp_err_t posture_model_run_inference(camera_fb_t* fb, posture_output_t* output)
{
    if (s_model == NULL || fb == NULL || output == NULL) {
        return ESP_FAIL;
    }

    memset(output, 0, sizeof(posture_output_t));

    uint64_t start_time = esp_timer_get_time();

    // 1. JPEG 解码为 RGB
    dl::image::jpeg_img_t jpeg_img;
    jpeg_img.data = (void*)fb->buf;
    jpeg_img.data_len = fb->len;

    auto img = dl::image::sw_decode_jpeg(jpeg_img, dl::image::DL_IMAGE_PIX_TYPE_RGB888);
    if (img.data == nullptr) {
        ESP_LOGE(TAG, "JPEG decode failed");
        return ESP_FAIL;
    }

    // 2. HWC -> CHW 转换 + 填入 TensorBase
    // 模型输入: [1, 3, 240, 320] CHW 格式
    static float s_chw_buf[MODEL_INPUT_C * MODEL_INPUT_H * MODEL_INPUT_W];

    uint8_t* rgb = (uint8_t*)img.data;
    int src_h = img.height;
    int src_w = img.width;

    for (int h = 0; h < src_h; h++) {
        for (int w = 0; w < src_w; w++) {
            for (int c = 0; c < MODEL_INPUT_C; c++) {
                // img 是 HWC (H, W, C)，转换为 CHW
                int chw_idx = c * src_h * src_w + h * src_w + w;
                int hwc_idx = h * src_w * MODEL_INPUT_C + w * MODEL_INPUT_C + c;
                s_chw_buf[chw_idx] = (float)rgb[hwc_idx];
            }
        }
    }

    std::vector<int> input_shape = {1, MODEL_INPUT_C, MODEL_INPUT_H, MODEL_INPUT_W};
    // exponent=0, dtype=UINT8, deep=false
    dl::TensorBase* input_tensor = new dl::TensorBase(input_shape, s_chw_buf, 0, dl::DATA_TYPE_UINT8, false);

    // 3. 运行推理
    s_model->run(input_tensor);

    // 4. 获取输出
    dl::TensorBase* output_tensor = s_model->get_output();
    if (output_tensor == nullptr) {
        ESP_LOGE(TAG, "No output tensor");
        delete input_tensor;
        heap_caps_free(img.data);
        return ESP_FAIL;
    }

    float* heatmaps = (float*)output_tensor->get_element_ptr();

    // 5. 解析热力图获取关键点
    parse_heatmaps(heatmaps, output->keypoints);

    // 6. 姿态判断
    output->result = judge_posture(output->keypoints, &output->ratio);

    uint64_t end_time = esp_timer_get_time();
    s_last_latency_us = (uint32_t)(end_time - start_time);

    ESP_LOGI(TAG, "Inference done in %lu us, result=%d", (unsigned long)s_last_latency_us, output->result);

    delete input_tensor;
    heap_caps_free(img.data);

    return ESP_OK;
}

void posture_model_deinit(void)
{
    if (s_model != nullptr) {
        delete s_model;
        s_model = nullptr;
        ESP_LOGI(TAG, "Model deinitialized");
    }
}

uint32_t posture_model_get_last_latency_us(void)
{
    return s_last_latency_us;
}
