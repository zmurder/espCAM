/**
 * ESP32-S3 坐姿检测示例主程序
 * ============================
 * 摄像头(OV3660, 320×240) → 模型推理 → heatmap 后处理 → 坐姿判断
 *
 * 模型: pose_model.espdl (~150KB, int8, ESP32-S3, 重构版 ~0.39G MACs)
 *   输入 (1,3,240,320) RGB + ImageNet 归一化
 *   输出 (1,6,120,160) heatmap，6 通道: 左眼/右眼/左耳/右耳/左肩/右肩
 *
 * 坐姿判断: 双肩连线倾斜角 >10° 判为歪斜; 低头看不到眼时用双耳定位头部
 */
#include "pose_inference.h"
#include "pose_postprocess.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_camera.h"  // esp32-camera 驱动
#include <cmath>
#include <vector>

static const char* TAG = "pose";

// ---- 摄像头引脚配置（按你的 ESP32-S3 开发板修改） ----
// 下面是常见 S3 + OV3660 配置示例，不同板子引脚不同，请查阅板子原理图
static camera_config_t camera_config = {
    .pin_pwdn  = -1,
    .pin_reset = -1,
    .pin_xclk  = 10,
    .pin_sccb_sda = 40,
    .pin_sccb_scl = 39,
    .pin_d7 = 15, .pin_d6 = 17, .pin_d5 = 18, .pin_d4 = 12,
    .pin_d3 = 14, .pin_d2 = 16, .pin_d1 = 11, .pin_d0 = 48,
    .pin_vsync = 38,
    .pin_href   = 47,
    .pin_pclk   = 13,
    .xclk_freq_hz = 16000000,
    .ledc_timer  = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_RGB888,  // RGB888，与训练 RGB 对齐（OV3660 支持）
    .frame_size   = FRAMESIZE_QVGA,    // 320×240，正好匹配模型输入（4:3）
    .jpeg_quality = 12,
    .fb_count     = 2,
    .fb_location  = CAMERA_FB_IN_PSRAM,
    .grab_mode    = CAMERA_GRAB_LATEST,
};

// 模型加载位置：SD 卡 / 嵌入固件(RODATA) 二选一
// #define LOAD_FROM_RODATA  // 嵌入固件（需把 .espdl 转成 C 数组，见 README）

extern "C" void app_main()
{
    // 1. 初始化摄像头
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "摄像头初始化失败: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "摄像头就绪 (320x240 RGB888)");

    // 2. 加载模型
    //    SD 卡方式：需先把 pose_model.espdl + pose_model.info 拷到 /sdcard/
#ifdef LOAD_FROM_RODATA
    extern const uint8_t pose_model_espdl[] asm("_binary_pose_model_espdl_start");
    pose::PoseInference pose_model((const char*)pose_model_espdl,
                                    fbs::MODEL_LOCATION_IN_FLASH_RODATA);
#else
    pose::PoseInference pose_model("/sdcard/pose_model.espdl",
                                    fbs::MODEL_LOCATION_IN_SDCARD);
#endif
    ESP_LOGI(TAG, "模型加载完成，开始坐姿检测");

    while (true) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "取帧失败");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // 3. 构造 ESP-DL img_t
        //    注意：img_t 字段名/格式枚举按你的 esp-dl 版本确认
        //    （参考 esp-dl/vision/image/dl_image_process.hpp 与 dl_image_define.hpp）
        dl::image::img_t img = {};
        img.data   = fb->buf;
        img.width  = fb->width;
        img.height = fb->height;
        img.format = dl::image::DL_IMAGE_FORMAT_RGB888;  // 与 pixel_format 一致

        // 4. center crop area：QVGA(320×240) 已是 4:3，全图即可
        //    若摄像头分辨率非 4:3，需 center crop 到 4:3（与训练 center_crop_resize 对齐）
        //    例：640×480 -> {0,0,640,480}；1920×1080 -> {240,0,1440,1080}
        std::vector<int> crop_area;  // 空 = 全图

        // 5. 推理
        dl::TensorBase* output = pose_model.infer(img, crop_area);

        // 6. 后处理：argmax + conf 阈值(0.6) + 坐姿指标
        int8_t* hm = output->get_element_ptr<int8_t>();
        pose::PoseResult r = pose::postprocess(hm, output->get_exponent());

        // 7. 坐姿判断
        if (!r.reliable) {
            ESP_LOGW(TAG, "姿态不可靠(仅%d点可信)，请调整后重测", r.n_valid);
        } else if (std::fabs(r.shoulder_tilt_deg) > pose::TILT_WARN_DEG) {
            ESP_LOGW(TAG, ">> 坐姿歪斜! 双肩倾斜 %.1f° (阈值%.0f°) 肩宽%.0fpx",
                     r.shoulder_tilt_deg, pose::TILT_WARN_DEG, r.shoulder_width_px);
        } else {
            const char* head_src = r.head_source == 0 ? "眼" :
                                   r.head_source == 1 ? "耳" : "无";
            ESP_LOGI(TAG, "坐姿正常 肩倾%.1f° 头倾%.1f°(来自%s) 肩宽%.0fpx 头距%.0fpx",
                     r.shoulder_tilt_deg, r.head_tilt_deg, head_src,
                     r.shoulder_width_px, r.head_dist_px);
        }

        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(200));  // ~5 fps
    }
}
