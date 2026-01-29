#include "esp_camera.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"  // 添加 FreeRTOS 核心头文件
#include "freertos/task.h"      // 再添加 FreeRTOS 任务相关头文件
#include "camera_app.h"         // 添加头文件引用

// 相机配置 - ESP32S3_EYE板型
#define CAM_PIN_PWDN PWDN_GPIO_NUM
#define CAM_PIN_RESET RESET_GPIO_NUM
#define CAM_PIN_XCLK XCLK_GPIO_NUM
#define CAM_PIN_SIOD SIOD_GPIO_NUM
#define CAM_PIN_SIOC SIOC_GPIO_NUM

#define CAM_PIN_D7 Y9_GPIO_NUM
#define CAM_PIN_D6 Y8_GPIO_NUM
#define CAM_PIN_D5 Y7_GPIO_NUM
#define CAM_PIN_D4 Y6_GPIO_NUM
#define CAM_PIN_D3 Y5_GPIO_NUM
#define CAM_PIN_D2 Y4_GPIO_NUM
#define CAM_PIN_D1 Y3_GPIO_NUM
#define CAM_PIN_D0 Y2_GPIO_NUM
#define CAM_PIN_VSYNC VSYNC_GPIO_NUM
#define CAM_PIN_HREF HREF_GPIO_NUM
#define CAM_PIN_PCLK PCLK_GPIO_NUM

static const char* TAG = "camera";  // 添加TAG用于日志输出

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    .xclk_freq_hz = 5000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,  // YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,    // FRAMESIZE_VGA,     // FRAMESIZE_QVGA,    // QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the
                                     //   ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    .jpeg_quality = 12,  // 0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 1,       // When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    // .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,  // CAMERA_GRAB_WHEN_EMPTY,  // CAMERA_GRAB_LATEST. Sets when buffers should be filled
};

// 临时的process_image函数实现，后续可根据需要替换
void process_image(int width, int height, pixformat_t format, uint8_t* buf, size_t len)
{
    ESP_LOGI(TAG, "Processing image: %dx%d, format: %d, size: %zu bytes", width, height, format, len);
    // 在这里添加图像处理逻辑
    // 例如: 将图像数据发送到Web服务器、存储到SD卡或其他处理
}

esp_err_t camera_init()
{
    // power up the camera if PWDN pin is defined
    if (CAM_PIN_PWDN != -1) {
        gpio_config_t conf;
        conf.intr_type = GPIO_INTR_DISABLE;
        conf.mode = GPIO_MODE_OUTPUT;
        conf.pin_bit_mask = 1ULL << CAM_PIN_PWDN;
        conf.pull_down_en = 0;
        conf.pull_up_en = 0;
        ESP_ERROR_CHECK(gpio_config(&conf));
        gpio_set_level(CAM_PIN_PWDN, 0);
        vTaskDelay(5 / portTICK_PERIOD_MS);  // 延迟等待电源稳定
    }

    // initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }
    else {
        ESP_LOGI(TAG, "Camera Init Success");
    }

    return ESP_OK;
}

esp_err_t camera_capture()
{
    // acquire a frame
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera Capture Failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Camera Capture Success. Frame size: %zu bytes", fb->len);

    // replace this with your own function
    process_image(fb->width, fb->height, fb->format, fb->buf, fb->len);

    // return the frame buffer back to the driver for reuse
    esp_camera_fb_return(fb);
    return ESP_OK;
}