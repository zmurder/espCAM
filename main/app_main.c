/*
 * ESP32CAM WiFi Camera Application
 *
 * Main application entry point for WiFi camera with SoftAP/STA mode
 * and configuration portal support.
 */

#include <string.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_net_stack.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_intr_types.h"

#if IP_NAPT
#include "lwip/lwip_napt.h"
#endif

#include "esp_camera.h"
#include "camera_app.h"
#include "wifi_manager.h"
#include "wifi_config_manager.h"
#include "led.h"
#include "udp_camera_client.h"
#include "posture_model.h"
#include "audio_player.h"
#include "time_sync.h"
#include "posture_sched.h"

static const char* TAG = "APP_MAIN";

#define WIFI_CONFIG_BUTTON_GPIO 14

// UDP 输出开关：每推理一帧，发一帧图像(20000) + 一帧检测结果(20002)
#define SEND_IMAGE_VIA_UDP 1   // 1=发送推理帧 JPEG 到 20000
#define SEND_RESULT_VIA_UDP 1  // 1=发送检测结果(关键点)到 20002

// 坐姿不良语音提示确认次数：连续 N 帧不良才播提示音（中间出现任何非不良帧即重新计数；
// 触发一次后持续不良期间不重播，恢复正常坐姿后才会再次触发）。
// 仅影响音频提示，UDP 发送的逐帧 result 不受影响
#define POSTURE_ALERT_CONSECUTIVE 2

static void audio_player_init_task(void* arg)
{
    ESP_LOGI(TAG, "Audio player init task running on CPU core %d", xPortGetCoreID());
    esp_err_t ret = audio_player_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Audio player initialization failed: %s", esp_err_to_name(ret));
    }
    else {
        ESP_LOGI(TAG, "Audio player initialized successfully on CPU1");
    }
    vTaskDelete(NULL);
}

/**
 * @brief 坐姿检测推理任务
 */
static void posture_inference_task(void* arg)
{
    ESP_LOGI(TAG, "Posture inference task started on core %d", xPortGetCoreID());

    // 等待系统稳定
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 初始化姿态检测模型
    ESP_LOGI(TAG, "Posture model init...");
    esp_err_t ret = posture_model_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Posture model init failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Posture model init OK");

    ESP_LOGI(TAG, "Starting posture inference loop...");

#if POSTURE_TEST_MODE
    // 测试模式：test() 已在 posture_model_init() 中运行，这里直接结束任务
    ESP_LOGI(TAG, "POSTURE_TEST_MODE: Model test completed, exiting task");
    posture_model_deinit();
    vTaskDelete(NULL);
#else
    // 实际推理模式：一次捕获，同时用于推理和UDP发送
    while (1) {
        // 检测时间段调度：窗口外不取帧不推理不发送（时间未同步/未设置时段时始终检测）
        if (!posture_sched_is_active()) {
            vTaskDelay(pdMS_TO_TICKS(5000));  // 低频轮询等待进窗（分钟级粒度足够）
            continue;
        }

        camera_fb_t* fb = esp_camera_fb_get();
        if (fb == NULL) {
            ESP_LOGW(TAG, "Failed to get camera frame");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // 姿态推理
        posture_output_t output;
        ret = posture_model_run_inference(fb, &output);

        if (ret == ESP_OK) {
            // 连续不良确认：POSTURE_ALERT_CONSECUTIVE 宏控制连续多少帧不良才播提示音
            static uint32_t s_bad_streak = 0;    // 连续不良帧计数（非不良帧清零）
            static bool s_alert_played = false;  // 本轮连续不良是否已播过提示（恢复后复位，防止重播轰炸）

            if (output.result == POSTURE_BAD_NECK || output.result == POSTURE_BAD_SHOULDER) {
                const char* bad_name = (output.result == POSTURE_BAD_NECK) ? "BAD_NECK" : "BAD_SHOULDER";
                s_bad_streak++;
                if (!s_alert_played && s_bad_streak >= POSTURE_ALERT_CONSECUTIVE) {
                    ESP_LOGE(TAG, "Posture: %s (ratio=%.3f) streak=%lu/%d -> play alert", bad_name, output.ratio, (unsigned long)s_bad_streak, POSTURE_ALERT_CONSECUTIVE);
                    audio_player_play_posture_alert(output.result);
                    s_alert_played = true;
                }
                else {
                    ESP_LOGW(TAG,
                             "Posture: %s (ratio=%.3f) streak=%lu/%d%s",
                             bad_name,
                             output.ratio,
                             (unsigned long)s_bad_streak,
                             POSTURE_ALERT_CONSECUTIVE,
                             s_alert_played ? " (alerted)" : " (no alert yet)");
                }
            }
            else {
                s_bad_streak = 0;  // 连续被打断（OK/漏检/不可信），重新计数
                s_alert_played = false;
                if (output.result == POSTURE_OK) {
                    ESP_LOGI(TAG, "Posture: OK (ratio=%.3f)", output.ratio);
                }
                else if (output.result == POSTURE_NOT_DETECTED) {
                    ESP_LOGW(TAG, "Posture: Keypoints not detected");
                }
                else if (output.result == POSTURE_UNRELIABLE) {
                    ESP_LOGW(TAG, "Posture: unreliable keypoints (implausible geometry), skip");
                }
                else if (output.result == POSTURE_TOO_FAR) {
                    ESP_LOGW(TAG, "Posture: subject too far, skip");
                }
            }

// UDP 输出：先结果(20002)后图像(20000)，PC 收到图像时结果已到，可直接叠加
#if SEND_RESULT_VIA_UDP
            send_posture_result_via_udp(&output);
#endif
#if SEND_IMAGE_VIA_UDP
            send_image_via_udp(fb);  // 推理一张发一张（fb 须在下方 return 前使用）
#endif

            ESP_LOGI(TAG, "Inference latency: %lu us", (unsigned long)posture_model_get_last_latency_us());
        }

        // 释放帧缓冲：放在 UDP 发送之后，确保 send_image_via_udp 用的 fb 数据有效
        esp_camera_fb_return(fb);

        // 推理间隔：给 wifi/其他任务喘息
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    posture_model_deinit();
    vTaskDelete(NULL);
#endif
}

void app_main(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // SNTP 对时：STA 拿到 IP 后自动同步；配合 SYSTEM 日志时间戳源，此后日志带真实日期时间
    time_sync_init();

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 检测时间段调度：从 NVS 加载（断电保持；须在 nvs_flash_init 之后）；
    // 时间未同步或未设置时段时全天检测
    posture_sched_init();

    /* Initialize status LED on GPIO2, active High */
    led_init(2, false);

    /* Initialize camera first (to allocate high-priority interrupts) */
    esp_err_t camera_err = camera_init();
    if (camera_err != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed: %s", esp_err_to_name(camera_err));
        led_set_state(LED_STATE_BLINK_FAST);
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Initialize audio player on CPU1 to use CPU1's interrupt resources */
    xTaskCreatePinnedToCore(audio_player_init_task, "audio_init", 4096, NULL, 5, NULL, 1);
    ESP_LOGI(TAG, "Audio player initialization DISABLED for debugging");

    /* Initialize WiFi manager */
    ESP_ERROR_CHECK(wifi_manager_init());

    /* Register WiFi event handlers */
    wifi_register_event_handlers(NULL);

    /* Initialize AP */
    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    esp_netif_t* esp_netif_ap = wifi_init_softap();

    /* Initialize STA */
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    esp_netif_t* esp_netif_sta = wifi_init_sta();

    /* Start WiFi */
    ESP_ERROR_CHECK(esp_wifi_start());

    // 初始化WiFi配置管理器
    ESP_ERROR_CHECK(wifi_config_manager_init(WIFI_CONFIG_BUTTON_GPIO, wifi_get_event_group(), esp_netif_ap));

    /*
     * If a compile-time STA SSID is configured, wait for connection result
     */
    if (strlen(WIFI_STA_SSID) > 0) {
        EventBits_t bits = xEventGroupWaitBits(wifi_get_event_group(), WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", WIFI_STA_SSID, WIFI_STA_PASSWD);
            wifi_set_dns_addr(esp_netif_ap, esp_netif_sta);
        }
        else if (bits & WIFI_FAIL_BIT) {
            ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", WIFI_STA_SSID, WIFI_STA_PASSWD);
            led_set_state(LED_STATE_BLINK_FAST);
        }
        else {
            ESP_LOGE(TAG, "UNEXPECTED EVENT");
            return;
        }
    }
    else {
        ESP_LOGI(TAG, "No compile-time STA SSID configured; skipping auto-connect wait.");
        led_set_state(LED_STATE_BLINK_FAST);
    }

    /* Set sta as the default interface */
    esp_netif_set_default_netif(esp_netif_sta);

    /* Enable napt on the AP netif */
    if (esp_netif_napt_enable(esp_netif_ap) != ESP_OK) {
        ESP_LOGE(TAG, "NAPT not enabled on the netif: %p", esp_netif_ap);
    }

    // 等待音频初始化完成
    vTaskDelay(pdMS_TO_TICKS(100));

    // 打印最终中断分配情况
    esp_intr_dump(NULL);

    // 启动 UDP 自动发现监听（PC 周期广播 → 自动学习/更新目标 IP，免改 UDP_SERVER_IP）
    udp_discovery_start();

    // 启动坐姿检测推理任务（core 1，避开 core 0 的 wifi 任务争用 CPU）
    xTaskCreatePinnedToCore(posture_inference_task, "posture_inf", 32768, NULL, 5, NULL, 1);

    // 禁用独立 start_udp_camera()：图像改为随推理发送（posture_inference_task 内推理一张发一张）
    // start_udp_camera();
}
