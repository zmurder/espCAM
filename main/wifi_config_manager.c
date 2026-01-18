/*
 * WiFi Configuration Manager Implementation
 *
 * This module handles dynamic WiFi configuration via a captive portal.
 */

#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "cJSON.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/apps/netbiosns.h"
#include "mdns.h"

#include "esp_err.h"
#include "wifi_config_manager.h"
#include "wifi_manager.h"
#include "esp_mac.h"
#include "led.h"
#include "udp_camera_client.h"
#include "esp_task_wdt.h"

static const char* TAG = "wifi_config";

#define WIFI_CONFIG_BUTTON_GPIO 12               // 默认使用GPIO0，即开发板上的BOOT按钮
#define WIFI_PROV_HOLD_TIME pdMS_TO_TICKS(5000)  // 长按5秒进入配置模式
#define WIFI_CONFIG_NAMESPACE "wifi_config"
#define WIFI_CONFIG_SSID_KEY "ssid"
#define WIFI_CONFIG_PASS_KEY "password"

#define PROV_AP_CHANNEL 1
#define PROV_AP_SSID_PREFIX "esp32cam_config"

static int s_button_gpio = WIFI_CONFIG_BUTTON_GPIO;
static bool s_provisioning_mode = false;
static TaskHandle_t s_prov_task_handle = NULL;
static httpd_handle_t s_server = NULL;
static esp_netif_t* s_ap_netif = NULL;
static esp_netif_ip_info_t s_ap_ip_info;
static bool s_isr_service_installed = false;  // 标记ISR服务是否已安装

/* WiFi扫描相关变量 */
#define MAX_SCAN_RESULTS 20


static void wifi_config_check_button_task(void* arg);
static void start_provisioning_mode(void);
static void stop_provisioning_mode(void);
static esp_err_t start_webserver(void);
static esp_err_t stop_webserver(void);
static esp_err_t wifi_connect_to_ap(const char* ssid, const char* password);
static void wifi_prov_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static esp_err_t scan_handler(httpd_req_t* req);

// GPIO中断处理函数
static void IRAM_ATTR button_isr_handler(void* arg)
{
    // 检查任务句柄是否有效，避免空指针访问
    if (s_prov_task_handle != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(s_prov_task_handle, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

// 配置模式WiFi事件处理器
static void wifi_prov_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*)event_data;
        ESP_LOGI(TAG, "Station %02x:%02x:%02x:%02x:%02x:%02x joined, AID=%d", event->mac[0], event->mac[1], event->mac[2], event->mac[3], event->mac[4], event->mac[5], event->aid);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*)event_data;
        ESP_LOGI(
            TAG, "Station %02x:%02x:%02x:%02x:%02x:%02x left, AID=%d, reason:%d", event->mac[0], event->mac[1], event->mac[2], event->mac[3], event->mac[4], event->mac[5], event->aid, event->reason);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_AP_STAIPASSIGNED) {
        ip_event_ap_staipassigned_t* event = (ip_event_ap_staipassigned_t*)event_data;
        ESP_LOGI(TAG, "Station assigned IP: " IPSTR, IP2STR(&event->ip));
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Station started");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_LOST_IP) {
        ESP_LOGI(TAG, "Lost IP");
    }
}

// HTTP请求处理函数
static esp_err_t index_handler(httpd_req_t* req)
{
    static const char html_response[] =
        "<!DOCTYPE html>"
        "<html>"
        "<head><title>ESP32CAM WiFi Configuration</title>"
        "<meta charset=\"utf-8\">"
        "<style>"
        "body { font-family: Arial, sans-serif; margin: 40px; background-color: #f0f0f0; }"
        ".container { max-width: 600px; margin: 0 auto; background-color: white; padding: 30px; border-radius: 8px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }"
        "h1 { color: #333; text-align: center; }"
        "form { margin-top: 20px; }"
        "label { display: block; margin: 10px 0 5px; font-weight: bold; }"
        "input[type='text'], input[type='password'], select { width: 100%; padding: 10px; margin-bottom: 15px; border: 1px solid #ddd; border-radius: 4px; box-sizing: border-box; }"
        "button { background-color: #4CAF50; color: white; padding: 12px 20px; border: none; border-radius: 4px; cursor: pointer; font-size: 16px; }"
        "button:hover { background-color: #45a049; }"
        ".scan-btn { background-color: #2196F3; color: white; padding: 12px 20px; border: none; border-radius: 4px; cursor: pointer; font-size: 16px; margin-top: 10px; }"
        ".scan-btn:hover { background-color: #1a7fd9; }"
        ".status { margin-top: 20px; padding: 10px; border-radius: 4px; }"
        ".success { background-color: #dff0d8; color: #3c763d; }"
        ".error { background-color: #f2dede; color: #a94442; }"
        ".scan-info { margin-top: 10px; padding: 10px; border-radius: 4px; background-color: #e3f2fd; font-size: 14px; }"
        ".scan-info strong { color: #1976d2; }"
        "</style>"
        "</head>"
        "<body>"
        "<div class=\"container\">"
        "<h1>ESP32CAM WiFi Configuration</h1>"
        "<p>请输入或选择要连接的WiFi网络：</p>"
        "<button class=\"scan-btn\" onclick=\"scanWifi()\">扫描WiFi网络</button>"
        "<div id=\"scanInfo\" class=\"scan-info\" style=\"display:none;\"></div>"
        "<form id=\"configForm\" action=\"/save_wifi\" method=\"post\">"
        "<label for=\"ssid\">WiFi名称 (SSID):</label>"
        "<input type=\"text\" id=\"ssid\" name=\"ssid\" required placeholder=\"请输入WiFi名称或点击扫描按钮\">"

        "<label for=\"wifiSelect\">或从扫描结果中选择：</label>"
        "<select id=\"wifiSelect\" onchange=\"selectWifi()\">"
        "<option value=\"\">-- 请先扫描WiFi网络 --</option>"
        "</select>"

        "<label for=\"password\">WiFi密码:</label>"
        "<input type=\"password\" id=\"password\" name=\"password\" placeholder=\"请输入WiFi密码\">"

        "<button type=\"submit\">保存并连接</button>"
        "</form>"
        "<div id=\"result\" class=\"status\" style=\"display:none;\"></div>"
        "</div>"

        "<script>"
        "function scanWifi() {"
        "    const scanBtn = document.querySelector('.scan-btn');"
        "    scanBtn.disabled = true;"
        "    scanBtn.textContent = '扫描中...';"
        "    fetch('/scan')"
        "    .then(response => response.json())"
        "    .then(data => {"
        "        const select = document.getElementById('wifiSelect');"
        "        const infoDiv = document.getElementById('scanInfo');"
        "        select.innerHTML = '<option value=\"\">-- 请选择WiFi网络 --</option>';"
        "        if (data.length > 0) {"
        "            for (let i = 0; i < data.length; i++) {"
        "                const authText = data[i].authmode == 0 ? 'Open' : 'WPA2';"
        "                const option = document.createElement('option');"
        "                option.value = data[i].ssid;"
        "                option.textContent = data[i].ssid + ' (' + authText + ', 信号: ' + data[i].rssi + ')';"
        "                select.appendChild(option);"
        "            }"
        "            infoDiv.innerHTML = '<strong>扫描完成！</strong> 找到 ' + data.length + ' 个WiFi网络，请从下拉列表中选择。';"
        "            infoDiv.style.display = 'block';"
        "        } else {"
        "            infoDiv.innerHTML = '<strong>未扫描到WiFi网络</strong>，请检查设备是否在WiFi覆盖范围内。';"
        "            infoDiv.style.display = 'block';"
        "        }"
        "    })"
        "    .catch(error => {"
        "        console.error('Scan error:', error);"
        "        const infoDiv = document.getElementById('scanInfo');"
        "        infoDiv.innerHTML = '<strong>扫描失败：</strong> ' + error.message;"
        "        infoDiv.style.display = 'block';"
        "    })"
        "    .finally(() => {"
        "        scanBtn.disabled = false;"
        "        scanBtn.textContent = '扫描WiFi网络';"
        "    });"
        "}"
        "function selectWifi() {"
        "    const select = document.getElementById('wifiSelect');"
        "    const ssidInput = document.getElementById('ssid');"
        "    const selectedValue = select.value;"
        "    if (selectedValue) {"
        "        ssidInput.value = selectedValue;"
        "    }"
        "}"
        "document.getElementById('configForm').addEventListener('submit', function(e) {"
        "    e.preventDefault();"
        "    const formData = new FormData(this);"
        "    const data = {};"
        "    for (let [key, value] of formData.entries()) {"
        "        data[key] = value;"
        "    }"
        "    fetch('/save_wifi', {"
        "        method: 'POST',"
        "        body: JSON.stringify(data)"
        "    })"
        "    .then(response => response.json())"
        "    .then(data => {"
        "        const resultDiv = document.getElementById('result');"
        "        resultDiv.style.display = 'block';"
        "        if (data.success) {"
        "            resultDiv.className = 'status success';"
        "            resultDiv.innerHTML = '配置保存成功！设备将在几秒内重启并连接到新网络。';"
        "        } else {"
        "            resultDiv.className = 'status error';"
        "            resultDiv.innerHTML = '保存失败: ' + data.message;"
        "        }"
        "    });"
        "});"
        "</script>"
        "</body>"
        "</html>";

    httpd_resp_send(req, html_response, strlen(html_response));
    return ESP_OK;
}

static void restart_task(void* pvParameter)
{
    vTaskDelay(pdMS_TO_TICKS(2000));  // 等待2秒让响应返回
    esp_restart();
}

static esp_err_t save_wifi_handler(httpd_req_t* req)
{
    static char content[200];  // 使用静态存储以减少栈使用
    int ret, remaining = req->content_len;

    wifi_credentials_t creds = {0};

    // 读取请求体
    while (remaining > 0) {
        ret = httpd_req_recv(req, content, MIN(remaining, sizeof(content) - 1));
        if (ret <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            return ESP_FAIL;
        }
        content[ret] = '\0';
        break;
    }

    // 解析JSON
    cJSON* root = cJSON_Parse(content);
    if (!root) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "{\"success\": false, \"message\": \"Invalid JSON\"}");
        return ESP_OK;  // 返回成功状态，避免连接挂起
    }

    cJSON* ssid_json = cJSON_GetObjectItem(root, "ssid");
    cJSON* pass_json = cJSON_GetObjectItem(root, "password");

    if (!ssid_json || !cJSON_IsString(ssid_json)) {
        cJSON_Delete(root);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "{\"success\": false, \"message\": \"SSID is required\"}");
        return ESP_FAIL;
    }

    strncpy(creds.ssid, ssid_json->valuestring, sizeof(creds.ssid) - 1);
    if (pass_json && cJSON_IsString(pass_json)) {
        strncpy(creds.password, pass_json->valuestring, sizeof(creds.password) - 1);
    }
    else {
        creds.password[0] = '\0';  // 空密码
    }

    cJSON_Delete(root);

    // 保存到NVS
    esp_err_t err = wifi_config_save_credentials(&creds);
    if (err != ESP_OK) {
        const char* ename = esp_err_to_name(err);
        char resp[256];
        snprintf(resp, sizeof(resp), "{\"success\": false, \"message\": \"Failed to save credentials: %s\"}", ename);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, resp);
        return ESP_OK;  // 返回成功状态，避免连接挂起
    }

    // 尝试使用新凭证直接连接
    esp_err_t conn_res = wifi_connect_to_ap(creds.ssid, creds.password);

    httpd_resp_set_type(req, "application/json");
    if (conn_res == ESP_OK) {
        httpd_resp_sendstr(req, "{\"success\": true, \"message\": \"Connected to WiFi, device will restart\"}");
        // 先创建重启任务
        xTaskCreate(restart_task, "restart_task", 4096, NULL, 5, NULL);
        // 然后停止配置模式
        stop_provisioning_mode();
    }
    else {
        // 连接失败快速闪烁提示
        led_set_state(LED_STATE_BLINK_FAST);
        httpd_resp_sendstr(req, "{\"success\": false, \"message\": \"Failed to connect to network\"}");
        // 可选：保留 portal 让用户重新提交或选择重启
    }

    return ESP_OK;
}

static esp_err_t connectivity_check_handler_android(httpd_req_t* req)
{
    // Android连接检查：重定向到配置页面
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t connectivity_check_handler_ios(httpd_req_t* req)
{
    // iOS连接检查：重定向到配置页面
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

// Windows连接检查处理器
static esp_err_t connectivity_check_handler_windows(httpd_req_t* req)
{
    // Windows连接检查：重定向到配置页面
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

// 通用重定向处理器 - 捕获所有其他请求
static esp_err_t redirect_handler(httpd_req_t* req)
{
    // 重定向到配置页面
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static httpd_uri_t index_uri = {.uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL};

static httpd_uri_t save_wifi_uri = {.uri = "/save_wifi", .method = HTTP_POST, .handler = save_wifi_handler, .user_ctx = NULL};

static httpd_uri_t android_connectivity_uri = {.uri = "/generate_204", .method = HTTP_GET, .handler = connectivity_check_handler_android, .user_ctx = NULL};

static httpd_uri_t ios_connectivity_uri = {.uri = "/hotspot-detect.html", .method = HTTP_GET, .handler = connectivity_check_handler_ios, .user_ctx = NULL};

static httpd_uri_t ios_connectivity_uri2 = {.uri = "/library/test/success.html", .method = HTTP_GET, .handler = connectivity_check_handler_ios, .user_ctx = NULL};

static httpd_uri_t windows_connectivity_uri1 = {.uri = "/connecttest", .method = HTTP_GET, .handler = connectivity_check_handler_windows, .user_ctx = NULL};

static httpd_uri_t windows_connectivity_uri2 = {.uri = "/fwlink", .method = HTTP_GET, .handler = connectivity_check_handler_windows, .user_ctx = NULL};

// WiFi扫描处理器
static esp_err_t scan_handler(httpd_req_t* req)
{
    ESP_LOGI(TAG, "WiFi scan requested");

    // 喂看门狗，防止扫描时间过长导致复位
    esp_task_wdt_reset();

    // 启动WiFi扫描（阻塞式）
    esp_err_t err = esp_wifi_scan_start(NULL, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start scan: %s", esp_err_to_name(err));
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "[]");
        return ESP_OK;
    }

    // 获取扫描到的AP数量
    uint16_t ap_num = 0;
    err = esp_wifi_scan_get_ap_num(&ap_num);
    if (err != ESP_OK || ap_num == 0) {
        ESP_LOGI(TAG, "No APs found or error getting AP count");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "[]");
        return ESP_OK;
    }

    // 限制最大扫描结果数量
    if (ap_num > MAX_SCAN_RESULTS) {
        ap_num = MAX_SCAN_RESULTS;
    }

    // 分配内存存储扫描结果
    wifi_ap_record_t* ap_list = (wifi_ap_record_t*)malloc(sizeof(wifi_ap_record_t) * ap_num);
    if (ap_list == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for scan results");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "[]");
        return ESP_OK;
    }

    // 获取扫描结果
    err = esp_wifi_scan_get_ap_records(&ap_num, ap_list);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get AP records: %s", esp_err_to_name(err));
        free(ap_list);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "[]");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Scan completed, found %d APs", ap_num);

    // 构建JSON响应
    cJSON* root = cJSON_CreateArray();

    for (int i = 0; i < ap_num; i++) {
        // 跳过空SSID
        if (strlen((char*)ap_list[i].ssid) == 0) {
            continue;
        }

        cJSON* ap = cJSON_CreateObject();
        cJSON_AddStringToObject(ap, "ssid", (char*)ap_list[i].ssid);
        cJSON_AddNumberToObject(ap, "rssi", ap_list[i].rssi);
        cJSON_AddNumberToObject(ap, "authmode", ap_list[i].authmode);
        cJSON_AddItemToArray(root, ap);
    }

    char* json_str = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json_str);

    free(json_str);
    cJSON_Delete(root);
    free(ap_list);

    return ESP_OK;
}

// 通用重定向URI - 捕获所有其他请求
static httpd_uri_t redirect_uri = {.uri = "/*", .method = HTTP_GET, .handler = redirect_handler, .user_ctx = NULL};

static esp_err_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.max_open_sockets = 5;   // 增加最大连接数
    config.max_uri_handlers = 16;  // 增加URI处理器槽位
    config.task_priority = tskIDLE_PRIORITY + 5;
    config.stack_size = 8192;  // 增加HTTP服务器任务栈大小

    ESP_LOGI(TAG, "Starting HTTP server on port: %d", config.server_port);
    if (httpd_start(&s_server, &config) == ESP_OK) {
        // 注册主要URI处理器
        httpd_register_uri_handler(s_server, &index_uri);
        httpd_register_uri_handler(s_server, &save_wifi_uri);

        // 注册WiFi扫描处理器
        httpd_uri_t scan_uri = {.uri = "/scan", .method = HTTP_GET, .handler = scan_handler, .user_ctx = NULL};
        httpd_register_uri_handler(s_server, &scan_uri);

        // 注册设备连接检查处理器
        httpd_register_uri_handler(s_server, &android_connectivity_uri);
        httpd_register_uri_handler(s_server, &ios_connectivity_uri);
        httpd_register_uri_handler(s_server, &ios_connectivity_uri2);
        httpd_register_uri_handler(s_server, &windows_connectivity_uri1);
        httpd_register_uri_handler(s_server, &windows_connectivity_uri2);

        // 注册通用重定向处理器（必须在最后注册，以捕获所有其他请求）
        httpd_register_uri_handler(s_server, &redirect_uri);

        ESP_LOGI(TAG, "HTTP server started successfully");
        return ESP_OK;
    }
    ESP_LOGE(TAG, "Failed to start HTTP server");
    return ESP_FAIL;
}

static esp_err_t stop_webserver(void)
{
    return httpd_stop(s_server);
}

esp_err_t wifi_config_manager_init(int button_gpio, EventGroupHandle_t event_group, esp_netif_t* ap_netif)
{
    s_button_gpio = button_gpio;

    /* 保存外部传入的 AP netif 句柄 */
    s_ap_netif = ap_netif;

    ESP_LOGI(TAG, "Initializing WiFi config manager with button on GPIO %d", s_button_gpio);

    // 初始化GPIO
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << s_button_gpio);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;

    // 中断模式：使用下降沿触发
    io_conf.intr_type = GPIO_INTR_NEGEDGE;  // 下降沿触发（按钮按下时电平从高变低）
    ESP_LOGI(TAG, "Using interrupt mode for button (GPIO %d)", s_button_gpio);

    gpio_config(&io_conf);

    // 安装GPIO中断服务（只安装一次）
    if (!s_isr_service_installed) {
        esp_err_t ret = gpio_install_isr_service(0);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        }
        else {
            s_isr_service_installed = true;
            ESP_LOGI(TAG, "GPIO ISR service installed successfully");
        }
    }

    esp_err_t isr_ret = gpio_isr_handler_add(s_button_gpio, button_isr_handler, NULL);
    if (isr_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add GPIO ISR handler for pin %d: %s", s_button_gpio, esp_err_to_name(isr_ret));
    }
    else {
        ESP_LOGI(TAG, "GPIO ISR handler added for pin %d", s_button_gpio);
    }

    // 创建配置任务
    xTaskCreate(wifi_config_check_button_task, "wifi_config_btn", 4096, NULL, 5, &s_prov_task_handle);

    // 尝试加载已保存的WiFi配置
    wifi_credentials_t creds;
    esp_err_t load_ret = wifi_config_load_credentials(&creds);
    ESP_LOGI(TAG, "wifi_config_load_credentials returned: %s", esp_err_to_name(load_ret));

    if (load_ret == ESP_OK) {
        ESP_LOGI(TAG, "Loaded saved WiFi credentials, attempting to connect...");
        ESP_LOGI(TAG, "SSID: '%s', Password length: %d", creds.ssid, strlen(creds.password));
        if (wifi_connect_to_ap(creds.ssid, creds.password) == ESP_OK) {
            ESP_LOGI(TAG, "Successfully connected to saved WiFi network");
            return ESP_OK;
        }
        else {
            ESP_LOGI(TAG, "Failed to connect to saved WiFi, entering provisioning mode");
            start_provisioning_mode();
        }
    }
    else {
        // 没有保存的WiFi凭据，直接进入配置模式
        ESP_LOGI(TAG, "No saved WiFi credentials, entering provisioning mode");
        start_provisioning_mode();
    }

    return ESP_OK;
}

static void wifi_config_check_button_task(void* arg)
{
    ESP_LOGI(TAG, "Button check task started, waiting for GPIO %d interrupt...", s_button_gpio);

    while (1) {
        // 中断模式：等待GPIO中断
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        ESP_LOGI(TAG, "GPIO %d interrupt triggered!", s_button_gpio);

        // 记录按下开始时间
        TickType_t press_start = xTaskGetTickCount();

        // 检查按钮是否仍然被按下（低电平表示按下）
        vTaskDelay(WIFI_PROV_HOLD_TIME);

        int level = gpio_get_level(s_button_gpio);
        ESP_LOGI(TAG, "GPIO %d level after delay: %d (0=pressed, 1=released)", s_button_gpio, level);

        if (level == 0) {
            // 按钮仍在按下状态，启动配置模式
            TickType_t press_duration = xTaskGetTickCount() - press_start;

            if (press_duration >= WIFI_PROV_HOLD_TIME) {
                ESP_LOGI(TAG, "Long press detected (%d ms), entering provisioning mode s_provisioning_mode=%d", (int)(press_duration * portTICK_PERIOD_MS), s_provisioning_mode);
                start_provisioning_mode();
            }
        }
        else {
            ESP_LOGI(TAG, "Button released before hold time, ignoring");
        }
    }
}

static esp_event_handler_instance_t s_wifi_ap_conn_inst = NULL;
static esp_event_handler_instance_t s_wifi_ap_disc_inst = NULL;
static esp_event_handler_instance_t s_ip_event_handler_inst = NULL;

static void start_provisioning_mode(void)
{
    s_provisioning_mode = true;

    ESP_LOGI(TAG, "Starting provisioning mode");
    // 停止UDP图像传输（WiFi即将重置）
    stop_udp_camera();
    // 配置模式下快速闪烁
    led_set_state(LED_STATE_BLINK_FAST);

    // 断开STA连接，避免与AP功能冲突
    esp_wifi_disconnect();
    ESP_LOGI(TAG, "Disconnected from STA network");

    // 初始化mDNS
    mdns_init();
    mdns_hostname_set("esp32cam-config");
    mdns_instance_name_set("ESP32CAM Config Portal");

    // 注意：AP配置已在softap_sta.c中初始化，这里不再重复设置
    // 只需启动HTTP服务器和其他服务

    // 获取真实 MAC 的后三字节生成唯一SSID（不依赖 esp_wifi 已初始化）
    uint8_t mac[6];
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK) {
        mac[3] = 0x12;
        mac[4] = 0x34;
        mac[5] = 0x56;  // fallback
    }
    char ap_ssid[64];
    snprintf(ap_ssid, sizeof(ap_ssid), "%s%02X%02X%02X", PROV_AP_SSID_PREFIX, mac[3], mac[4], mac[5]);

    wifi_config_t wifi_ap_config;
    memset(&wifi_ap_config, 0, sizeof(wifi_config_t));

    wifi_ap_config.ap.ssid_len = (uint8_t)strlen(ap_ssid);
    wifi_ap_config.ap.channel = PROV_AP_CHANNEL;
    wifi_ap_config.ap.max_connection = 4;
    wifi_ap_config.ap.authmode = WIFI_AUTH_OPEN;
    strncpy((char*)wifi_ap_config.ap.ssid, ap_ssid, sizeof(wifi_ap_config.ap.ssid) - 1);
    wifi_ap_config.ap.password[0] = '\0';  // 无密码

    ESP_LOGI(TAG, "Starting AP with SSID: %s (open network)", ap_ssid);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config));

    // 注册事件处理器（只注册AP相关事件），保存实例用于后续注销
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_AP_STACONNECTED, &wifi_prov_event_handler, NULL, &s_wifi_ap_conn_inst));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, &wifi_prov_event_handler, NULL, &s_wifi_ap_disc_inst));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &wifi_prov_event_handler, NULL, &s_ip_event_handler_inst));

    // 获取AP的IP信息（如果 netif 可用）
    if (s_ap_netif) {
        esp_netif_get_ip_info(s_ap_netif, &s_ap_ip_info);
        ESP_LOGI(TAG, "Provisioning AP IP: " IPSTR, IP2STR(&s_ap_ip_info.ip));
    }

    // 启动HTTP服务器
    start_webserver();

    // 启用NetBIOS名称服务
    netbiosns_init();
    netbiosns_set_name("esp32cam-config");


    ESP_LOGI(TAG, "Provisioning mode started. Connect to '%s' (open network)", ap_ssid);
}

static void stop_provisioning_mode(void)
{
    s_provisioning_mode = false;

    ESP_LOGI(TAG, "Stopping provisioning mode");

    stop_webserver();


    // 重启UDP图像传输（WiFi已重新连接）
    restart_udp_camera();

    // 注销事件处理器（如果已注册）
    if (s_wifi_ap_conn_inst) {
        esp_event_handler_instance_unregister(WIFI_EVENT, WIFI_EVENT_AP_STACONNECTED, s_wifi_ap_conn_inst);
        s_wifi_ap_conn_inst = NULL;
    }
    if (s_wifi_ap_disc_inst) {
        esp_event_handler_instance_unregister(WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, s_wifi_ap_disc_inst);
        s_wifi_ap_disc_inst = NULL;
    }
    if (s_ip_event_handler_inst) {
        esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, s_ip_event_handler_inst);
        s_ip_event_handler_inst = NULL;
    }

    // 移除GPIO中断处理器
    gpio_isr_handler_remove(s_button_gpio);

    // 不在此处停止全局 WiFi 或销毁 netif，由 app_main 统一管理
    // 结束配置模式，若已连接则常亮
    led_set_state(LED_STATE_ON);
    ESP_LOGI(TAG, "Provisioning mode stopped");
}

bool get_wifi_provisioning_mode(void)
{
    return s_provisioning_mode;
}

/**
 * @brief 清除NVS中保存的WiFi凭据（包括ESP32 WiFi驱动自动保存的）
 * @return esp_err_t
 */
esp_err_t wifi_config_clear_all_credentials(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // 清除wifi_config命名空间下的凭据
    err = nvs_open(WIFI_CONFIG_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        nvs_erase_all(nvs_handle);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "Cleared wifi_config namespace");
    }

    // 清除ESP32 WiFi驱动使用的默认命名空间
    err = nvs_open("nvs.net80211", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        nvs_erase_all(nvs_handle);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "Cleared nvs.net80211 namespace (ESP32 WiFi driver credentials)");
    }

    return ESP_OK;
}

esp_err_t wifi_config_save_credentials(wifi_credentials_t* creds)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    if (creds == NULL) {
        ESP_LOGE(TAG, "wifi_config_save_credentials: NULL creds");
        return ESP_ERR_INVALID_ARG;
    }

    if (creds->ssid[0] == '\0') {
        ESP_LOGE(TAG, "wifi_config_save_credentials: empty SSID");
        return ESP_ERR_INVALID_ARG;
    }

    err = nvs_open(WIFI_CONFIG_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_str(nvs_handle, WIFI_CONFIG_SSID_KEY, creds->ssid);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_str(ssid) failed: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    /* 如果密码为空，写入空字符串 */
    err = nvs_set_str(nvs_handle, WIFI_CONFIG_PASS_KEY, creds->password);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_str(password) failed: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_commit failed: %s", esp_err_to_name(err));
    }
    nvs_close(nvs_handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "WiFi credentials saved to NVS (SSID='%s')", creds->ssid);
    }

    return err;
}

esp_err_t wifi_config_load_credentials(wifi_credentials_t* creds)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    size_t ssid_len, pass_len;

    err = nvs_open(WIFI_CONFIG_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No NVS namespace '%s' or open failed: %s", WIFI_CONFIG_NAMESPACE, esp_err_to_name(err));
        return err;
    }

    // 获取SSID长度
    err = nvs_get_str(nvs_handle, WIFI_CONFIG_SSID_KEY, NULL, &ssid_len);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved SSID in NVS");
        nvs_close(nvs_handle);
        return ESP_ERR_NVS_NOT_FOUND;
    }
    else if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_get_str(ssid) failed: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    // 获取密码长度
    err = nvs_get_str(nvs_handle, WIFI_CONFIG_PASS_KEY, NULL, &pass_len);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "nvs_get_str(password) failed: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    // 读取SSID
    err = nvs_get_str(nvs_handle, WIFI_CONFIG_SSID_KEY, creds->ssid, &ssid_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_get_str(ssid) read failed: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    // 读取密码（如果存在）
    if (pass_len > 0) {
        err = nvs_get_str(nvs_handle, WIFI_CONFIG_PASS_KEY, creds->password, &pass_len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "nvs_get_str(password) read failed: %s", esp_err_to_name(err));
            nvs_close(nvs_handle);
            return err;
        }
    }
    else {
        creds->password[0] = '\0';  // 设置为空字符串
    }

    nvs_close(nvs_handle);

    ESP_LOGI(TAG, "WiFi credentials loaded from NVS: SSID='%s'", creds->ssid);

    return ESP_OK;
}

static esp_err_t wifi_connect_to_ap(const char* ssid, const char* password)
{
    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    if (password && strlen(password) > 0) {
        strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    }
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_connect());

    EventGroupHandle_t event_group = wifi_get_event_group();
    if (event_group == NULL) {
        ESP_LOGW(TAG, "No event group available; returning after esp_wifi_connect");
        return ESP_OK;
    }

    EventBits_t bits = xEventGroupWaitBits(event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(10000)  // 等待10秒
    );

    bool connected = (bits & WIFI_CONNECTED_BIT) != 0;
    return connected ? ESP_OK : ESP_FAIL;
}

const esp_netif_ip_info_t* wifi_get_ap_ip_info(void)
{
    return &s_ap_ip_info;
}
