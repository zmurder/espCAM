/*
 * WiFi Configuration Manager Header
 *
 * This module handles dynamic WiFi configuration via a captive portal.
 */

#ifndef WIFI_CONFIG_MANAGER_H
#define WIFI_CONFIG_MANAGER_H

#include "esp_err.h"
#include "esp_netif.h"
#include <stdbool.h>
#include "freertos/event_groups.h"
#include "wifi_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Structure to store WiFi credentials
 */
typedef struct
{
    char ssid[64];
    char password[64];
} wifi_credentials_t;

/**
 * @brief Initialize WiFi configuration manager
 *
 * @param button_gpio GPIO pin connected to the configuration button
 * @param event_group Event group handle for WiFi connection status
 * @param ap_netif Pointer to the AP network interface
 * @return esp_err_t
 */
esp_err_t wifi_config_manager_init(int button_gpio, EventGroupHandle_t event_group, esp_netif_t* ap_netif);

/**
 * @brief Check if device is in configuration mode
 *
 * @return true if in configuration mode, false otherwise
 */
bool get_wifi_provisioning_mode(void);

/**
 * @brief Save WiFi credentials to NVS
 *
 * @param creds WiFi credentials to save
 * @return esp_err_t
 */
esp_err_t wifi_config_save_credentials(wifi_credentials_t* creds);

/**
 * @brief Load WiFi credentials from NVS
 *
 * @param creds Pointer to wifi_credentials_t to store loaded credentials
 * @return esp_err_t
 */
esp_err_t wifi_config_load_credentials(wifi_credentials_t* creds);


/**
 * @brief Get the SoftAP IP address
 *
 * @return esp_netif_ip_info_t* IP information of the SoftAP
 */
const esp_netif_ip_info_t* wifi_get_ap_ip_info(void);

/**
 * @brief Clear all WiFi credentials from NVS (including ESP32 WiFi driver saved credentials)
 *
 * @return esp_err_t
 */
esp_err_t wifi_config_clear_all_credentials(void);

#ifdef __cplusplus
}
#endif

#endif  // WIFI_CONFIG_MANAGER_H