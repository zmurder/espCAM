/*
 * DNS Server for Captive Portal
 *
 * This module implements a DNS server that redirects all DNS queries
 * to the AP IP address, enabling captive portal functionality.
 */

#ifndef DNS_SERVER_H
#define DNS_SERVER_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化DNS服务器
 * @return ESP_OK on success
 */
esp_err_t dns_server_init(void);

/**
 * @brief 启动DNS服务器
 * @return ESP_OK on success
 */
esp_err_t dns_server_start(void);

/**
 * @brief 停止DNS服务器
 * @return ESP_OK on success
 */
esp_err_t dns_server_stop(void);

/**
 * @brief 检查DNS服务器是否正在运行
 * @return true if running, false otherwise
 */
bool dns_server_is_running(void);

/**
 * @brief 设置DNS服务器的AP IP地址
 * @param ap_ip AP IP地址（网络字节序）
 * @return ESP_OK on success
 */
esp_err_t dns_server_set_ap_ip(uint32_t ap_ip);

#ifdef __cplusplus
}
#endif

#endif // DNS_SERVER_H
