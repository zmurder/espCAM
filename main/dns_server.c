/*
 * DNS Server for Captive Portal Implementation
 *
 * This module implements a DNS server that redirects all DNS queries
 * to the AP IP address, enabling captive portal functionality.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "dns_server.h"

static const char* TAG = "dns_server";

#define DNS_SERVER_PORT 53
#define DNS_MAX_LEN 512
#define DNS_TASK_STACK_SIZE 4096
#define DNS_TASK_PRIORITY 5

// DNS服务器状态
static bool s_dns_server_running = false;
static TaskHandle_t s_dns_task_handle = NULL;
static int s_dns_socket = -1;
static uint32_t s_ap_ip = 0xC0A80401;  // 默认192.168.4.1

// DNS header structure
#pragma pack(push, 1)
typedef struct {
    uint16_t id;
    uint16_t flags;
    uint16_t qdcount;
    uint16_t ancount;
    uint16_t nscount;
    uint16_t arcount;
} dns_header_t;

typedef struct {
    uint16_t type;
    uint16_t class;
} dns_question_t;

typedef struct {
    uint16_t name_ptr;
    uint16_t type;
    uint16_t class;
    uint32_t ttl;
    uint16_t length;
    uint32_t addr;
} dns_answer_t;
#pragma pack(pop)

/**
 * @brief DNS服务器任务
 */
static void dns_server_task(void* pvParameters)
{
    char rx_buffer[DNS_MAX_LEN];
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(DNS_SERVER_PORT);

    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        s_dns_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    s_dns_socket = sock;

    // 设置socket选项
    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    int err = bind(sock, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(sock);
        s_dns_socket = -1;
        s_dns_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "DNS server started on port %d", DNS_SERVER_PORT);

    while (s_dns_server_running) {
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);

        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr*)&source_addr, &socklen);

        if (len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            break;
        }
        else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
            break;
        }
        else {
            inet_ntoa_r(source_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
            ESP_LOGI(TAG, "Received DNS query from %s, length: %d", addr_str, len);

            // 解析DNS请求
            if (len >= sizeof(dns_header_t)) {
                dns_header_t* header = (dns_header_t*)rx_buffer;

                // 只响应查询请求
                if ((ntohs(header->flags) & 0x8000) == 0) {
                    // 构建DNS响应
                    char tx_buffer[DNS_MAX_LEN];
                    memset(tx_buffer, 0, sizeof(tx_buffer));

                    // 复制DNS header
                    dns_header_t* resp_header = (dns_header_t*)tx_buffer;
                    memcpy(resp_header, header, sizeof(dns_header_t));

                    // 设置响应标志
                    resp_header->flags = htons(0x8180);  // 标准响应，无错误
                    resp_header->qdcount = header->qdcount;
                    resp_header->ancount = htons(1);  // 1个答案
                    resp_header->nscount = 0;
                    resp_header->arcount = 0;

                    // 复制问题部分
                    int tx_len = sizeof(dns_header_t);
                    int qd_len = len - sizeof(dns_header_t);

                    if (qd_len > 0 && tx_len + qd_len < sizeof(tx_buffer)) {
                        memcpy(tx_buffer + tx_len, rx_buffer + sizeof(dns_header_t), qd_len);
                        tx_len += qd_len;

                        // 添加答案部分
                        dns_answer_t* answer = (dns_answer_t*)(tx_buffer + tx_len);
                        answer->name_ptr = htons(0xC00C);  // 指向问题中的域名
                        answer->type = htons(1);  // A记录
                        answer->class = htons(1);  // IN类
                        answer->ttl = htonl(60);  // TTL 60秒
                        answer->length = htons(4);  // IPv4地址长度
                        answer->addr = s_ap_ip;  // 使用动态AP IP

                        tx_len += sizeof(dns_answer_t);

                        // 发送响应
                        int tolen = sizeof(source_addr);
                        int sent = sendto(sock, tx_buffer, tx_len, 0, (struct sockaddr*)&source_addr, tolen);
                        
                        // 将uint32_t转换为esp_ip4_addr_t用于打印
                        esp_ip4_addr_t ip_addr;
                        ip_addr.addr = s_ap_ip;
                        
                        ESP_LOGI(TAG, "DNS query from %s, responded with " IPSTR ", sent %d bytes", addr_str, IP2STR(&ip_addr), sent);
                    }
                }
            }
        }
    }

    ESP_LOGI(TAG, "DNS server task exiting");
    close(sock);
    s_dns_socket = -1;
    s_dns_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t dns_server_init(void)
{
    ESP_LOGI(TAG, "DNS server initialized");
    return ESP_OK;
}

esp_err_t dns_server_start(void)
{
    if (s_dns_server_running) {
        ESP_LOGW(TAG, "DNS server already running");
        return ESP_OK;
    }

    s_dns_server_running = true;

    BaseType_t ret = xTaskCreate(dns_server_task, "dns_server", DNS_TASK_STACK_SIZE, NULL, DNS_TASK_PRIORITY, &s_dns_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create DNS server task");
        s_dns_server_running = false;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "DNS server start requested");
    return ESP_OK;
}

esp_err_t dns_server_stop(void)
{
    if (!s_dns_server_running) {
        ESP_LOGW(TAG, "DNS server not running");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Stopping DNS server");
    s_dns_server_running = false;

    // 关闭socket以解除recvfrom阻塞
    if (s_dns_socket >= 0) {
        shutdown(s_dns_socket, SHUT_RDWR);
        close(s_dns_socket);
        s_dns_socket = -1;
    }

    // 等待任务退出
    if (s_dns_task_handle != NULL) {
        // 等待任务退出，最多2秒
        int timeout = 20;
        while (s_dns_task_handle != NULL && timeout-- > 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (s_dns_task_handle != NULL) {
            ESP_LOGW(TAG, "DNS server task did not exit gracefully");
            s_dns_task_handle = NULL;
        }
    }

    ESP_LOGI(TAG, "DNS server stopped");
    return ESP_OK;
}

bool dns_server_is_running(void)
{
    return s_dns_server_running;
}

esp_err_t dns_server_set_ap_ip(uint32_t ap_ip)
{
    s_ap_ip = ap_ip;
    
    // 将uint32_t转换为esp_ip4_addr_t用于打印
    esp_ip4_addr_t ip_addr;
    ip_addr.addr = ap_ip;
    
    ESP_LOGI(TAG, "DNS server AP IP set to: " IPSTR, IP2STR(&ip_addr));
    return ESP_OK;
}
