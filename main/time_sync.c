/**
 * time_sync.c
 * SNTP 网络对时：STA 拿到 IP 后向 NTP 服务器同步，
 * 同步后 time()/localtime() 为真实本地时间（CST-8），日志时间戳随之变为真实日期时间
 * （配合 CONFIG_LOG_TIMESTAMP_SOURCE_SYSTEM=y，见 sdkconfig.defaults）
 */

#include <time.h>
#include <stdbool.h>
#include <sys/time.h>

#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_sntp.h"

#include "posture_sched.h"

static const char* TAG = "TIME_SYNC";

// "刚完成对时"事件标志：SNTP 回调（tcpip_thread 上下文）只置位，
// 由 20003 任务轮询取走后做 UDP 推送——回调里直接 sendto 会自等待死锁
static volatile bool s_just_synced = false;

// 同步完成回调：打印一次当前时间，方便确认对时成功
static void on_time_sync(struct timeval* tv)
{
    struct tm timeinfo;
    localtime_r(&tv->tv_sec, &timeinfo);
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    ESP_LOGI(TAG, "Time synced: %s (CST)", buf);
    posture_sched_notify_time_synced();  // 调度模块播报生效时段（纯内存操作，安全）
    s_just_synced = true;                // UDP 推送由 20003 任务经 time_sync_pop_event() 完成
}

// STA 拿到 IP（含配网后重连）即启动 SNTP；重复调用时幂等
static void on_got_ip(void* arg, esp_event_base_t base, int32_t event_id, void* event_data)
{
    if (esp_sntp_enabled()) {
        return;
    }
    ESP_LOGI(TAG, "Got IP, starting SNTP sync...");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "ntp.aliyun.com");  // 主：国内 NTP，快
    esp_sntp_setservername(1, "pool.ntp.org");    // 备：国际公共池
    esp_sntp_set_time_sync_notification_cb(on_time_sync);
    esp_sntp_init();
}

void time_sync_init(void)
{
    setenv("TZ", "CST-8", 1);  // 中国时区 UTC+8（SNTP 返回 UTC，localtime 自动转换）
    tzset();
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_got_ip, NULL));
    ESP_LOGI(TAG, "Time sync init OK (SNTP on got-IP, TZ=CST-8)");
}

bool time_sync_pop_event(void)
{
    // 取走"刚完成对时"事件（单写单读，volatile 足够）；true 时调用方需推送状态到 PC
    bool ev = s_just_synced;
    s_just_synced = false;
    return ev;
}

bool time_sync_is_done(void)
{
    time_t now = 0;
    struct tm timeinfo = { 0 };
    time(&now);
    localtime_r(&now, &timeinfo);
    return timeinfo.tm_year >= (2020 - 1900);  // 未同步时为 1970 年
}
