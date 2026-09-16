/**
 * posture_sched.c
 * 坐姿检测时间段调度实现：
 * - 每日重复窗口（最多 5 段，支持跨午夜），NVS 持久化（断电重启后仍生效）
 * - 时间未同步（SNTP 未完成）→ 不判断窗口，全部检测（状态打印 + 发给 PC）
 * - 协议（20003 端口，与自动发现同通道）：
 *   PC→ESP: ESPCAM_SCHED_SET n HH:MM HH:MM ...（设置+写 NVS）/ ESPCAM_SCHED_GET（查询）
 *   ESP→PC: ESPCAM_SCHED_STATE synced active n HH:MM HH:MM ...（synced/active: 0|1）
 */

#include <string.h>
#include <stdio.h>
#include <time.h>

#include "esp_log.h"
#include "nvs.h"

#include "posture_sched.h"
#include "time_sync.h"
#include "udp_camera_client.h"

static const char* TAG = "POSTURE_SCHED";

#define NVS_NAMESPACE "psched"
#define NVS_KEY_SLOTS "slots"

static posture_sched_slot_t s_slots[POSTURE_SCHED_MAX_SLOTS];
static int s_slot_count = 0;   // 已设置段数（0=未设置=全天检测）
static int s_last_active = -1; // 上次窗口状态：-1=未知（首帧/重同步后），0/1=已知（边沿日志用）

// 纯计算：minute 是否落在任一窗口（start>end 视为跨午夜绕过 00:00）
static bool calc_in_window(int minute)
{
    for (int i = 0; i < s_slot_count; i++) {
        uint16_t s = s_slots[i].start_min;
        uint16_t e = s_slots[i].end_min;
        if (s <= e) {
            if (minute >= s && minute < e) return true;   // 同日窗口 [start, end)
        }
        else {
            if (minute >= s || minute < e) return true;   // 跨午夜窗口
        }
    }
    return false;
}

// 当前是否应检测（无边沿日志副作用，is_active/build_state 共用）
static bool calc_active(void)
{
    if (!time_sync_is_done()) return true;  // 无网络时间：不判断窗口，全检测
    if (s_slot_count == 0) return true;     // 未设置时段：全天检测
    time_t now = 0;
    struct tm ti = { 0 };
    time(&now);
    localtime_r(&now, &ti);
    return calc_in_window(ti.tm_hour * 60 + ti.tm_min);
}

static void save_to_nvs(void)
{
    uint8_t buf[1 + sizeof(s_slots)];
    buf[0] = (uint8_t)s_slot_count;
    memcpy(buf + 1, s_slots, sizeof(posture_sched_slot_t) * s_slot_count);

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        ESP_ERROR_CHECK(nvs_set_blob(h, NVS_KEY_SLOTS, buf, 1 + sizeof(posture_sched_slot_t) * s_slot_count));
        ESP_ERROR_CHECK(nvs_commit(h));
        nvs_close(h);
    }
    else {
        ESP_LOGE(TAG, "NVS open failed, schedule NOT persisted");
    }
}

void posture_sched_init(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "No schedule in NVS, detection always on");
        return;
    }
    uint8_t buf[1 + sizeof(s_slots)];
    size_t len = sizeof(buf);
    if (nvs_get_blob(h, NVS_KEY_SLOTS, buf, &len) == ESP_OK && len >= 1) {
        int n = buf[0];
        if (n > POSTURE_SCHED_MAX_SLOTS) n = POSTURE_SCHED_MAX_SLOTS;
        memcpy(s_slots, buf + 1, sizeof(posture_sched_slot_t) * n);
        s_slot_count = n;
        ESP_LOGI(TAG, "Loaded %d slot(s) from NVS:", n);
        posture_sched_log_slots();
    }
    else {
        ESP_LOGI(TAG, "No schedule in NVS, detection always on");
    }
    nvs_close(h);
}

void posture_sched_log_slots(void)
{
    if (s_slot_count == 0) {
        ESP_LOGI(TAG, "  (no slots, detection always on)");
        return;
    }
    for (int i = 0; i < s_slot_count; i++) {
        ESP_LOGI(TAG, "  slot %d: %02d:%02d-%02d:%02d", i + 1,
                 s_slots[i].start_min / 60, s_slots[i].start_min % 60,
                 s_slots[i].end_min / 60, s_slots[i].end_min % 60);
    }
}

// 时段列表格式化为单行 "HH:MM-HH:MM HH:MM-HH:MM ..."（未设置 → "none(always on)"）
static void format_slots(char* buf, int buf_len)
{
    if (s_slot_count == 0) {
        snprintf(buf, buf_len, "none(always on)");
        return;
    }
    int off = 0;
    for (int i = 0; i < s_slot_count && off < buf_len - 13; i++) {
        off += snprintf(buf + off, buf_len - off, "%s%02d:%02d-%02d:%02d", i ? " " : "",
                        s_slots[i].start_min / 60, s_slots[i].start_min % 60,
                        s_slots[i].end_min / 60, s_slots[i].end_min % 60);
    }
}

// 每帧取图前打印：当前设置时段 + 是否在窗（与每帧姿态日志同密度）
static void log_current_state(void)
{
    char slots[13 * POSTURE_SCHED_MAX_SLOTS + 1];
    format_slots(slots, sizeof(slots));
    if (!time_sync_is_done()) {
        // 未同步时本地时间不可信，不打 HH:MM
        ESP_LOGI(TAG, "[unsynced] sched: slots=[%s] time not synced -> always detect", slots);
        return;
    }
    time_t now = 0;
    struct tm ti = { 0 };
    time(&now);
    localtime_r(&now, &ti);
    // 与 calc_active() 语义一致：0 段=全天检测（calc_in_window 对 0 段返回 false，需单列）
    const char* state = s_slot_count == 0 ? "always ON"
                        : (calc_in_window(ti.tm_hour * 60 + ti.tm_min) ? "IN window -> detect"
                                                                       : "OUT of window -> pause");
    ESP_LOGI(TAG, "[%02d:%02d] sched: slots=[%s] %s", ti.tm_hour, ti.tm_min, slots, state);
}

bool posture_sched_is_active(void)
{
    bool active = calc_active();

    // 每帧取图前打印当前设置时段与窗口状态（本函数在 esp_camera_fb_get 之前调用；
    // 窗口外由推理任务 5s 轮询一次本函数，即 5s 一行）
    log_current_state();

    if (s_last_active < 0 || (int)active != s_last_active) {
        // 窗口切换边沿（或首次/重同步后）额外打印醒目事件行
        time_t now = 0;
        struct tm ti = { 0 };
        time(&now);
        localtime_r(&now, &ti);
        char cur[6];
        snprintf(cur, sizeof(cur), "%02d:%02d", ti.tm_hour, ti.tm_min);
        if (s_last_active < 0) {
            ESP_LOGI(TAG, "[%s] schedule state: %s", cur,
                     active ? "IN window, detection ON" : "OUT of window, detection OFF");
        }
        else if (active) {
            ESP_LOGI(TAG, "[%s] enter detection window, detection ON", cur);
        }
        else {
            ESP_LOGI(TAG, "[%s] leave detection window, detection OFF", cur);
        }
        s_last_active = (int)active;
    }
    return active;
}

bool posture_sched_set_slots(const posture_sched_slot_t* slots, int n)
{
    if (n < 0 || n > POSTURE_SCHED_MAX_SLOTS) return false;
    for (int i = 0; i < n; i++) {
        if (slots[i].start_min >= 1440 || slots[i].end_min >= 1440) return false;
        if (slots[i].start_min == slots[i].end_min) return false;  // 空窗口无意义
    }
    memset(s_slots, 0, sizeof(s_slots));
    memcpy(s_slots, slots, sizeof(posture_sched_slot_t) * n);
    s_slot_count = n;
    s_last_active = -1;
    save_to_nvs();
    ESP_LOGI(TAG, "Schedule set: %d slot(s), saved to NVS", n);
    return true;
}

bool posture_sched_parse_set(const char* args, char* err, int err_len)
{
#define SCHED_FAIL(fmt, ...)                             \
    do {                                                 \
        if (err != NULL) snprintf(err, err_len, fmt, ##__VA_ARGS__); \
        return false;                                    \
    } while (0)

    int n = 0;
    int pos = 0;
    if (err != NULL && err_len > 0) err[0] = '\0';
    if (sscanf(args, " %d%n", &n, &pos) != 1 || n < 0 || n > POSTURE_SCHED_MAX_SLOTS) {
        SCHED_FAIL("bad slot count (expect 0-%d)", POSTURE_SCHED_MAX_SLOTS);
    }

    posture_sched_slot_t tmp[POSTURE_SCHED_MAX_SLOTS];
    const char* p = args + pos;
    for (int i = 0; i < n; i++) {
        int h1, m1, h2, m2, adv = 0;
        if (sscanf(p, " %d:%d %d:%d%n", &h1, &m1, &h2, &m2, &adv) != 4) {
            SCHED_FAIL("bad time format at slot %d", i + 1);
        }
        if (h1 < 0 || h1 > 23 || m1 < 0 || m1 > 59 || h2 < 0 || h2 > 23 || m2 < 0 || m2 > 59) {
            SCHED_FAIL("time out of range at slot %d", i + 1);
        }
        tmp[i].start_min = h1 * 60 + m1;
        tmp[i].end_min = h2 * 60 + m2;
        if (tmp[i].start_min == tmp[i].end_min) {
            SCHED_FAIL("start==end at slot %d", i + 1);
        }
        p += adv;
    }
#undef SCHED_FAIL
    return posture_sched_set_slots(tmp, n);
}

int posture_sched_build_state(char* buf, int buf_len)
{
    int off = snprintf(buf, buf_len, "ESPCAM_SCHED_STATE %d %d %d",
                       time_sync_is_done() ? 1 : 0, calc_active() ? 1 : 0, s_slot_count);
    for (int i = 0; i < s_slot_count && off < buf_len - 13; i++) {
        off += snprintf(buf + off, buf_len - off, " %02d:%02d %02d:%02d",
                        s_slots[i].start_min / 60, s_slots[i].start_min % 60,
                        s_slots[i].end_min / 60, s_slots[i].end_min % 60);
    }
    return off;
}

void posture_sched_notify_time_synced(void)
{
    ESP_LOGI(TAG, "Time synced, schedule now in effect:");
    posture_sched_log_slots();
    s_last_active = -1;  // 重播当前状态
    // 注意：这里不能调 udp_sched_push_state()（sendto）——本函数运行在 SNTP 回调
    // 即 lwIP tcpip_thread 上下文，sendto 要等 tcpip_thread 处理，自等待 = 死锁，
    // 之后所有 socket 操作全部挂起。对时后的状态推送改由 20003 任务轮询
    // time_sync_pop_event() 触发（见 udp_discovery_task）。
}
