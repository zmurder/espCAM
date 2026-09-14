/**
 * posture_sched.h
 * 坐姿检测时间段调度：每日重复的检测窗口（最多 5 段，支持跨午夜），
 * NVS 持久化断电保持；时间未同步（SNTP 未完成）时不判断窗口、全部检测。
 * 设置经 UDP 20003 下发（ESPCAM_SCHED_SET），状态回发 PC（ESPCAM_SCHED_STATE）
 */

#ifndef __POSTURE_SCHED_H__
#define __POSTURE_SCHED_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define POSTURE_SCHED_MAX_SLOTS 5

// 检测窗口：start_min/end_min 为一天内分钟数（0-1439）；start > end 表示跨午夜（如 22:00-06:00）
typedef struct
{
    uint16_t start_min;
    uint16_t end_min;
} posture_sched_slot_t;

/**
 * @brief 初始化：从 NVS 加载已保存的时间段（须在 nvs_flash_init 之后调用）
 */
void posture_sched_init(void);

/**
 * @brief 当前是否处于检测窗口内（== true 才推理）：
 *        时间未同步或未设置时段 → 恒 true（全检测）；窗口切换时打印边沿日志
 */
bool posture_sched_is_active(void);

/**
 * @brief 设置时间段并写入 NVS（n=0 清空=全天检测）；含合法性校验
 */
bool posture_sched_set_slots(const posture_sched_slot_t* slots, int n);

/**
 * @brief 解析 "n HH:MM HH:MM ..."（n 组共 2n 个时间）并设置；用于 UDP 文本协议。
 *        解析失败时把原因（英文短语）写入 err（可传 NULL 忽略），供回发 ESPCAM_SCHED_ERR
 */
bool posture_sched_parse_set(const char* args, char* err, int err_len);

/**
 * @brief 生成状态文本 "ESPCAM_SCHED_STATE synced active n HH:MM HH:MM ..."，返回长度
 */
int posture_sched_build_state(char* buf, int buf_len);

/**
 * @brief SNTP 同步完成通知（time_sync 回调调用）：播报调度生效 + 主动推送状态到 PC
 */
void posture_sched_notify_time_synced(void);

/**
 * @brief 打印当前全部时间段（串口日志）
 */
void posture_sched_log_slots(void);

#ifdef __cplusplus
}
#endif

#endif  // __POSTURE_SCHED_H__
