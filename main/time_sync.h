/**
 * time_sync.h
 * SNTP 网络对时（STA 联网拿到 IP 后自动同步；时区固定 CST-8）
 */

#ifndef __TIME_SYNC_H__
#define __TIME_SYNC_H__

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化对时：注册 IP 事件，STA 拿到 IP 后自动启动 SNTP（含配网后重连）
 *        需在默认事件循环创建之后调用一次
 */
void time_sync_init(void);

/**
 * @brief 系统时间是否已同步（同步前 time()/localtime() 返回 1970 年）
 */
bool time_sync_is_done(void);

#ifdef __cplusplus
}
#endif

#endif  // __TIME_SYNC_H__
