/**
 * app_version.h — 固件版本号（手动维护）
 * 格式：年月日_当日序号，如 20260916_1；发布新固件前手动 +1（同日递增，跨日重置）。
 * 用途：ESP 每条日志前缀 [版本号]；PC 接收器经 ESPCAM_ACK 显示版本。
 * 注意：版本号只影响显示；OTA 自动升级判断依据是镜像里的 elf_sha256（代码一变必变），
 *       忘记改版本号不会导致漏升级。
 */

#ifndef __APP_VERSION_H__
#define __APP_VERSION_H__

#define APP_VERSION "20260927_2"

#endif  // __APP_VERSION_H__
