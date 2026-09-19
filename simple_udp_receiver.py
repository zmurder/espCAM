#!/usr/bin/env python3
"""
ESP32 坐姿检测 — 图像 + 检测结果 UDP 接收器
================================================
同时监听两个端口，把"图像 + 检测关键点"叠加后保存：
  20000 : 推理帧 JPEG 图像（分包，与 ESP send_image_via_udp 对应）
  20002 : 检测结果（0x02 + result + ratio + 6 关键点 x/y/conf）

每收到一帧完整图像，叠加最新的检测结果，保存到 received/latest.png。
配合 ESP32 app_main.c 的开关：SEND_IMAGE_VIA_UDP / SEND_RESULT_VIA_UDP。

用法：
  python simple_udp_receiver.py
  打开 received/latest.png 查看（每帧覆盖更新）；Ctrl+C 退出。
"""

import http.server
import io
import json
import math
import os
import re
import shutil
import socket
import sqlite3
import struct
import sys
import threading
import time
from datetime import datetime, timedelta

from PIL import Image, ImageDraw

UDP_IMG_PORT = 20000  # 图像
UDP_RESULT_PORT = 20002  # 检测结果

W, H = 320, 240  # 模型输入尺寸，关键点归一化坐标基准
CONF_THRESH = 0.4  # 与 ESP 一致：眼/耳置信度阈值
SHOULDER_CONF_THRESH = 0.3  # 与 ESP 一致：双肩单独阈值（更低，趴近时肩仍可用）
# 6 关键点顺序与 ESP KEYPOINT_* 枚举一致：0=L_eye 1=R_eye 2=L_ear 3=R_ear 4=L_sh 5=R_sh
KP_NAMES = ["L_eye", "R_eye", "L_ear", "R_ear", "L_sh", "R_sh"]
KP_COLORS = ["red", "blue", "magenta", "purple", "orange", "cyan"]
RESULT_TAG = {0: "OK", 1: "BAD_NECK", 2: "BAD_SHOULDER", 3: "NOT_DET", 4: "UNRELIABLE", 5: "TOO_FAR"}

# 绘制开关：True=叠加关键点（连线+圆圈+标签）+ 左上角判断文字 + 右下角时间戳；False=只存干净原图
DRAW_KEYPOINTS = False

# 保存频率：每收到 N 帧图像只保存 1 帧（1=每帧都存）。跳过的帧不解码不落盘，
SAVE_EVERY_N = 10

# 以上两项可运行中修改（img 命令 / Web 图像保存卡），支持"全部设备默认 + 按设备覆盖"
# （与检测时段记忆同一套模型），持久化到 recv_settings.json；无该文件时用上面的默认值
IMG_SETTINGS_FILE = "recv_settings.json"
# people=仅保存有人帧：设备最近结果连续 IMG_NOBODY_SKIP_N 帧无效（没人/太远/不可信）→ 停止保存
IMG_NOBODY_SKIP_N = 50
_img_cfg = {"default": {"draw_kp": DRAW_KEYPOINTS, "save_n": SAVE_EVERY_N, "people": False}, "devices": {}}


def _load_img_cfg():
    try:
        with open(IMG_SETTINGS_FILE, encoding="utf-8") as f:
            m = json.load(f)
        d = m.get("default")
        if isinstance(d, dict):
            if isinstance(d.get("draw_kp"), bool):
                _img_cfg["default"]["draw_kp"] = d["draw_kp"]
            if isinstance(d.get("save_n"), int) and d["save_n"] >= 0:
                _img_cfg["default"]["save_n"] = d["save_n"]
            if isinstance(d.get("people"), bool):
                _img_cfg["default"]["people"] = d["people"]
        if isinstance(m.get("devices"), dict):
            _img_cfg["devices"] = m["devices"]
    except (OSError, ValueError):
        pass


def _save_img_cfg():
    try:
        with open(IMG_SETTINGS_FILE, "w", encoding="utf-8") as f:
            json.dump(_img_cfg, f)
    except OSError as e:
        print(f"[img] 设置保存失败: {e}")


def _img_cfg_for(devid):
    """该设备生效的图像保存设置：默认 + 专属覆盖（未覆盖字段回退默认）"""
    merged = dict(_img_cfg["default"])
    merged.update(_img_cfg["devices"].get(devid) or {})
    return merged


_load_img_cfg()

# 与 ESP judge_posture 一致的阈值（4 条任意成立即不良；左上角显示参考用）
EYE_FORWARD_RATIO_MIN = 1.5  # 条件1：眼肩垂直距离/双眼距离 < 此值 → 前倾（需据正常坐姿标定）
EAR_FORWARD_RATIO_MIN = 1.5  # 条件2：耳肩垂直距离/双耳距离 < 此值 → 前倾
EYE_HEAD_TILT_WARN = 35.0  # 条件3：双眼-双肩相对倾斜角 > 此值 → 歪头
EAR_HEAD_TILT_WARN = 35.0  # 条件4：双耳-双肩相对倾斜角 > 此值 → 歪头
# 几何合理性预检（与 ESP 一致）：同组连线近垂直属明显误检；双肩超阈或眼+耳全超阈 → 本帧不判断
SHOULDER_LINE_TILT_MAX = 40.0  # 双肩连线 |倾斜角| 上限（度）
EYE_LINE_TILT_MAX = 40.0  # 双眼连线 |倾斜角| 上限（度）
EAR_LINE_TILT_MAX = 40.0  # 双耳连线 |倾斜角| 上限（度）
SHOULDER_DIST_MIN = 0.19  # 与 ESP 一致：肩距(归一化) < 此值 → 人太远（定位噪声占比大），本帧不判断

# 自动发现（与 ESP udp_discovery_task 对应）：PC 周期广播 → ESP32 学习本机 IP 为发送目标并回 ACK
# 之后 20000/20002 的数据会自动发到本机，UDP_SERVER_IP 无需再改
DISCOVERY_PORT = 20003
DISCOVERY_REQ = b"ESPCAM_DISCOVER"  # ESP 应答 "ESPCAM_ACK <版本号>"（版本变化时打印）

# 坐姿检测时间段设置（每日重复，最多 5 段，支持跨午夜如 "22:00-06:30"）：
# None = 不下发（保留 ESP 当前设置）；[] = 清空（全天检测）；非空 = 设置并写入 ESP NVS（断电保持）
# ESP 未同步网络时间时不判断时段、全部检测（状态见 [sched] 打印）
# simple_udp_receiver.py 顶部
# POSTURE_SCHED_SLOTS = ["09:00-11:30", "14:00-18:00"]   # 设两个时段
# POSTURE_SCHED_SLOTS = ["22:00-06:30"]                   # 跨午夜
# POSTURE_SCHED_SLOTS = []                                # 清空 = 全天检测
# POSTURE_SCHED_SLOTS = None                              # 不动 ESP 现有设置

POSTURE_SCHED_SLOTS = ["09:00-11:30", "14:00-21:00"]  # 例: ["09:00-11:30", "14:00-18:00"]
SCHED_GET = b"ESPCAM_SCHED_GET"

# 检测时段的按设备记忆（sched_per_dev.json）：default=全部设备统一设置（None=不下发），
# devices=各设备专属设置（覆盖 default）。手动"全部设备"设置会统一并清掉专属；
# 启动时 default 广播一次 + 对有专属设置的在线设备单播覆盖，设备重新上线（ACK）自动补发。
# 没有记忆文件时 default 取 POSTURE_SCHED_SLOTS（兼容旧语义）
SCHED_MEM_FILE = "sched_per_dev.json"


def _load_sched_mem():
    def _norm(x):  # JSON 读回的 [[s,e],...] → [(s,e),...]（与内存新建一致）
        return [tuple(p) for p in x] if isinstance(x, list) else x

    try:
        with open(SCHED_MEM_FILE, encoding="utf-8") as f:
            m = json.load(f)
        if isinstance(m, dict) and isinstance(m.get("devices"), dict):
            return {"default": _norm(m.get("default")), "devices": {k: _norm(v) for k, v in m["devices"].items()},
                    "alert_default": m.get("alert_default", True),       # 不良提醒模式默认（True=连续播）
                    "alert_devices": {k: bool(v) for k, v in m.get("alert_devices", {}).items()}}  # 各设备专属
    except (OSError, ValueError):
        pass
    # 无记忆文件：POSTURE_SCHED_SLOTS 常量作 default（解析成分钟元组，旧语义不变）
    try:
        dflt = _parse_slots_arg(",".join(POSTURE_SCHED_SLOTS)) if POSTURE_SCHED_SLOTS is not None else None
    except ValueError as e:
        print(f"[sched] POSTURE_SCHED_SLOTS 配置错误: {e}")
        dflt = None
    return {"default": dflt, "devices": {}, "alert_default": True, "alert_devices": {}}


# 坐姿历史库（SQLite）：result_receiver 逐帧写入 ts/result/ratio，
# 供 posture_report.py 查询统计与绘制趋势图（kps 不入库，体积小且趋势分析用不到）
HISTORY_DB = "posture_history.db"

# OTA 固件升级：独立目录 ota/（HTTP 服务只共享它，不会暴露整个 build/）。
# 升级判断依据：镜像偏移 0x20+0x90 处的 app_elf_sha256 前 8 位（构建系统回填，代码一变必变），
# ESP 经 ACK 上报自己的 sha8，与 ota/espCAM_WIFI.bin 对比，不一致 → 自动触发升级
# （自动检测可用命令 ota on/off 开关，连续 3 次未成功自动停用并提示）
OTA_DIR = "ota"
OTA_BIN = os.path.join(OTA_DIR, "espCAM_WIFI.bin")
OTA_SRC = os.path.join("build", "espCAM_WIFI.bin")  # idf.py build 的产物（ota 命令自动拷入 OTA_DIR）
OTA_HTTP_PORT = 20004  # 与 20000 段统一（本机 HTTP 服务端口，ESP 从这里下载固件）


def _norm_tilt(deg):
    """把 atan2 角度归一化到 [-90,90]，使水平(0°或±180°)都映射为 0°"""
    if deg > 90:
        return deg - 180
    if deg < -90:
        return deg + 180
    return deg


def _broadcast_targets():
    """广播目标集合：受限广播 + 默认出口网段的 /24 定向广播
    （Windows 多网卡时 255.255.255.255 可能从虚拟网卡出去，定向广播更可靠）"""
    targets = {("255.255.255.255", DISCOVERY_PORT)}
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))  # 不实际发包，仅探测默认出口 IP
        ip = s.getsockname()[0]
        s.close()
        if not ip.startswith("127."):
            targets.add((ip.rsplit(".", 1)[0] + ".255", DISCOVERY_PORT))
    except OSError:
        pass
    return targets


def _parse_slots_arg(arg):
    """'09:00-11:30,14:00-18:00' → [(start_min,end_min),...]；'clear' → []（清空=全天检测）。
    本地预校验（与 ESP 一致）：HH:MM-HH:MM 格式、时间范围、起止不同、最多 5 段；失败抛 ValueError"""
    if arg.strip().lower() in ("clear", "c"):
        return []
    slots = []
    for part in arg.split(","):
        m = re.fullmatch(r"\s*(\d{1,2}):(\d{2})\s*-\s*(\d{1,2}):(\d{2})\s*", part)
        if not m:
            raise ValueError(f"时段格式错误: '{part.strip()}'（应为 HH:MM-HH:MM）")
        h1, m1, h2, m2 = map(int, m.groups())
        if h1 > 23 or h2 > 23 or m1 > 59 or m2 > 59:
            raise ValueError(f"时间超出范围: '{part.strip()}'（HH 0-23，MM 0-59）")
        s, e = h1 * 60 + m1, h2 * 60 + m2
        if s == e:
            raise ValueError(f"起止时间相同（空时段）: '{part.strip()}'")
        slots.append((s, e))
    if len(slots) > 5:
        raise ValueError(f"最多 5 个时段，当前 {len(slots)} 个")
    return slots


def _slots_to_set_msg(slots):
    """[(start_min,end_min),...] → 'ESPCAM_SCHED_SET n HH:MM HH:MM ...'"""
    times = []
    for s, e in slots:
        times += [f"{s // 60:02d}:{s % 60:02d}", f"{e // 60:02d}:{e % 60:02d}"]
    return f"ESPCAM_SCHED_SET {len(slots)} " + " ".join(times)


_sched_mem = _load_sched_mem()  # 放在 _parse_slots_arg 之后（无记忆文件时用它解析常量）
_sched_synced = set()  # 本轮进程内已补发专属时段的设备（重启 PC 才会再补发）
_alert_synced = set()  # 本轮进程内已补发专属提醒模式的设备（同上）


_disc_sock = None  # discovery socket（broadcaster 创建后共享给主线程命令下发）
# 多设备表：devid（ACK 上报的 WiFi MAC 后 3 字节 hex；旧固件无 ID 时以 IP 兜底）
#   → {ip, ver, sha8, fails, last_seen, last_try}；发现周期 2s 刷新，30s 无应答视为离线
_devices = {}
_devices_order = []  # ota list 打印时的序号顺序（ota <序号> 引用）
_ota_httpd = None  # 本地 OTA HTTP 服务（懒启动一次）
_ota_state = {"watch": True}  # 自动检测总开关（fails 按设备记在 _devices 里）
# OTA 批次（逐台串行：一台确认/失败/超时后再推下一台，避免多台同时下载挤占 WiFi 带宽）
_ota_batch = {"queue": [], "current": None, "before_sha": None, "deadline": 0.0,
              "phase": None, "progress": None}  # phase: push(已触发)→download(下载中)→verify(等重启确认)；progress: {cur,tot,t}
OTA_STEP_TIMEOUT = 300  # 单台：触发 → 下载 → DONE 的上限（秒）
OTA_VERIFY_TIMEOUT = 90  # 单台：DONE 重启 → ACK 上报新 sha 的上限（秒）


def _bin_sha8(path):
    """读固件镜像的 app_elf_sha256 前 8 位 hex（偏移 0x20=appdesc 段 + 0x90=sha256 字段）；失败返回 None"""
    try:
        with open(path, "rb") as f:
            f.seek(0x20 + 0x90)
            raw = f.read(32)
        return raw[:4].hex() if len(raw) == 32 else None
    except OSError:
        return None


def _local_ip():
    """本机出口 IP（探测默认路由，不实际发包；失败返回 None）"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        ip = s.getsockname()[0]
        s.close()
        return None if ip.startswith("127.") else ip
    except OSError:
        return None


class _OtaFileHandler(http.server.SimpleHTTPRequestHandler):
    """OTA 文件服务：只共享 ota/ 目录"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=OTA_DIR, **kwargs)


class _QuietHTTPServer(http.server.ThreadingHTTPServer):
    """ESP 下载完成重启/中途断开时 TCP 连接被重置，属正常现象——
    覆盖服务器对象的 handle_error（异常处理在这里，不在 handler 类上），
    静默为一行提示，不打整段 traceback"""

    def handle_error(self, request, client_address):
        print(f"[ota] HTTP 连接中断（{client_address[0]}，ESP 重启/下载中断时正常）")


def _start_ota_server():
    """懒启动本地 HTTP 服务共享 ota 目录（ESP 从这里下载固件）"""
    global _ota_httpd
    if _ota_httpd is not None:
        return True
    if not os.path.exists(OTA_BIN):
        print(f"[ota] 未找到固件 {OTA_BIN}（先 idf.py build 后输入 ota 拷入，或手动复制）")
        return False
    os.makedirs(OTA_DIR, exist_ok=True)
    _ota_httpd = _QuietHTTPServer(("0.0.0.0", OTA_HTTP_PORT), _OtaFileHandler)
    threading.Thread(target=_ota_httpd.serve_forever, daemon=True).start()
    print(f"[ota] HTTP 服务已启动 :{OTA_HTTP_PORT}（共享 {OTA_DIR}/，供 ESP 下载固件）")
    return True


def _fw_version():
    """读 main/app_version.h 的 APP_VERSION（build 产物对应的源码版本，升级提示显示用；
    注意：若改了 .h 未重新 build，此值会超前于 ota/ 实际固件——升级判断仍以 sha 为准）"""
    try:
        with open(os.path.join("main", "app_version.h"), encoding="utf-8") as f:
            m = re.search(r'#define\s+APP_VERSION\s+"([^"]+)"', f.read())
        return m.group(1) if m else "?"
    except OSError:
        return "?"


def _trigger_one(devid, reason=""):
    """向指定设备单播 ESPCAM_OTA_START（多设备下绝不广播：广播会同时打到所有设备），
    URL 带本机当前 IP（动态），ESP 回 OTA_STARTED 后开始下载"""
    d = _devices.get(devid)
    if not d or not d.get("ip"):
        print(f"[ota] 设备 {devid} 不在线，跳过")
        return False
    if not _start_ota_server():
        return False
    ip = _local_ip()
    if not ip:
        print("[ota] 无法确定本机 IP（网络未连接？）")
        return False
    msg = f"ESPCAM_OTA_START http://{ip}:{OTA_HTTP_PORT}/espCAM_WIFI.bin"
    try:
        _disc_sock.sendto(msg.encode(), (d["ip"], DISCOVERY_PORT))
    except OSError:
        print(f"[ota] 发送给 {devid}（{d['ip']}）失败")
        return False
    print(f"[ota] 已触发 {devid}（{d['ip']}）升级 → {_fw_version()}: {msg}")
    return True


def _ota_advance():
    """推进 OTA 批次：空闲且有排队 → 触发下一台；当前台超时 → 放弃转下一台。
    逐台串行（一台确认/失败/超时再推下一台），避免多台同时下载挤占 WiFi 带宽"""
    b = _ota_batch
    if b["current"]:
        if time.time() > b["deadline"]:
            print(f"[ota] 设备 {b['current']} 升级超时，跳过（继续下一台）")
            b["current"] = None
            b["phase"] = None
            b["progress"] = None
        else:
            return
    if not b["queue"]:
        return
    devid = b["queue"].pop(0)
    d = _devices.get(devid)
    if not d or not d.get("ip") or not d.get("sha8"):
        return  # 已离线，下一轮继续（该台被跳过）
    b["current"] = devid
    b["before_sha"] = d["sha8"]
    b["deadline"] = time.time() + OTA_STEP_TIMEOUT
    b["phase"] = "push"
    b["progress"] = None
    if not _trigger_one(devid):
        b["current"] = None
        b["phase"] = None


def _devid_by_ip(ip):
    """按源 IP 反查设备 ID（OTA_STARTED/PROGRESS/DONE/ERR 回包无设备标识，以其源 IP 归属）"""
    for k, d in _devices.items():
        if d.get("ip") == ip:
            return k
    return ip


def _ota_load():
    """把 build/espCAM_WIFI.bin 拷入 ota/（内容相同则跳过）；返回是否就绪"""
    src_sha = _bin_sha8(OTA_SRC)
    if not src_sha:
        print(f"[ota] 未找到固件产物 {OTA_SRC}（先 idf.py build）")
        return False
    if src_sha == _bin_sha8(OTA_BIN):
        return True  # ota/ 已是最新
    os.makedirs(OTA_DIR, exist_ok=True)
    shutil.copy2(OTA_SRC, OTA_BIN)
    print(f"[ota] 已拷入最新固件 → {OTA_BIN}")
    return True


def _auto_ota_check(devid):
    """对该设备的自动检测（每次收到它的 ACK 调用）：其 sha 与 ota/ 固件不一致 → 入队升级。
    批次进行中让路（下轮 ACK 再查）；60s 冷却；触发预记失败，成功（ACK 上报新 sha）清零；
    连续 3 次未成功暂停该设备的自动升级（ota on 重开）"""
    if not _ota_state["watch"]:
        return
    b = _ota_batch
    if b["current"] or b["queue"]:
        return
    d = _devices.get(devid)
    if not d or d.get("fails", 0) >= 3 or not d.get("sha8"):
        return
    want = _bin_sha8(OTA_BIN)
    if not want or want == d["sha8"]:
        return
    if time.time() - d.get("last_try", 0.0) < 60:
        return
    d["last_try"] = time.time()
    d["fails"] += 1
    print(f"[ota] 自动升级 {devid} → {_fw_version()}（sha {want} ≠ 当前 {d['sha8']}）")
    b["queue"] = [devid]
    if d["fails"] >= 3:
        print(f"[ota] {devid} 连续 3 次未确认成功，暂停其自动升级（ota on 重开 / 手动 ota 重试）")


def _print_devices():
    """打印设备表（ota <序号> 引用这里生成的序号）"""
    global _devices_order
    now = time.time()
    online = [(k, d) for k, d in _devices.items() if now - d.get("last_seen", 0) < 30]
    _devices_order = [k for k, _ in online]
    if not online:
        print("[devices] 暂无设备应答（2s 发现周期，稍候）")
        return
    want = _bin_sha8(OTA_BIN)
    print(f"[devices] {len(online)} 台在线：")
    for i, (k, d) in enumerate(online, 1):
        up = "？" if not (want and d.get("sha8")) else ("待升级" if d["sha8"] != want else "最新")
        print(f"  {i}. {k}  {d['ip']}  固件 {d.get('ver') or '?'}  {up}")


def _resolve_device(arg):
    """'2'（ota list 的序号）或设备 ID 前缀 → devid；找不到返回 None 并提示"""
    if arg.isdigit() and 1 <= int(arg) <= len(_devices_order):
        return _devices_order[int(arg) - 1]
    hits = [k for k in _devices if k.startswith(arg)]
    if len(hits) == 1:
        return hits[0]
    print(f"[ota] 找不到设备 '{arg}'（先 ota list 看序号/ID）")
    return None


def _ota_cmd(arg=""):
    """OTA 命令分发（input 交互与将来的 Web UI 共用此入口）：
    ''|all = 全部待升级设备逐台串行；list = 设备表；<序号|ID前缀> = 指定设备；on|off = 自动检测开关"""
    if _disc_sock is None:
        print("[ota] 发现通道未就绪，稍后再试")
        return
    if arg in ("list", "l", "devices", "dev"):
        _print_devices()
        return
    if arg == "on":
        _ota_state["watch"] = True
        for d in _devices.values():
            d["fails"] = 0
        print("[ota] 自动检测已开启（各设备固件与 ota/ 不一致时自动升级）")
        return
    if arg == "off":
        _ota_state["watch"] = False
        print("[ota] 自动检测已关闭（仍可手动 ota 升级）")
        return
    if not _ota_load():
        return
    want = _bin_sha8(OTA_BIN)
    now = time.time()
    if arg in ("", "all", "a"):
        targets = [k for k, d in _devices.items() if d.get("sha8") and d["sha8"] != want and now - d.get("last_seen", 0) < 30]
        if not targets:
            known = sum(1 for d in _devices.values() if d.get("sha8"))
            print(f"[ota] 无待升级设备（已应答的 {known} 台均在运行最新固件）")
            return
        _ota_batch["queue"] = targets
        print(f"[ota] 批量升级 {len(targets)} 台（逐台串行，各自完成后自动继续）")
        _ota_advance()
        return
    devid = _resolve_device(arg)
    if not devid:
        return
    if want and _devices[devid].get("sha8") == want:
        print(f"[ota] {devid} 已运行最新固件，无需升级")
        return
    _ota_batch["queue"] = [devid]
    _ota_advance()


def _send_sched_set(slots, devid=None):
    """经 20003 通道下发 ESPCAM_SCHED_SET（ESP 回 SCHED_STATE 确认 / SCHED_ERR 报错）。
    devid=None 广播全部设备；否则单播到该设备当前 IP（只改它的 NVS，其余设备不受影响）"""
    if _disc_sock is None:
        print("[sched] 发现通道未就绪，稍后再试")
        return
    msg = _slots_to_set_msg(slots).encode()
    targets = _broadcast_targets() if devid is None else None
    if devid is not None:
        d = _devices.get(devid)
        if not d or not d.get("ip"):
            print(f"[sched] 设备 {devid} 不在线，无法单播（先 ota list 查看）")
            return
        targets = [(d["ip"], DISCOVERY_PORT)]
    for t in targets:
        try:
            _disc_sock.sendto(msg, t)
        except OSError:
            pass
    desc = ", ".join(f"{s // 60:02d}:{s % 60:02d}-{e // 60:02d}:{e % 60:02d}" for s, e in slots)
    who = "全部设备" if devid is None else devid
    print(f"[sched] 已下发设置({who}): {desc or '全天检测(清空)'}，等待 ESP 确认...")


def _save_sched_mem():
    try:
        with open(SCHED_MEM_FILE, "w", encoding="utf-8") as f:
            json.dump(_sched_mem, f)
    except OSError as e:
        print(f"[sched] 记忆文件写入失败（本次设置仍已下发）: {e}")


def _sched_set_memory(devid, slots):
    """把一次设置记入 sched_per_dev.json（devid=None 表示全部设备：更新 default 并清掉各设备
    专属——统一后专属失效；指定设备只写自己的条目）。下次启动 default 广播后按条目单播覆盖"""
    if devid is None:
        _sched_mem["default"] = slots
        _sched_mem["devices"].clear()
        _sched_synced.clear()
    else:
        _sched_mem["devices"][devid] = slots
        _sched_synced.discard(devid)  # 让 ACK 补发路径重新同步一次
    _save_sched_mem()


_ALERT_DESC = {True: "连续播", False: "只播一次"}


def _send_alert_set(val, devid=None):
    """下发 ESPCAM_ALERT_SET（devid=None 广播全部；否则单播该设备），ESP 写 NVS 后回 ALERT_STATE 确认"""
    if _disc_sock is None:
        print("[alert] 发现通道未就绪，稍后再试")
        return
    msg = f"ESPCAM_ALERT_SET {1 if val else 0}".encode()
    targets = _broadcast_targets() if devid is None else None
    if devid is not None:
        d = _devices.get(devid)
        if not d or not d.get("ip"):
            print(f"[alert] 设备 {devid} 不在线，无法单播（先 ota list 查看）")
            return
        targets = [(d["ip"], DISCOVERY_PORT)]
    for t in targets:
        try:
            _disc_sock.sendto(msg, t)
        except OSError:
            pass
    who = "全部设备" if devid is None else devid
    print(f"[alert] 已下发({_ALERT_DESC[val]})({who})，等待 ESP 确认...")


def _alert_set_memory(devid, val):
    """提醒模式记忆（与时段同一份 sched_per_dev.json）：全部=统一默认并清专属；单台=只写自己"""
    if devid is None:
        _sched_mem["alert_default"] = val
        _sched_mem["alert_devices"].clear()
        _alert_synced.clear()
    else:
        _sched_mem["alert_devices"][devid] = val
        _alert_synced.discard(devid)
    _save_sched_mem()


def _alert_cmd(arg=""):
    """不良提醒模式命令（键盘与 Web 共用）：'' = 概览；repeat=连续播 / once=只播一次；
    @<序号|ID> repeat|once = 只改该设备。设置写 ESP NVS（断电保持）并记入 sched_per_dev.json"""
    usage = "[alert] 用法: alert（查看） | alert repeat|once（全部设备） | alert @<序号|ID> repeat|once（指定设备）"
    if not arg:
        print(f"[alert] 默认(全部设备): {_ALERT_DESC[_sched_mem['alert_default']]}（repeat=连续播, once=只播一次）")
        for k, v in _sched_mem["alert_devices"].items():
            print(f"[alert]   {k}（专属）: {_ALERT_DESC[v]}")
        for k, d in _devices.items():
            if d.get("ip") and d.get("alert") is not None:
                print(f"[alert]   {k} 在线生效: {_ALERT_DESC[d['alert']]}")
        return
    devid = None
    if arg.startswith("@"):
        head, _, rest = arg.partition(" ")
        devid = _resolve_device(head[1:].lower())
        if not devid:
            return
        arg = rest.strip()
    if arg not in ("repeat", "once"):
        print(usage)
        return
    val = arg == "repeat"
    _alert_set_memory(devid, val)
    _send_alert_set(val, devid)


_sched_last = {}  # devid → {"text","t"}：上次 [sched] 打印（内容+时刻）：相同内容 60s 一条心跳，不每 2s 刷屏


def _handle_sched_state(data, devid):
    """解析 'ESPCAM_SCHED_STATE synced active n HH:MM HH:MM ...'；状态变化立即打印，
    不变则每 60s 打一条心跳（按设备去重，多设备互不干扰）；同时把时段/时间同步状态
    存入设备表（Web 设备列表展示），行尾附该设备固件版本"""
    try:
        parts = data.decode(errors="ignore").split()
        synced, active, n = int(parts[1]), int(parts[2]), int(parts[3])
        slots = []
        for i in range(n):
            slots.append(f"{parts[4 + 2 * i]}-{parts[5 + 2 * i]}")
    except (IndexError, ValueError):
        return
    desc = ", ".join(slots) if slots else "全天"
    sync_desc = "时间已同步" if synced else "时间未同步(暂全检测)"
    act_desc = "检测开启" if active else "检测暂停(时段外)"
    d = _devices.get(devid)
    if d is not None:
        d["sched_desc"] = desc
        d["sched_synced"] = bool(synced)
        d["sched_active"] = bool(active)
        d["sched_seen"] = time.time()
    ver_suffix = f"| ESP 固件 {d['ver']}" if d and d.get("ver") else ""
    ui_line = f"检测时段: {desc} | ESP {sync_desc} | 当前{act_desc}"
    line = f"[{devid}] {ui_line}{ver_suffix}"
    now = time.time()
    last = _sched_last.get(devid)
    if last and last["text"] == line and now - last["t"] < 60:
        return
    _sched_last[devid] = {"text": line, "ui": ui_line, "t": now}
    print(f"[sched] {datetime.now().strftime('%H:%M:%S')} {line}")


def discovery_broadcaster():
    """20003 通道：①周期广播发现包（ESP 学习本机 IP）②启动时下发检测时间段设置
    ③每周期查询调度状态并显示（含 ESP 时间同步状态）"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(("0.0.0.0", DISCOVERY_PORT))  # 固定源端口，可收 ESP 主动推送（SNTP 同步后）
    sock.settimeout(0.5)
    global _disc_sock
    _disc_sock = sock  # 暴露给主线程：交互命令 s 借此 socket 下发
    print(f"[discovery] 广播发现包 → :{DISCOVERY_PORT}（ESP32 将自动学习本机 IP）")
    # 启动下发：default（全部设备统一，None=不下发）广播一次；有专属记忆的设备随后在 ACK 时单播覆盖
    sched_set = None
    if _sched_mem["default"] is not None:
        try:
            sched_set = _slots_to_set_msg(_sched_mem["default"])
        except (ValueError, TypeError) as e:
            print(f"[sched] 记忆的默认时段配置错误，未下发: {e}")
    if sched_set:
        desc = ", ".join(f"{s // 60:02d}:{s % 60:02d}-{e // 60:02d}:{e % 60:02d}" for s, e in _sched_mem["default"])
        print(f"[sched] 下发默认时段(全部设备): {desc or '全天检测(清空)'}（将写入 ESP NVS）")
        if _sched_mem["devices"]:
            print(f"[sched] 另有 {len(_sched_mem['devices'])} 台专属设置，将在其应答后单播覆盖")
    # 启动下发：不良提醒模式默认值（同通道广播一次；有专属记忆的设备 ACK 后单播覆盖）
    alert_set = f"ESPCAM_ALERT_SET {1 if _sched_mem['alert_default'] else 0}".encode()
    print(f"[alert] 下发默认提醒模式(全部设备): {_ALERT_DESC[_sched_mem['alert_default']]}（将写入 ESP NVS）")
    if _sched_mem["alert_devices"]:
        print(f"[alert] 另有 {len(_sched_mem['alert_devices'])} 台专属设置，将在其应答后单播覆盖")
    first_round = True
    while True:
        for t in _broadcast_targets():
            try:
                if first_round and sched_set:
                    sock.sendto(sched_set.encode(), t)  # 启动时下发设置（None=不下发）
                if first_round:
                    sock.sendto(alert_set, t)  # 启动时下发提醒模式默认值
                sock.sendto(SCHED_GET, t)  # 每周期查询调度状态
                sock.sendto(DISCOVERY_REQ, t)  # 保活：IP 变化 2s 内自动跟随
            except OSError:
                pass
        first_round = False
        while True:  # 排空本轮应答（ACK + SCHED_STATE + OTA 回执）
            try:
                data, addr = sock.recvfrom(256)
            except socket.timeout:
                break
            if data.startswith(b"ESPCAM_ACK"):
                parts = data.decode(errors="ignore").split()
                # 新固件: ESPCAM_ACK <devid> <ver> <sha8>；旧固件（升级过渡期）无 devid，以源 IP 兜底
                if len(parts) >= 4:
                    devid, ver, sha8 = parts[1], parts[2], parts[3]
                else:
                    devid = addr[0]
                    ver = parts[1] if len(parts) > 1 else ""
                    sha8 = parts[2] if len(parts) > 2 else ""
                d = _devices.get(devid)
                if d is None:
                    d = {"ip": None, "ver": None, "sha8": None, "fails": 0, "last_seen": 0.0, "last_try": 0.0}
                    _devices[devid] = d
                    print(f"[discovery] 发现设备 {devid}（{addr[0]}），图像/结果将发往本机")
                    # 旧固件条目以 IP 为键，升级后带真 devid 上报 → 合并失败计数、移除旧条目
                    # （仅当 devid ≠ IP 才清理——IP 兜底时两者相同，会把刚插入的自己删掉）
                    if devid != addr[0]:
                        legacy = _devices.pop(addr[0], None)
                        if legacy and legacy.get("fails"):
                            d["fails"] = legacy["fails"]
                old_sha = d["sha8"]
                d["ip"] = addr[0]
                d["last_seen"] = time.time()
                if sha8 and sha8 != old_sha:
                    if old_sha is not None and d["fails"] > 0:
                        print(f"[ota] {devid} 升级成功确认：已运行新固件")
                    d["fails"] = 0  # ACK 上报新 sha = 升级成功
                    d["sha8"] = sha8
                if ver and ver != d["ver"]:
                    d["ver"] = ver
                    print(f"[version] {devid} 固件版本: {ver}")
                # 第 5 段 = 不良提醒模式（旧固件无此段 → 保持 None，Web 显示"—"）
                if len(parts) >= 5 and parts[4] in ("0", "1"):
                    d["alert"] = parts[4] == "1"
                # 批次确认：当前台 ACK 上报新 sha = 已重启进新固件，继续下一台
                b = _ota_batch
                if b["current"] == devid and b["before_sha"] and sha8 and sha8 != b["before_sha"]:
                    print(f"[ota] {devid} 批次完成，继续下一台")
                    b["current"] = None
                    b["phase"] = None
                    b["progress"] = None
                    _ota_advance()
                _auto_ota_check(devid)
                # 该设备有专属时段记忆且本进程内未同步 → 单播覆盖默认广播（重启/重新上线自动恢复）
                if devid in _sched_mem["devices"] and devid not in _sched_synced:
                    _sched_synced.add(devid)
                    _send_sched_set(_sched_mem["devices"][devid], devid)
                # 提醒模式专属记忆同理
                if devid in _sched_mem["alert_devices"] and devid not in _alert_synced:
                    _alert_synced.add(devid)
                    _send_alert_set(_sched_mem["alert_devices"][devid], devid)
            elif data.startswith(b"ESPCAM_SCHED_STATE"):
                _handle_sched_state(data, _devid_by_ip(addr[0]))
            elif data.startswith(b"ESPCAM_ALERT_STATE"):
                # ESPCAM_ALERT_STATE 0|1：ALERT_SET 的回执 / ALERT_GET 的应答（确认实际生效模式）
                devid = _devid_by_ip(addr[0])
                try:
                    val = int(data.split()[1]) == 1
                except (IndexError, ValueError):
                    val = None
                d = _devices.get(devid)
                if d is not None and val is not None:
                    if d.get("alert") != val:
                        print(f"[alert] {devid} 提醒模式已生效: {_ALERT_DESC[val]}")
                    d["alert"] = val
            elif data.startswith(b"ESPCAM_ALERT_ERR"):
                print(f"[alert] ESP 设置失败: {data.decode(errors='ignore')[17:].strip()}")
            elif data.startswith(b"ESPCAM_SCHED_ERR"):
                print(f"[sched] ESP 设置失败: {data.decode(errors='ignore')[16:].strip()}")
            elif data == b"ESPCAM_OTA_STARTED":
                devid = _devid_by_ip(addr[0])
                print(f"[ota] {devid} 已开始下载固件（{datetime.now().strftime('%H:%M:%S')}）")
                if _ota_batch["current"] == devid:
                    _ota_batch["phase"] = "download"
                    _ota_batch["progress"] = None
            elif data.startswith(b"ESPCAM_OTA_PROGRESS"):
                # ESP 每 256KB 推送一次：ESPCAM_OTA_PROGRESS <已收字节> <总字节>
                try:
                    cur, tot = map(int, data.split()[1:3])
                    devid = _devid_by_ip(addr[0])
                    print(f"[ota] {devid} 下载进度: {cur // 1024}/{tot // 1024} KB（{cur * 100 // tot}%）")
                    if _ota_batch["current"] == devid:
                        _ota_batch["phase"] = "download"
                        _ota_batch["progress"] = {"cur": cur, "tot": tot, "t": time.time()}
                except (IndexError, ValueError):
                    pass
            elif data == b"ESPCAM_OTA_DONE":
                devid = _devid_by_ip(addr[0])
                print(f"[ota] {devid} 下载校验完成，即将重启（约 15s 后 ACK 带新版本）")
                if _ota_batch["current"] == devid:
                    _ota_batch["phase"] = "verify"
                    _ota_batch["progress"] = None
                    _ota_batch["deadline"] = time.time() + OTA_VERIFY_TIMEOUT  # 转入重启确认阶段
            elif data.startswith(b"ESPCAM_OTA_ERR"):
                emsg = data.decode(errors="ignore")[15:].strip()
                devid = _devid_by_ip(addr[0])
                if emsg == "busy":
                    print(f"[ota] {devid} 反馈升级已在进行，忽略本次重复触发")  # 重复触发的正常防重入
                else:
                    print(f"[ota] {devid} 升级失败: {emsg}（当前固件继续运行）")
                    if _ota_batch["current"] == devid:
                        _ota_batch["current"] = None
                        _ota_batch["phase"] = None
                        _ota_batch["progress"] = None
                        _ota_advance()  # 失败即推进下一台
        _ota_advance()  # 批次推进（含超时检查）
        time.sleep(2)


# 最新检测结果（result 线程写，image 线程读）
latest = {"result": -1, "ratio": 0.0, "kps": []}
lock = threading.Lock()
frame_seq = {}  # devid → 已收到的完整图像帧计数（SAVE_EVERY_N 按设备独立抽稀；多设备合用一个计数会互相挤占）
os.makedirs("received_images", exist_ok=True)


def _open_history_db():
    """打开历史库（无则自动建库建表）；失败返回 None（仅影响历史记录，不影响接收）"""
    try:
        db = sqlite3.connect(HISTORY_DB)
        db.execute(
            "CREATE TABLE IF NOT EXISTS posture_log ("
            "ts REAL PRIMARY KEY, "  # PC 本地时间（unix 秒，浮点）
            "result INTEGER NOT NULL, "  # 0-5，与 RESULT_TAG 对应
            "ratio REAL, "  # 触发条件的比值（前倾判断依据）
            "devid TEXT)"  # 来源设备（WiFi MAC 后 3 字节 hex；多设备统计用）
        )
        try:
            db.execute("ALTER TABLE posture_log ADD COLUMN devid TEXT")  # 旧库迁移（列已存在则忽略）
        except sqlite3.Error:
            pass
        db.commit()
        return db
    except sqlite3.Error as e:
        print(f"[history] 历史库打开失败: {e}")
        return None


def result_receiver():
    """收 20002：包格式 0x02(1) result(1) ratio(f32) 6*(x,y,score)(3×f32) = 78 字节；
    逐帧写入历史库（posture_report.py 查询/绘图用）"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_RESULT_PORT))
    sock.settimeout(1.0)
    db = _open_history_db()
    print(f"[result] 监听 :{UDP_RESULT_PORT}" + (f"，历史记录 → {HISTORY_DB}" if db else ""))
    while True:
        try:
            data, addr = sock.recvfrom(256)
        except socket.timeout:
            continue
        # 1(tag) + 1(result) + 4(ratio) + 6*12(kps) = 78
        if len(data) < 6 + 6 * 12 or data[0] != 0x02:
            continue
        result = data[1]
        ratio = struct.unpack_from("<f", data, 2)[0]  # little-endian (ESP memcpy)
        kps = [struct.unpack_from("<fff", data, 6 + i * 12) for i in range(6)]
        devid = _devid_by_ip(addr[0])  # 按源 IP 归属设备（多设备分开统计/日志过滤）
        with lock:
            latest.update(result=result, ratio=ratio, kps=kps)
            _last_result[devid] = result  # 按设备记录最近结果（img people 模式判定用）
        print(f"[result] [{devid}] {RESULT_TAG.get(result, '?'):11s} ratio={ratio:.3f} " f"conf=[{','.join(f'{k[2]:.2f}' for k in kps)}]")
        if db is not None:
            try:
                db.execute("INSERT OR REPLACE INTO posture_log(ts, result, ratio, devid) VALUES(?, ?, ?, ?)", (time.time(), result, ratio, devid))
                db.commit()
            except sqlite3.Error:
                print("[history] 写库失败，历史记录停用（接收不受影响）")
                db = None


def image_receiver():
    """收 20000：分包 JPEG，包头 chunk_id/total_chunks/image_size（网络字节序）"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_IMG_PORT))
    sock.settimeout(5.0)
    print(f"[image]  监听 :{UDP_IMG_PORT}")
    buf = bytearray()
    total = 0
    expected = 1
    while True:
        try:
            data, addr = sock.recvfrom(65535)
        except socket.timeout:
            continue
        if len(data) < 12:
            continue
        chunk_id, _total_chunks, image_size = struct.unpack_from("!III", data, 0)
        payload = data[12:]
        if chunk_id == 0:  # 新一帧开始
            buf = bytearray()
            total = image_size
            expected = 1
            buf.extend(payload)
        elif chunk_id == expected:  # 顺序到达
            buf.extend(payload)
            expected = chunk_id + 1
            if total > 0 and len(buf) >= total:
                on_image(bytes(buf), _devid_by_ip(addr[0]))
                buf = bytearray()
                total = 0
                expected = 0


_last_result = {}    # devid → 最近一帧 result（result_receiver 按设备记录）
_nobody_streak = {}  # devid → 连续无有效结果的图像帧数（people 模式停存判定）


def _person_present(devid):
    """people 模式判定：该设备最近结果有效（OK/BAD=有人）→ 保存并清零计数；
    尚无结果包（刚启动/旧固件）→ 保守视为有人；连续 IMG_NOBODY_SKIP_N 帧无效 → 停止保存。
    有效口径与统计一致：result 0=OK 1=BAD_NECK 2=BAD_SHOULDER（3..5 无效）"""
    r = _last_result.get(devid)
    if r is None or r in (0, 1, 2):
        _nobody_streak[devid] = 0
        return True
    _nobody_streak[devid] = _nobody_streak.get(devid, 0) + 1
    return _nobody_streak[devid] <= IMG_NOBODY_SKIP_N  # 前 N 帧仍保存（容忍短暂离场），之后停


def on_image(jpg, devid=""):
    devid = devid or "unknown"
    cfg = _img_cfg_for(devid)  # 该设备生效的抽稀/叠加设置（默认 + 专属覆盖）
    if cfg["save_n"] < 1:
        return  # 0=不保存：不解码不落盘也不计数（重开后重新抽稀，首帧即存）
    if cfg.get("people") and not _person_present(devid):
        return  # 仅保存有人帧：连续 IMG_NOBODY_SKIP_N 帧无有效结果 → 停存（不推进抽稀计数）
    n = frame_seq.get(devid, 0) + 1
    frame_seq[devid] = n
    if (n - 1) % cfg["save_n"] != 0:  # 该设备首帧即保存，之后每 N 帧存 1 帧
        return

    if len(jpg) < 2 or jpg[0] != 0xFF or jpg[1] != 0xD8:
        print("[image]  非 JPEG，丢弃")
        return
    try:
        img = Image.open(io.BytesIO(jpg)).convert("RGB").resize((W, H))
    except Exception as e:
        print(f"[image]  解码失败: {e}")
        return
    draw = ImageDraw.Draw(img)

    with lock:
        kps = list(latest["kps"])
        result = latest["result"]

    if len(kps) == 6:
        if cfg["draw_kp"]:  # 关键点叠加开关（连线 + 圆圈 + 标签）
            # 双眼 / 双耳 / 双肩 三条连线
            for a, b in ((0, 1), (2, 3), (4, 5)):
                draw.line([kps[a][0] * W, kps[a][1] * H, kps[b][0] * W, kps[b][1] * H], fill="white", width=2)
            for i, (x, y, s) in enumerate(kps):
                px, py = x * W, y * H
                # 肩(idx 4,5)用单独阈值绘制，与 shoulders_ok 判定一致
                r = 6 if s >= (SHOULDER_CONF_THRESH if i >= 4 else CONF_THRESH) else 4
                draw.ellipse([px - r, py - r, px + r, py + r], fill=KP_COLORS[i], outline="white")
                draw.text((px + 8, py - 8), f"{KP_NAMES[i]}:{s:.2f}", fill="yellow")
        # 左上角：复刻 ESP judge_posture 4 条标准（PC 端参考；权威结果以 ESP 发来的 result 为准）
        valid = [s >= CONF_THRESH for (_, _, s) in kps]
        le, re, lear, rear, ls, rs = kps
        eyes_ok = valid[0] and valid[1]
        ears_ok = valid[2] and valid[3]
        shoulders_ok = (kps[4][2] >= SHOULDER_CONF_THRESH) and (kps[5][2] >= SHOULDER_CONF_THRESH)

        def _fmt(v):
            return f"{v:.2f}" if v is not None else " -- "

        eye_ratio = eye_rel = ear_ratio = ear_rel = None
        judge = "NOT_DET"
        if shoulders_ok and (eyes_ok or ears_ok):
            sh_t = _norm_tilt(math.degrees(math.atan2(rs[1] - ls[1], rs[0] - ls[0])))
            sh_y = (ls[1] + rs[1]) * 0.5
            sh_dist = math.hypot(rs[0] - ls[0], rs[1] - ls[1])  # 肩距：人物大小代理
            # 几何合理性预检（与 ESP 一致）：同组连线近垂直属明显误检，该组不参与判断
            eye_t = _norm_tilt(math.degrees(math.atan2(re[1] - le[1], re[0] - le[0])))
            ear_t = _norm_tilt(math.degrees(math.atan2(rear[1] - lear[1], rear[0] - lear[0])))
            eyes_usable = eyes_ok and abs(eye_t) <= EYE_LINE_TILT_MAX
            ears_usable = ears_ok and abs(ear_t) <= EAR_LINE_TILT_MAX
            if sh_dist < SHOULDER_DIST_MIN:
                judge = "TOO_FAR"  # 人太远，定位噪声占比过大 → 本帧不判断
            elif abs(sh_t) > SHOULDER_LINE_TILT_MAX or (not eyes_usable and not ears_usable):
                judge = "UNRELIABLE"  # 双肩基准误检 或 头部连线全不合理 → 本帧不判断
            else:
                if eyes_usable:  # 条件 1 & 3 指标
                    edx, edy = re[0] - le[0], re[1] - le[1]
                    d = math.hypot(edx, edy)
                    eye_ratio = (sh_y - (le[1] + re[1]) * 0.5) / d if d > 0.001 else 0.0
                    eye_rel = _norm_tilt(eye_t - sh_t)
                if ears_usable:  # 条件 2 & 4 指标
                    edx, edy = rear[0] - lear[0], rear[1] - lear[1]
                    d = math.hypot(edx, edy)
                    ear_ratio = (sh_y - (lear[1] + rear[1]) * 0.5) / d if d > 0.001 else 0.0
                    ear_rel = _norm_tilt(ear_t - sh_t)
                # 判断（眼优先；任意成立即不良）
                judge = "OK"
                if eyes_usable:
                    if eye_ratio < EYE_FORWARD_RATIO_MIN:
                        judge = "BAD_NECK"
                    elif abs(eye_rel) > EYE_HEAD_TILT_WARN:
                        judge = "BAD_SHOULDER"
                if judge == "OK" and ears_usable:
                    if ear_ratio < EAR_FORWARD_RATIO_MIN:
                        judge = "BAD_NECK"
                    elif abs(ear_rel) > EAR_HEAD_TILT_WARN:
                        judge = "BAD_SHOULDER"

        info = [
            f"result: {RESULT_TAG.get(result, '?')}  (pc: {judge})",
            f"eye: ratio={_fmt(eye_ratio)} rel={_fmt(eye_rel)}  [thr<{EYE_FORWARD_RATIO_MIN}, >{EYE_HEAD_TILT_WARN:.0f}]",
            f"ear: ratio={_fmt(ear_ratio)} rel={_fmt(ear_rel)}  [thr<{EAR_FORWARD_RATIO_MIN}, >{EAR_HEAD_TILT_WARN:.0f}]",
            f"conf: [{','.join(f'{k[2]:.2f}' for k in kps)}]",
        ]
        color = "red" if result in (1, 2) else "lime"  # 坐姿不良(BAD_NECK/BAD_SHOULDER)显示红色
        if cfg["draw_kp"]:  # 左上角判断文字同样受叠加开关控制
            yy = 4
            for line in info:
                draw.text((4, yy), line, fill=color)
                yy += 12

    # 右下角时间戳：PC 收到本帧的时刻（同样受叠加开关控制）
    if cfg["draw_kp"]:
        stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        tw = draw.textlength(stamp)
        draw.text((W - tw - 4, H - 14), stamp, fill="yellow")

    ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    out_dir = os.path.join("received_images", devid)  # 按设备分目录（旧固件 devid=源 IP，同样可用）
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, f"image_{ts}.png")
    img.save(out)
    print(f"[image]  [{devid}] 保存 {out}  第{n}帧 ({RESULT_TAG.get(result, '?')})")


def _img_save_desc(n):
    """抽稀设置的显示文案（0=不保存）"""
    return "不保存图片" if n < 1 else f"每 {n} 帧存 1 帧"


def _img_cmd(arg=""):
    """图像保存设置命令（键盘与 Web 共用）：
    '' = 概览；kp on|off = 关键点叠加；save N = 抽稀保存（0=不保存）；people on|off = 仅保存有人帧
    （最近结果连续 50 帧无效即停存，恢复有人立即续存）；reset @ID = 清除设备专属"""
    usage = "[img] 用法: img | img kp on|off | img save N（0=不保存） | img people on|off | 指定设备 img kp @<序号|ID> on|off / img save @<序号|ID> N / img people @<序号|ID> on|off / img reset @<序号|ID>"
    if not arg:
        d = _img_cfg["default"]
        print(f"[img] 默认(全部设备): 关键点叠加{'开' if d['draw_kp'] else '关'} | {_img_save_desc(d['save_n'])} | 仅有人帧{'开' if d['people'] else '关'}")
        for k, v in _img_cfg["devices"].items():
            if v:
                c = _img_cfg_for(k)
                print(f"[img]   {k}（专属）: 叠加{'开' if c['draw_kp'] else '关'} | {_img_save_desc(c['save_n'])} | 仅有人帧{'开' if c['people'] else '关'}")
        return
    kind, _, rest = arg.partition(" ")
    devid = None
    if rest.startswith("@"):  # img kp @dev on / img save @dev N / img reset @dev
        head, _, rest = rest.partition(" ")
        devid = _resolve_device(head[1:].lower())
        if not devid:
            return
        rest = rest.strip()
    if kind == "reset":
        if devid and _img_cfg["devices"].pop(devid, None) is not None:
            _save_img_cfg()
            print(f"[img] 已清除 {devid} 的专属设置（恢复跟随默认）")
        return
    if kind == "kp" and rest in ("on", "off"):
        val = rest == "on"
        if devid is None:  # 全部设备：写默认并清掉该字段的所有专属
            _img_cfg["default"]["draw_kp"] = val
            for v in _img_cfg["devices"].values():
                v.pop("draw_kp", None)
        else:
            _img_cfg["devices"].setdefault(devid, {})["draw_kp"] = val
    elif kind == "people" and rest in ("on", "off"):
        val = rest == "on"
        if devid is None:
            _img_cfg["default"]["people"] = val
            for v in _img_cfg["devices"].values():
                v.pop("people", None)
        else:
            _img_cfg["devices"].setdefault(devid, {})["people"] = val
    elif kind == "save" and rest.isdigit() and int(rest) >= 0:
        n = int(rest)
        if devid is None:
            _img_cfg["default"]["save_n"] = n
            for v in _img_cfg["devices"].values():
                v.pop("save_n", None)
        else:
            _img_cfg["devices"].setdefault(devid, {})["save_n"] = n
    else:
        print(usage)
        return
    _img_cfg["devices"] = {k: v for k, v in _img_cfg["devices"].items() if v}  # 清空 dict 条目
    _save_img_cfg()
    who = "全部设备" if devid is None else devid
    if kind == "kp":
        what = f"关键点叠加{'开' if rest == 'on' else '关'}"
    elif kind == "people":
        what = f"仅有人帧{'开' if rest == 'on' else '关'}（连续 {IMG_NOBODY_SKIP_N} 帧无人停存）"
    else:
        what = _img_save_desc(int(rest))
    print(f"[img] {who}: {what}（已保存，重启仍生效）")


def handle_command(cmd):
    """处理一条交互命令（控制台 input 与 Web UI 共用同一条路径，行为完全一致）"""
    if not cmd:
        return
    if cmd.lower() == "ota" or cmd.lower().startswith("ota "):
        _ota_cmd(cmd[3:].strip().lower())
        return
    if cmd.lower() == "img" or cmd.lower().startswith("img "):
        _img_cmd(cmd[3:].strip().lower())
        return
    if cmd.lower() == "alert" or cmd.lower().startswith("alert "):
        _alert_cmd(cmd[5:].strip().lower())
        return
    if not cmd.lower().startswith("s"):
        print(f"未知命令: {cmd}（s=设置检测时段，ota=固件升级，img=图像保存设置，alert=不良提醒模式）")
        return
    arg = cmd[1:].strip()
    if not arg:
        print("[sched] 用法: s HH:MM-HH:MM[,HH:MM-HH:MM...]（最多 5 段，支持跨午夜 22:00-06:30）或 s clear；" "指定设备: s @<序号|ID> HH:MM-HH:MM...（只改该设备）")
        return
    devid = None
    if arg.startswith("@"):  # s @<序号|ID> 时段：仅下发该设备（与 ota 命令同一套设备引用）
        head, _, rest = arg.partition(" ")
        devid = _resolve_device(head[1:].lower())
        if not devid:
            return
        arg = rest.strip()
        if not arg:
            print("[sched] 用法: s @<序号|ID> HH:MM-HH:MM... 或 s @<序号|ID> clear")
            return
    try:
        slots = _parse_slots_arg(arg)
    except ValueError as e:
        print(f"[sched] 设置失败(未下发): {e}")
        return
    _sched_set_memory(devid, slots)
    _send_sched_set(slots, devid)


# Web 统计的时间范围：自然周期起点（与 posture_report.py 一致）→ 分桶粒度
_WEB_RANGES = {"day": "hour", "week": "day", "month": "day", "year": "month", "all": "month"}


def _range_start(key):
    """时间窗起点（自然周期，与 posture_report.py 一致）；"all" 返回 None（起点=最早记录）"""
    now = datetime.now()
    if key == "day":
        return now.replace(hour=0, minute=0, second=0, microsecond=0)
    if key == "week":
        monday = now - timedelta(days=now.weekday())
        return monday.replace(hour=0, minute=0, second=0, microsecond=0)
    if key == "month":
        return now.replace(day=1, hour=0, minute=0, second=0, microsecond=0)
    if key == "year":
        return now.replace(month=1, day=1, hour=0, minute=0, second=0, microsecond=0)
    return None


def _bucket_label(k, unit):
    if unit == "hour":
        return f"{k.hour:02d}时"
    if unit == "day":
        return f"{k.month:02d}-{k.day:02d}"
    return f"{k.year}-{k.month:02d}"


def _web_stats(range_key="day", device="all"):
    """Web 坐姿统计（/api/stats）：按设备 + 时间范围出摘要与分桶计数。
    摘要=检测/有效/不良帧数、不良率、不良事件数与最长持续（连续 BAD 间隔>5s 分段，
    与 posture_report.py 口径一致）；桶补零（诚实显示空时段）；失败返回 error 不影响面板"""
    range_key = range_key if range_key in _WEB_RANGES else "day"
    unit = _WEB_RANGES[range_key]
    try:
        db = sqlite3.connect(HISTORY_DB, timeout=1)
        devs = [r[0] for r in db.execute("SELECT devid FROM posture_log WHERE devid IS NOT NULL " "GROUP BY devid ORDER BY MIN(ts)")]  # 有历史记录的设备（首测时间排序）
        for k in _devices:
            if k not in devs and not k.replace(".", "").isdigit():  # 在线但暂无记录的设备也列出（IP 兜底键除外）
                devs.append(k)

        where = "ts >= ?"
        start = _range_start(range_key)
        args = [start.timestamp() if start else 0.0]  # all 用 0.0（Windows 上 epoch0 的 datetime.timestamp() 会抛 OSError）
        if device != "all":
            where += " AND devid = ?"
            args.append(device)
        if start is None:  # 全部：起点收敛到最早记录所在月（无记录则当月）
            row = db.execute(f"SELECT MIN(ts) FROM posture_log WHERE {where}", args).fetchone()
            first = datetime.fromtimestamp(row[0]) if row and row[0] else datetime.now()
            start = first.replace(day=1, hour=0, minute=0, second=0, microsecond=0)

        total = valid = bad = events = 0
        longest_ev = 0.0
        last_bad_ts = None
        ev_start = None
        buckets = {}
        for ts, result in db.execute(f"SELECT ts, result FROM posture_log WHERE {where} ORDER BY ts", args):
            total += 1
            t = datetime.fromtimestamp(ts)
            if unit == "hour":
                key = t.replace(minute=0, second=0, microsecond=0)
            elif unit == "day":
                key = t.replace(hour=0, minute=0, second=0, microsecond=0)
            else:
                key = t.replace(day=1, hour=0, minute=0, second=0, microsecond=0)
            cell = buckets.setdefault(key, [0, 0, 0])
            cell[0] += 1
            if 0 <= result <= 2:
                valid += 1
                cell[2] += 1
                if result >= 1:
                    bad += 1
                    cell[1] += 1
            if result in (1, 2):  # 不良事件分段：连续不良间隔>5s 开新段
                if last_bad_ts is None or ts - last_bad_ts > 5:
                    events += 1
                    ev_start = ts
                last_bad_ts = ts
                if ev_start is not None:
                    longest_ev = max(longest_ev, ts - ev_start)
            else:
                last_bad_ts = None
                ev_start = None
        db.close()

        keys = []  # 补零桶：从窗口起点铺到当前，缺的桶计 0
        now = datetime.now()
        k = start
        if unit == "hour":
            end = now.replace(minute=0, second=0, microsecond=0)
            while k <= end:
                keys.append(k)
                k += timedelta(hours=1)
        elif unit == "day":
            while k <= now:
                keys.append(k)
                k += timedelta(days=1)
        else:
            end = now.replace(day=1, hour=0, minute=0, second=0, microsecond=0)
            while k <= end:
                keys.append(k)
                k = (k.replace(day=1) + timedelta(days=32)).replace(day=1)  # 下个月
        def _bucket_out(k):  # 桶级不良率=该桶不良/该桶有效帧（valid 分母，与摘要同口径；随设备过滤区分全部/单台）
            t, b, v = buckets.get(k, (0, 0, 0))
            return {"label": _bucket_label(k, unit), "total": t, "bad": b,
                    "rate": round(b * 100 / v, 1) if v else None}

        bl = [_bucket_out(k) for k in keys]
        return {
            "range": range_key,
            "device": device,
            "devices": devs,
            "summary": {"total": total, "valid": valid, "bad": bad, "rate": round(bad * 100 / valid, 1) if valid else None, "events": events, "longest_min": round(longest_ev / 60, 1)},
            "buckets": bl,
            "unit": {"hour": "小时", "day": "天", "month": "月"}[unit],
        }
    except sqlite3.Error as e:
        return {"error": str(e), "devices": [], "summary": {}, "buckets": []}


def _web_state():
    """Web 控制台状态快照（web_ui 的 /api/state 每 2s 调一次）：
    设备表、调度状态行、OTA 批次、最新判断、今日统计"""
    want = _bin_sha8(OTA_BIN)
    now = time.time()
    devs = []
    for k, d in _devices.items():
        online = now - d.get("last_seen", 0) < 30
        sched_ok = online and now - d.get("sched_seen", 0) < 90  # 最近一次 SCHED_STATE（2s 查询周期）
        devs.append(
            {
                "id": k,
                "ip": d.get("ip"),
                "ver": d.get("ver"),
                "online": online,
                "up_to_date": bool(want and d.get("sha8") == want),
                "fails": d.get("fails", 0),
                "sched_desc": d.get("sched_desc") if sched_ok else None,  # 检测时段（None=尚未上报）
                "sched_synced": d.get("sched_synced") if sched_ok else None,  # ESP 时间同步状态
                "sched_active": d.get("sched_active") if sched_ok else None,
                "alert": d.get("alert"),  # 不良提醒模式（True=连续播；None=旧固件未上报）
            }
        )
    return {
        "fw_version": _fw_version(),
        "watch": _ota_state["watch"],
        "alert_default": _sched_mem["alert_default"],  # 不良提醒模式默认（全部设备）
        "ota_current": _ota_batch["current"],
        "ota_phase": _ota_batch["phase"],
        "ota_progress": ({"cur": _ota_batch["progress"]["cur"], "tot": _ota_batch["progress"]["tot"],
                          "pct": min(100, _ota_batch["progress"]["cur"] * 100 // _ota_batch["progress"]["tot"])}
                         if _ota_batch["progress"] and _ota_batch["progress"].get("tot") else None),
        "ota_queue": len(_ota_batch["queue"]),
        "devices": devs,
        "sched_map": {k: v["ui"] for k, v in _sched_last.items() if now - v["t"] < 90},  # devid → 调度状态（90s 心跳窗口）
        "img": {"default": dict(_img_cfg["default"]),
                "devices": {k: _img_cfg_for(k) for k in _devices}},  # 各设备生效的抽稀/叠加
        "img_custom": sorted(_img_cfg["devices"]),  # 有专属设置的设备
    }


if __name__ == "__main__":
    sys.stdout.reconfigure(line_buffering=True)  # IDE/管道下 stdout 是块缓冲，子线程 print 会积压不显示
    import web_ui

    web_ui.set_stats_provider(_web_stats)  # 坐姿统计：/api/stats（设备 × 时间范围）
    web_ui.start(_web_state, handle_command)  # Web 控制台 :20005（先装 print 镜像，再启其余线程）
    threading.Thread(target=result_receiver, daemon=True).start()
    threading.Thread(target=image_receiver, daemon=True).start()
    threading.Thread(target=discovery_broadcaster, daemon=True).start()
    print("接收器启动：图像 :20000 + 结果 :20002 + 发现 :20003 → received_images/<设备ID>/image_*.png（按设备分目录）")
    print("Web 控制台: http://localhost:20005（浏览器/手机打开，功能与下方命令等价）")
    print("命令：s 09:00-11:30,14:00-18:00 设置时段(全部) | s @2/a1b2c3 ... 指定设备 | s clear 清空 | ota 升级全部 | ota list 设备表 | ota 2/a1b2c3 指定设备 | ota on/off 自动检测 | img 图像保存设置 | alert repeat|once 不良提醒模式 | Ctrl+C 退出")
    try:
        while True:
            handle_command(input().strip())
    except EOFError:
        # stdin 不可用（后台运行/计划任务/无控制台）：退化为纯接收模式，Ctrl+C 退出
        print("[main] stdin 不可用，进入无交互模式（仅接收/记录）")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n退出")
    except KeyboardInterrupt:
        print("\n退出")
