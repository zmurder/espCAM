#!/usr/bin/env python3
"""
ESP32 坐姿检测 — 图像 + 检测结果 UDP 接收器
================================================
同时监听两个端口，把"图像 + 检测关键点"叠加后保存：
  8080 : 推理帧 JPEG 图像（分包，与 ESP send_image_via_udp 对应）
  8082 : 检测结果（0x02 + result + ratio + 6 关键点 x/y/conf）

每收到一帧完整图像，叠加最新的检测结果，保存到 received/latest.png。
配合 ESP32 app_main.c 的开关：SEND_IMAGE_VIA_UDP / SEND_RESULT_VIA_UDP。

用法：
  python simple_udp_receiver.py
  打开 received/latest.png 查看（每帧覆盖更新）；Ctrl+C 退出。
"""
import io
import math
import os
import socket
import struct
import threading
import time
from datetime import datetime

from PIL import Image, ImageDraw

UDP_IMG_PORT = 8080       # 图像
UDP_RESULT_PORT = 8082    # 检测结果

W, H = 320, 240           # 模型输入尺寸，关键点归一化坐标基准
CONF_THRESH = 0.4              # 与 ESP 一致：眼/耳置信度阈值
SHOULDER_CONF_THRESH = 0.2     # 与 ESP 一致：双肩单独阈值（更低，趴近时肩仍可用）
# 6 关键点顺序与 ESP KEYPOINT_* 枚举一致：0=L_eye 1=R_eye 2=L_ear 3=R_ear 4=L_sh 5=R_sh
KP_NAMES = ["L_eye", "R_eye", "L_ear", "R_ear", "L_sh", "R_sh"]
KP_COLORS = ["red", "blue", "magenta", "purple", "orange", "cyan"]
RESULT_TAG = {0: "OK", 1: "BAD_NECK", 2: "BAD_SHOULDER", 3: "NOT_DET"}

# 与 ESP judge_posture 一致的阈值（4 条任意成立即不良；左上角显示参考用）
EYE_FORWARD_RATIO_MIN = 1.5   # 条件1：眼肩垂直距离/双眼距离 < 此值 → 前倾（需据正常坐姿标定）
EAR_FORWARD_RATIO_MIN = 1.5   # 条件2：耳肩垂直距离/双耳距离 < 此值 → 前倾
EYE_HEAD_TILT_WARN = 35.0     # 条件3：双眼-双肩相对倾斜角 > 此值 → 歪头
EAR_HEAD_TILT_WARN = 35.0     # 条件4：双耳-双肩相对倾斜角 > 此值 → 歪头


def _norm_tilt(deg):
    """把 atan2 角度归一化到 [-90,90]，使水平(0°或±180°)都映射为 0°"""
    if deg > 90:
        return deg - 180
    if deg < -90:
        return deg + 180
    return deg

# 最新检测结果（result 线程写，image 线程读）
latest = {"result": -1, "ratio": 0.0, "kps": []}
lock = threading.Lock()
os.makedirs("received_images", exist_ok=True)


def result_receiver():
    """收 8082：包格式 0x02(1) result(1) ratio(f32) 6*(x,y,score)(3×f32) = 78 字节"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_RESULT_PORT))
    sock.settimeout(1.0)
    print(f"[result] 监听 :{UDP_RESULT_PORT}")
    while True:
        try:
            data, _ = sock.recvfrom(256)
        except socket.timeout:
            continue
        # 1(tag) + 1(result) + 4(ratio) + 6*12(kps) = 78
        if len(data) < 6 + 6 * 12 or data[0] != 0x02:
            continue
        result = data[1]
        ratio = struct.unpack_from("<f", data, 2)[0]                       # little-endian (ESP memcpy)
        kps = [struct.unpack_from("<fff", data, 6 + i * 12) for i in range(6)]
        with lock:
            latest.update(result=result, ratio=ratio, kps=kps)
        print(f"[result] {RESULT_TAG.get(result, '?'):11s} ratio={ratio:.3f} "
              f"conf=[{','.join(f'{k[2]:.2f}' for k in kps)}]")


def image_receiver():
    """收 8080：分包 JPEG，包头 chunk_id/total_chunks/image_size（网络字节序）"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_IMG_PORT))
    sock.settimeout(5.0)
    print(f"[image]  监听 :{UDP_IMG_PORT}")
    buf = bytearray()
    total = 0
    expected = 1
    while True:
        try:
            data, _ = sock.recvfrom(65535)
        except socket.timeout:
            continue
        if len(data) < 12:
            continue
        chunk_id, _total_chunks, image_size = struct.unpack_from("!III", data, 0)
        payload = data[12:]
        if chunk_id == 0:                       # 新一帧开始
            buf = bytearray()
            total = image_size
            expected = 1
            buf.extend(payload)
        elif chunk_id == expected:              # 顺序到达
            buf.extend(payload)
            expected = chunk_id + 1
            if total > 0 and len(buf) >= total:
                on_image(bytes(buf))
                buf = bytearray()
                total = 0
                expected = 0


def on_image(jpg):
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
        # 双眼 / 双耳 / 双肩 三条连线
        for a, b in ((0, 1), (2, 3), (4, 5)):
            draw.line([kps[a][0]*W, kps[a][1]*H, kps[b][0]*W, kps[b][1]*H], fill="white", width=2)
        for i, (x, y, s) in enumerate(kps):
            px, py = x * W, y * H
            # 肩(idx 4,5)用单独阈值绘制，与 shoulders_ok 判定一致
            r = 6 if s >= (SHOULDER_CONF_THRESH if i >= 4 else CONF_THRESH) else 4
            draw.ellipse([px-r, py-r, px+r, py+r], fill=KP_COLORS[i], outline="white")
            draw.text((px+8, py-8), f"{KP_NAMES[i]}:{s:.2f}", fill="yellow")
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
            if eyes_ok:  # 条件 1 & 3 指标
                edx, edy = re[0] - le[0], re[1] - le[1]
                d = math.hypot(edx, edy)
                eye_ratio = (sh_y - (le[1] + re[1]) * 0.5) / d if d > 0.001 else 0.0
                eye_rel = _norm_tilt(_norm_tilt(math.degrees(math.atan2(edy, edx))) - sh_t)
            if ears_ok:  # 条件 2 & 4 指标
                edx, edy = rear[0] - lear[0], rear[1] - lear[1]
                d = math.hypot(edx, edy)
                ear_ratio = (sh_y - (lear[1] + rear[1]) * 0.5) / d if d > 0.001 else 0.0
                ear_rel = _norm_tilt(_norm_tilt(math.degrees(math.atan2(edy, edx))) - sh_t)
            # 判断（眼优先；任意成立即不良）
            judge = "OK"
            if eyes_ok:
                if eye_ratio < EYE_FORWARD_RATIO_MIN:
                    judge = "BAD_NECK"
                elif abs(eye_rel) > EYE_HEAD_TILT_WARN:
                    judge = "BAD_SHOULDER"
            if judge == "OK" and ears_ok:
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
        yy = 4
        for line in info:
            draw.text((4, yy), line, fill=color)
            yy += 12

    ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    out = f"received_images/image_{ts}.png"
    img.save(out)
    print(f"[image]  保存 {out}  ({RESULT_TAG.get(result, '?')})")


if __name__ == "__main__":
    threading.Thread(target=result_receiver, daemon=True).start()
    threading.Thread(target=image_receiver, daemon=True).start()
    print("接收器启动：图像 :8080 + 结果 :8082 → received_images/image_*.png（Ctrl+C 退出）")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n退出")
