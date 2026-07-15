#!/usr/bin/env python3
"""
ESP32 坐姿检测 — 图像 + 检测结果 UDP 接收器
================================================
同时监听两个端口，把"图像 + 检测关键点"叠加后保存：
  8080 : 推理帧 JPEG 图像（分包，与 ESP send_image_via_udp 对应）
  8082 : 检测结果（0x02 + result + ratio + 4 关键点 x/y/conf）

每收到一帧完整图像，叠加最新的检测结果，保存到 received/latest.png。
配合 ESP32 app_main.c 的开关：SEND_IMAGE_VIA_UDP / SEND_RESULT_VIA_UDP。

用法：
  python simple_udp_receiver.py
  打开 received/latest.png 查看（每帧覆盖更新）；Ctrl+C 退出。
"""
import io
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
CONF_THRESH = 0.6
KP_NAMES = ["L_eye", "R_eye", "L_sh", "R_sh"]
KP_COLORS = ["red", "blue", "orange", "cyan"]
RESULT_TAG = {0: "OK", 1: "BAD_NECK", 2: "BAD_SHOULDER", 3: "NOT_DET"}

# 最新检测结果（result 线程写，image 线程读）
latest = {"result": -1, "ratio": 0.0, "kps": []}
lock = threading.Lock()
os.makedirs("received_images", exist_ok=True)


def result_receiver():
    """收 8082：包格式 0x02(1) result(1) ratio(f32) 4*(x,y,score)(3×f32)"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_RESULT_PORT))
    sock.settimeout(1.0)
    print(f"[result] 监听 :{UDP_RESULT_PORT}")
    while True:
        try:
            data, _ = sock.recvfrom(256)
        except socket.timeout:
            continue
        if len(data) < 6 or data[0] != 0x02:
            continue
        result = data[1]
        ratio = struct.unpack_from("<f", data, 2)[0]                       # little-endian (ESP memcpy)
        kps = [struct.unpack_from("<fff", data, 6 + i * 12) for i in range(4)]
        with lock:
            latest.update(result=result, ratio=ratio, kps=kps)
        print(f"[result] {RESULT_TAG.get(result, '?'):11s} ratio={ratio:.3f} "
              f"conf=[{kps[0][2]:.2f},{kps[1][2]:.2f},{kps[2][2]:.2f},{kps[3][2]:.2f}]")


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
        ratio = latest["ratio"]

    if len(kps) == 4:
        # 双眼、双肩连线
        draw.line([kps[0][0]*W, kps[0][1]*H, kps[1][0]*W, kps[1][1]*H], fill="white", width=2)
        draw.line([kps[2][0]*W, kps[2][1]*H, kps[3][0]*W, kps[3][1]*H], fill="white", width=2)
        for i, (x, y, s) in enumerate(kps):
            px, py = x * W, y * H
            r = 6 if s >= CONF_THRESH else 4
            draw.ellipse([px-r, py-r, px+r, py+r], fill=KP_COLORS[i], outline="white")
            draw.text((px+8, py-8), f"{KP_NAMES[i]}:{s:.2f}", fill="yellow")
        draw.text((4, 4), f"{RESULT_TAG.get(result, '?')} ratio={ratio:.2f}", fill="lime")

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
