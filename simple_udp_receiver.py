#!/usr/bin/env python3
"""
ESP32 UDP图像和姿态检测结果接收器
接收图像和姿态检测结果，在图片上绘制关键点后保存
"""

import socket
import struct
import time
import os
import sys
from datetime import datetime

try:
    import cv2
    import numpy as np
    HAS_OPENCV = True
except ImportError:
    HAS_OPENCV = False
    print("Warning: OpenCV not installed, will only save raw images")

# UDP配置
UDP_IP = "0.0.0.0"
UDP_PORT = 8080        # 与ESP32配置的端口一致
POSTURE_PORT = 8082    # 姿态结果专用端口

# 关键点名称
KEYPOINT_NAMES = [
    "left_eye", "right_eye", "left_ear", "right_ear",
    "nose", "left_shoulder", "right_shoulder"
]

# 关键点颜色 (BGR)
KEYPOINT_COLORS = [
    (255, 0, 0),    # 左眼 - 蓝色
    (0, 255, 0),    # 右眼 - 绿色
    (255, 255, 0),  # 左耳 - 青色
    (0, 255, 255),  # 右耳 - 黄色
    (255, 0, 255),  # 鼻子 - 紫色
    (0, 128, 255),  # 左肩 - 橙色
    (0, 0, 255),    # 右肩 - 红色
]

# 关键点连接（用于绘制骨架）
KEYPOINT_CONNECTIONS = [
    (0, 1),   # left_eye - right_eye (眼睛)
    (0, 2),   # left_eye - left_ear
    (1, 3),   # right_eye - right_ear
    (4, 5),   # nose - left_shoulder
    (4, 6),   # nose - right_shoulder
    (5, 6),   # left_shoulder - right_shoulder (肩膀)
]

class PostureReceiver:
    def __init__(self):
        # 创建两个socket：图像和姿态结果
        self.image_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.image_sock.bind((UDP_IP, UDP_PORT))
        self.image_sock.settimeout(5.0)

        self.posture_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.posture_sock.bind((UDP_IP, POSTURE_PORT))
        self.posture_sock.settimeout(5.0)

        self.current_posture = None  # 当前姿态结果
        self.last_posture_time = 0

        os.makedirs("received_images", exist_ok=True)
        os.makedirs("debug_images", exist_ok=True)

        print(f"UDP接收器启动")
        print(f"  图像端口: {UDP_PORT}")
        print(f"  姿态端口: {POSTURE_PORT}")

    def receive_posture(self):
        """接收姿态检测结果"""
        try:
            data, addr = self.posture_sock.recvfrom(1024)
            if len(data) >= 1 and data[0] == 0x02:
                self.parse_posture_data(data)
        except socket.timeout:
            pass
        except Exception as e:
            print(f"接收姿态数据错误: {e}")

    def parse_posture_data(self, data):
        """解析姿态数据"""
        offset = 1  # 跳过包类型
        result = data[offset]
        offset += 1

        ratio = struct.unpack_from('f', data, offset)[0]
        offset += 4

        keypoints = []
        for i in range(7):
            x = struct.unpack_from('f', data, offset)[0]
            offset += 4
            y = struct.unpack_from('f', data, offset)[0]
            offset += 4
            score = struct.unpack_from('f', data, offset)[0]
            offset += 4
            keypoints.append({'x': x, 'y': y, 'score': score, 'name': KEYPOINT_NAMES[i]})

        self.current_posture = {
            'result': result,
            'ratio': ratio,
            'keypoints': keypoints,
            'timestamp': datetime.now()
        }
        self.last_posture_time = time.time()

        result_str = ["OK", "BAD_NECK", "BAD_SHOULDER", "NOT_DETECTED"][result]
        print(f"姿态结果: {result_str}, ratio={ratio:.3f}")

    def draw_keypoints(self, img):
        """在图像上绘制关键点和骨架"""
        if self.current_posture is None:
            return img

        # 检查姿态结果是否过期（超过2秒）
        if time.time() - self.last_posture_time > 2.0:
            return img

        h, w = img.shape[:2]

        # 根据姿态结果选择骨架颜色
        result = self.current_posture['result']
        if result == 0:  # OK
            skeleton_color = (0, 255, 0)  # 绿色
        elif result == 1:  # BAD_NECK
            skeleton_color = (0, 0, 255)  # 红色
        elif result == 2:  # BAD_SHOULDER
            skeleton_color = (0, 165, 255)  # 橙色
        else:  # NOT_DETECTED
            skeleton_color = (128, 128, 128)  # 灰色

        # 绘制骨架连接线
        for conn in KEYPOINT_CONNECTIONS:
            kp1 = self.current_posture['keypoints'][conn[0]]
            kp2 = self.current_posture['keypoints'][conn[1]]

            # 只绘制置信度足够高的点
            if kp1['score'] > 0.05 and kp2['score'] > 0.05:
                pt1 = (int(kp1['x'] * w), int(kp1['y'] * h))
                pt2 = (int(kp2['x'] * w), int(kp2['y'] * h))
                cv2.line(img, pt1, pt2, skeleton_color, 2)

        # 绘制每个关键点（使用各自独立的颜色）
        for i, kp in enumerate(self.current_posture['keypoints']):
            if kp['score'] > 0.05:
                pt = (int(kp['x'] * w), int(kp['y'] * h))
                color = KEYPOINT_COLORS[i]
                # 绘制白色边框
                cv2.circle(img, pt, 7, (255, 255, 255), 2)
                # 绘制关键点本身
                cv2.circle(img, pt, 5, color, -1)
                # 绘制标签
                cv2.putText(img, kp['name'][:3], (pt[0]+8, pt[1]-5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # 在图像上添加姿态信息
        result_str = ["OK", "BAD_NECK", "BAD_SHOULDER", "NOT_DETECTED"][result]
        cv2.putText(img, f"Posture: {result_str}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, skeleton_color, 2)
        cv2.putText(img, f"Ratio: {self.current_posture['ratio']:.3f}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        return img

    def save_image_with_keypoints(self, img):
        """保存带有关键点标注的图像"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]

        # 保存带标注的图像
        debug_path = f"debug_images/debug_{timestamp}.jpg"
        cv2.imwrite(debug_path, img)
        print(f"调试图像已保存: {debug_path}")

        # 同时保存原始图像
        raw_path = f"received_images/image_{timestamp}.jpg"
        # 去除绘制的信息（如果需要原始图像）
        # 但我们已经覆盖保存了，所以这里只记录

    def process_images(self):
        """处理图像和姿态数据"""
        print("等待接收数据... 按 Ctrl+C 停止")

        current_image = None
        received_data = b''
        total_size = 0

        while True:
            # 优先接收姿态结果
            self.receive_posture()

            # 接收图像数据
            try:
                data, addr = self.image_sock.recvfrom(65535)

                if len(data) >= 12:
                    chunk_id = struct.unpack_from('!I', data, 0)[0]
                    total_chunks = struct.unpack_from('!I', data, 4)[0]
                    image_size = struct.unpack_from('!I', data, 8)[0]
                    image_data = data[12:]

                    print(f"收到UDP包: chunk_id={chunk_id}, total_chunks={total_chunks}, image_size={image_size}, len={len(data)}, data_len={len(image_data)}")

                    if chunk_id == 0:
                        if current_image is not None and len(received_data) > 0:
                            print(f"丢弃未完成的图像: received={len(received_data)}, expected={total_size}")
                            self.save_received_image(received_data)

                        current_image = bytearray()
                        total_size = image_size
                        received_data = bytearray(image_data)
                        print(f"新图像开始: total_size={total_size}, first_chunk_size={len(image_data)}")
                    else:
                        received_data.extend(image_data)
                        print(f"追加数据: received={len(received_data)}, target={total_size}")

                    if len(received_data) >= total_size:
                        print(f"图像接收完成: {len(received_data)} bytes")
                        self.process_received_image(bytes(received_data))
                        current_image = None
                        received_data = b''
                        total_size = 0

            except socket.timeout:
                continue
            except Exception as e:
                print(f"处理图像错误: {e}")
                continue

    def process_received_image(self, image_data):
        """处理接收到的完整图像"""
        if not HAS_OPENCV:
            # 没有OpenCV，只保存原始数据
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            path = f"received_images/image_{timestamp}.jpg"
            with open(path, 'wb') as f:
                f.write(image_data)
            print(f"图像已保存: {path}")
            return

        # 检查 JPEG 魔术字节
        if len(image_data) < 2:
            print(f"JPEG数据太短: {len(image_data)} 字节")
            return

        print(f"JPEG原始数据前20字节: {image_data[:20].hex()}")

        # 解码JPEG图像
        nparr = np.frombuffer(image_data, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        if img is None:
            print(f"JPEG解码失败，数据长度={len(image_data)}, 前2字节={image_data[:2].hex()}")
            return

        # 绘制关键点
        img = self.draw_keypoints(img)

        # 保存带标注的图像
        self.save_image_with_keypoints(img)

    def save_received_image(self, image_data):
        """保存原始图像（不标注，用于对比）"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        path = f"received_images/raw_{timestamp}.jpg"
        with open(path, 'wb') as f:
            f.write(image_data)

    def close(self):
        self.image_sock.close()
        self.posture_sock.close()

if __name__ == "__main__":
    if not HAS_OPENCV:
        print("Warning: OpenCV not installed. Install with: pip install opencv-python")
        print("Only raw image saving will work.")

    receiver = PostureReceiver()
    try:
        receiver.process_images()
    except KeyboardInterrupt:
        print("\n用户中断程序")
    finally:
        receiver.close()