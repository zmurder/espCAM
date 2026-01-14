#!/usr/bin/env python3
"""
简单的ESP32 UDP图像接收器
用于快速测试和接收图像数据
"""

import socket
import struct
import time
import os
from datetime import datetime

# UDP配置
UDP_IP = "0.0.0.0"  # 监听所有网络接口
UDP_PORT = 8080      # 与ESP32配置的端口一致

def receive_images():
    """接收并保存图像"""
    # 创建保存目录
    os.makedirs("received_images", exist_ok=True)
    
    # 创建UDP套接字
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(10.0)
    
    print(f"UDP图像接收器启动在 {UDP_IP}:{UDP_PORT}")
    print("等待ESP32发送图像...")
    print("按 Ctrl+C 停止接收")
    
    current_image = None
    total_size = 0
    received_data = b''
    chunk_count = 0
    
    try:
        while True:
            try:
                # 接收数据
                data, addr = sock.recvfrom(65535)
                print(f"接收到来自 {addr[0]}:{addr[1]} 的数据，长度: {len(data)} 字节")
                
                # 解析包头（前12字节）
                if len(data) >= 12:
                    chunk_id = struct.unpack_from('!I', data, 0)[0]
                    total_chunks = struct.unpack_from('!I', data, 4)[0]
                    image_size = struct.unpack_from('!I', data, 8)[0]
                    
                    image_data = data[12:]  # 去掉包头
                    
                    print(f"包信息: ID={chunk_id}, 总包数={total_chunks}, 图像大小={image_size}")
                    
                    # 如果是新图像的开始
                    if chunk_id == 0:
                        if current_image is not None:
                            print(f"保存之前的图像: {len(received_data)} 字节")
                            save_image(received_data)
                        
                        current_image = bytearray()
                        total_size = image_size
                        received_data = image_data
                        chunk_count = 1
                        print(f"开始新图像，总大小: {image_size} 字节")
                    
                    else:
                        # 添加数据到当前图像
                        received_data += image_data
                        chunk_count += 1
                    
                    # 检查是否接收完成
                    if len(received_data) >= total_size:
                        print(f"图像接收完成！总大小: {len(received_data)} 字节")
                        save_image(received_data)
                        current_image = None
                        received_data = b''
                        total_size = 0
                        chunk_count = 0
                
            except socket.timeout:
                print("等待数据...")
                continue
                
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        print(f"错误: {e}")
    finally:
        sock.close()
        if received_data:
            print(f"保存未完成的图像: {len(received_data)} 字节")
            save_image(received_data)

def save_image(image_data):
    """保存图像到文件"""
    try:
        # 生成文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filename = f"received_images/image_{timestamp}.jpg"
        
        # 检查JPEG文件头
        if len(image_data) >= 2 and image_data[0] == 0xFF and image_data[1] == 0xD8:
            print(f"保存JPEG图像: {filename}")
        else:
            print(f"保存二进制数据: {filename}")
        
        # 保存文件
        with open(filename, 'wb') as f:
            f.write(image_data)
        
        print(f"文件已保存: {len(image_data)} 字节")
        
    except Exception as e:
        print(f"保存文件错误: {e}")

if __name__ == "__main__":
    receive_images()