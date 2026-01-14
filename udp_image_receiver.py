#!/usr/bin/env python3
"""
ESP32 UDP图像接收器
接收并保存ESP32发送的UDP图像数据
支持分包传输和图像重组
"""

import socket
import struct
import time
import os
from datetime import datetime

# UDP配置
UDP_IP = "0.0.0.0"  # 监听所有网络接口
UDP_PORT = 8080      # 与ESP32配置的端口一致
BUFFER_SIZE = 65535  # 最大缓冲区大小

# 图像分包结构
# uint32_t chunk_id;      // 包序号
# uint32_t total_chunks;  // 总包数
# uint32_t image_size;    // 图像总大小
# uint8_t data[...];      // 数据区域

CHUNK_HEADER_SIZE = 12  # 3个uint32_t = 12字节

class ImageReceiver:
    def __init__(self, save_dir="received_images"):
        self.save_dir = save_dir
        self.socket = None
        self.current_image = None
        self.expected_chunks = 0
        self.received_chunks = 0
        self.total_size = 0
        self.chunk_data = {}
        self.last_image_time = 0
        
        # 创建保存目录
        os.makedirs(save_dir, exist_ok=True)
        
        print(f"UDP图像接收器启动")
        print(f"监听地址: {UDP_IP}:{UDP_PORT}")
        print(f"保存目录: {save_dir}")
    
    def start(self):
        """启动UDP服务器"""
        try:
            # 创建UDP套接字
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind((UDP_IP, UDP_PORT))
            self.socket.settimeout(30.0)  # 30秒超时
            
            print("等待ESP32图像数据...")
            
            while True:
                try:
                    # 接收数据
                    data, addr = self.socket.recvfrom(BUFFER_SIZE)
                    self.process_packet(data, addr)
                    
                except socket.timeout:
                    print("接收超时，继续等待...")
                    continue
                    
        except Exception as e:
            print(f"错误: {e}")
        finally:
            if self.socket:
                self.socket.close()
    
    def process_packet(self, data, addr):
        """处理接收到的数据包"""
        if len(data) < CHUNK_HEADER_SIZE:
            print(f"警告: 接收到无效数据包，长度: {len(data)}")
            return
        
        try:
            # 解析包头
            chunk_id = struct.unpack_from('!I', data, 0)[0]
            total_chunks = struct.unpack_from('!I', data, 4)[0]
            image_size = struct.unpack_from('!I', data, 8)[0]
            
            # 提取数据部分
            chunk_data = data[CHUNK_HEADER_SIZE:]
            
            # 检查是否是新图像的开始
            if chunk_id == 0 and total_chunks > 0:
                self.start_new_image(total_chunks, image_size)
            
            # 处理数据包
            if self.current_image is not None:
                self.add_chunk(chunk_id, chunk_data)
                
        except struct.error as e:
            print(f"解析数据包错误: {e}")
    
    def start_new_image(self, total_chunks, image_size):
        """开始接收新图像"""
        # 如果有未完成的图像，丢弃它
        if self.current_image is not None:
            print(f"丢弃未完成的图像，已接收 {self.received_chunks}/{self.expected_chunks} 包")
        
        self.current_image = bytearray()
        self.expected_chunks = total_chunks
        self.received_chunks = 0
        self.total_size = image_size
        self.chunk_data = {}
        
        print(f"开始接收新图像: {total_chunks} 包, 大小: {image_size} 字节")
    
    def add_chunk(self, chunk_id, chunk_data):
        """添加图像数据块"""
        if chunk_id >= self.expected_chunks:
            print(f"警告: 无效的包序号 {chunk_id}, 期望范围: 0-{self.expected_chunks-1}")
            return
        
        # 存储数据块
        self.chunk_data[chunk_id] = chunk_data
        self.received_chunks += 1
        
        # 显示接收进度
        if self.received_chunks % 5 == 0 or self.received_chunks == self.expected_chunks:
            progress = (self.received_chunks / self.expected_chunks) * 100
            print(f"接收进度: {self.received_chunks}/{self.expected_chunks} ({progress:.1f}%)")
        
        # 检查是否接收完成
        if self.received_chunks >= self.expected_chunks:
            self.complete_image()
    
    def complete_image(self):
        """图像接收完成，保存图像"""
        try:
            # 按顺序重组图像数据
            for i in range(self.expected_chunks):
                if i in self.chunk_data:
                    self.current_image.extend(self.chunk_data[i])
                else:
                    print(f"错误: 缺少第 {i} 包数据")
                    return
            
            # 验证图像大小
            if len(self.current_image) != self.total_size:
                print(f"警告: 图像大小不匹配，期望: {self.total_size}, 实际: {len(self.current_image)}")
            
            # 检查是否为有效的JPEG数据
            if len(self.current_image) < 4:
                print("错误: 图像数据过小")
                return
            
            # 检查JPEG文件头 (FF D8 FF)
            if self.current_image[0] != 0xFF or self.current_image[1] != 0xD8:
                print("警告: 图像数据可能不是有效的JPEG格式")
            
            # 生成文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"image_{timestamp}.jpg"
            filepath = os.path.join(self.save_dir, filename)
            
            # 保存图像
            with open(filepath, 'wb') as f:
                f.write(self.current_image)
            
            print(f"图像保存成功: {filepath} ({len(self.current_image)} 字节)")
            
            # 计算接收间隔
            current_time = time.time()
            if self.last_image_time > 0:
                interval = current_time - self.last_image_time
                print(f"图像间隔: {interval:.1f} 秒")
            self.last_image_time = current_time
            
        except Exception as e:
            print(f"保存图像错误: {e}")
        finally:
            # 重置状态
            self.current_image = None
            self.expected_chunks = 0
            self.received_chunks = 0
            self.total_size = 0
            self.chunk_data = {}

def main():
    """主函数"""
    print("ESP32 UDP图像接收器")
    print("=" * 40)
    
    receiver = ImageReceiver()
    
    try:
        receiver.start()
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"程序错误: {e}")
    finally:
        print("程序结束")

if __name__ == "__main__":
    main()