#!/usr/bin/env python3
"""
MP3转8位PCM转换工具
将MP3音频文件转换为ESP32可播放的8位PCM格式

使用方法:
    python convert_audio.py input.mp3 output.pcm

依赖:
    pip install numpy
    ffmpeg (需要添加到系统PATH)
"""

import sys
import os
import subprocess
import numpy as np

def convert_mp3_to_pcm(input_file, output_file, sample_rate=16000):
    """
    将MP3文件转换为8位PCM格式

    Args:
        input_file: 输入MP3文件路径
        output_file: 输出PCM文件路径
        sample_rate: 目标采样率（默认16000Hz）
    """
    try:
        print(f"正在读取文件: {input_file}")

        # 使用 ffmpeg 将音频转换为 16位有符号PCM单声道数据
        # -y: 覆盖输出文件
        # -i: 输入文件
        # -ac 1: 单声道
        # -ar: 采样率
        # -f s16le: 16位小端序PCM格式
        # pipe:1: 输出到stdout
        cmd = [
            'ffmpeg', '-y',
            '-i', input_file,
            '-ac', '1',
            '-ar', str(sample_rate),
            '-f', 's16le',
            'pipe:1'
        ]

        print(f"正在转换音频...")
        print(f"  采样率: {sample_rate} Hz")
        print(f"  声道: 单声道")

        # 运行 ffmpeg 并获取输出
        result = subprocess.run(cmd, capture_output=True)

        if result.returncode != 0:
            print(f"❌ ffmpeg 执行失败:")
            print(result.stderr.decode('utf-8', errors='ignore'))
            return None

        # 获取原始音频数据（16位有符号PCM）
        raw_data = result.stdout

        # 转换为16位有符号整数数组
        samples_16bit = np.frombuffer(raw_data, dtype=np.int16)

        # 转换为8位无符号PCM（0-255）
        # 先归一化到 [-1, 1]，然后映射到 [0, 255]
        samples_float = samples_16bit.astype(np.float32) / 32768.0
        samples_8bit = ((samples_float * 127.0) + 128.0).astype(np.uint8)

        # 写入PCM文件
        with open(output_file, 'wb') as f:
            f.write(samples_8bit.tobytes())

        print(f"\n✅ 转换完成!")
        print(f"输出文件: {output_file}")
        print(f"文件大小: {len(samples_8bit)} 字节 ({len(samples_8bit) / 1024:.2f} KB)")
        print(f"时长: {len(samples_8bit) / sample_rate:.2f} 秒")
        print(f"采样率: {sample_rate} Hz")
        print(f"声道: 单声道")
        print(f"位深: 8位")

        return len(samples_8bit)

    except FileNotFoundError:
        print("❌ 错误: 找不到 ffmpeg")
        print("请确保 ffmpeg 已安装并添加到系统 PATH")
        print("下载地址: https://ffmpeg.org/download.html")
        return None
    except Exception as e:
        print(f"❌ 转换失败: {e}")
        import traceback
        traceback.print_exc()
        return None

def main():
    if len(sys.argv) < 3:
        print("MP3转8位PCM转换工具")
        print("\n使用方法:")
        print("  python convert_audio.py <input.mp3> <output.pcm> [sample_rate]")
        print("\n示例:")
        print("  python convert_audio.py wifi_connected.mp3 wifi_connected.pcm")
        print("  python convert_audio.py wifi_connected.mp3 wifi_connected.pcm 16000")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]
    sample_rate = int(sys.argv[3]) if len(sys.argv) > 3 else 16000

    if not os.path.exists(input_file):
        print(f"❌ 错误: 输入文件不存在: {input_file}")
        sys.exit(1)

    # 转换文件
    size = convert_mp3_to_pcm(input_file, output_file, sample_rate)

    if size is None:
        sys.exit(1)

if __name__ == "__main__":
    main()