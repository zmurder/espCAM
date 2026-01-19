#!/usr/bin/env python3
"""
PCM转C数组工具
将PCM文件转换为C语言数组，用于嵌入ESP32代码

使用方法:
    python pcm_to_array.py input.pcm array_name output.c

依赖:
    无（仅使用标准库）
"""

import sys
import os

def pcm_to_c_array(pcm_file, array_name, output_file=None, bytes_per_line=16):
    """
    将PCM文件转换为C数组

    Args:
        pcm_file: 输入PCM文件路径
        array_name: C数组名称
        output_file: 输出C文件路径（可选，默认打印到屏幕）
        bytes_per_line: 每行显示的字节数
    """
    try:
        # 读取PCM文件
        with open(pcm_file, 'rb') as f:
            data = f.read()

        size = len(data)

        # 生成C数组
        hex_values = [f'0x{b:02X}' for b in data]

        # 格式化为多行
        lines = []
        for i in range(0, len(hex_values), bytes_per_line):
            chunk = hex_values[i:i + bytes_per_line]
            line = '    ' + ', '.join(chunk)
            if i + bytes_per_line < len(hex_values):
                line += ','
            lines.append(line)

        # 生成完整的C代码
        c_code = f"""// Auto-generated audio data from {os.path.basename(pcm_file)}
// Size: {size} bytes ({size / 1024:.2f} KB)
// Format: 8-bit PCM, 16000Hz, Mono

static const uint8_t {array_name}[] = {{
{chr(10).join(lines)}
}};

#define {array_name.upper()}_LEN {size}
"""

        # 输出到文件或屏幕
        if output_file:
            with open(output_file, 'w', encoding='utf-8') as f:
                f.write(c_code)
            print(f"✅ C数组已生成: {output_file}")
            print(f"   数组名称: {array_name}")
            print(f"   数组大小: {size} 字节 ({size / 1024:.2f} KB)")
        else:
            print(c_code)

        return size

    except FileNotFoundError:
        print(f"❌ 错误: 文件不存在: {pcm_file}")
        return None
    except Exception as e:
        print(f"❌ 转换失败: {e}")
        return None

def main():
    if len(sys.argv) < 3:
        print("PCM转C数组工具")
        print("\n使用方法:")
        print("  python pcm_to_array.py <input.pcm> <array_name> [output.c]")
        print("\n示例:")
        print("  python pcm_to_array.py wifi_connected.pcm wifi_connected_audio")
        print("  python pcm_to_array.py wifi_connected.pcm wifi_connected_audio wifi_connected.c")
        sys.exit(1)

    pcm_file = sys.argv[1]
    array_name = sys.argv[2]
    output_file = sys.argv[3] if len(sys.argv) > 3 else None

    if not os.path.exists(pcm_file):
        print(f"❌ 错误: PCM文件不存在: {pcm_file}")
        sys.exit(1)

    # 转换文件
    size = pcm_to_c_array(pcm_file, array_name, output_file)

    if size is None:
        sys.exit(1)

if __name__ == "__main__":
    main()