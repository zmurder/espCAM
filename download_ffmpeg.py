#!/usr/bin/env python3
"""
自动下载并安装ffmpeg的脚本
"""
import os
import sys
import zipfile
import subprocess
from urllib.request import urlretrieve

def download_ffmpeg():
    """下载ffmpeg"""
    print("正在下载ffmpeg...")
    
    # 使用国内镜像
    url = "https://www.gyan.dev/ffmpeg/builds/ffmpeg-release-essentials.zip"
    
    # 使用项目目录作为临时位置
    temp_file = os.path.join(os.getcwd(), "ffmpeg.zip")
    
    try:
        # 显示下载进度
        def report_progress(block_num, block_size, total_size):
            downloaded = block_num * block_size
            percent = min(100, (downloaded * 100) // total_size) if total_size > 0 else 0
            print(f"\r下载进度: {percent}%", end="", flush=True)
        
        urlretrieve(url, temp_file, reporthook=report_progress)
        print("\n下载完成!")
        return temp_file
    except Exception as e:
        print(f"\n下载失败: {e}")
        return None

def extract_ffmpeg(zip_file):
    """解压ffmpeg"""
    print("正在解压ffmpeg...")
    
    install_dir = "C:\\ffmpeg"
    
    try:
        with zipfile.ZipFile(zip_file, 'r') as zip_ref:
            # 解压到临时目录
            temp_extract = os.path.join(os.getcwd(), "ffmpeg_extract")
            zip_ref.extractall(temp_extract)
        
        # 找到bin目录
        for root, dirs, files in os.walk(temp_extract):
            if "ffmpeg.exe" in files and "bin" in root:
                # 复制到C:\ffmpeg
                os.makedirs(install_dir, exist_ok=True)
                import shutil
                shutil.copytree(root, os.path.join(install_dir, "bin"), dirs_exist_ok=True)
                break
        
        print(f"ffmpeg已安装到: {install_dir}")
        return True
    except Exception as e:
        print(f"解压失败: {e}")
        return False

def add_to_path():
    """添加到PATH环境变量"""
    print("正在添加到PATH环境变量...")
    
    ffmpeg_path = "C:\\ffmpeg\\bin"
    
    try:
        # 获取当前PATH
        import winreg
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, 
                           r"SYSTEM\CurrentControlSet\Control\Session Manager\Environment", 
                           0, winreg.KEY_READ)
        path, _ = winreg.QueryValueEx(key, "Path")
        winreg.CloseKey(key)
        
        # 检查是否已存在
        if ffmpeg_path not in path:
            # 添加到PATH
            new_path = path + ";" + ffmpeg_path
            
            # 写入注册表
            key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, 
                               r"SYSTEM\CurrentControlSet\Control\Session Manager\Environment", 
                               0, winreg.KEY_SET_VALUE)
            winreg.SetValueEx(key, "Path", 0, winreg.REG_EXPAND_SZ, new_path)
            winreg.CloseKey(key)
            
            # 通知系统环境变量已更改
            import ctypes
            HWND_BROADCAST = 0xFFFF
            WM_SETTINGCHANGE = 0x1A
            SMTO_ABORTIFHUNG = 0x0002
            result = ctypes.c_long()
            ctypes.windll.user32.SendMessageTimeoutW(
                HWND_BROADCAST, WM_SETTINGCHANGE, 0, "Environment", 
                SMTO_ABORTIFHUNG, 5000, ctypes.byref(result))
            
            print("已添加到PATH环境变量")
            print("请重新打开终端以使更改生效")
        else:
            print("ffmpeg已在PATH中")
        
        return True
    except Exception as e:
        print(f"添加PATH失败: {e}")
        print("您可以手动添加 C:\\ffmpeg\\bin 到系统PATH")
        return False

def verify_installation():
    """验证安装"""
    print("\n验证ffmpeg安装...")
    
    try:
        result = subprocess.run(["ffmpeg", "-version"], 
                              capture_output=True, 
                              text=True, 
                              timeout=5)
        if result.returncode == 0:
            print("✓ ffmpeg安装成功!")
            print(result.stdout.split('\n')[0])
            return True
        else:
            print("✗ ffmpeg安装失败")
            return False
    except Exception as e:
        print(f"验证失败: {e}")
        print("请重新打开终端后再试")
        return False

def main():
    """主函数"""
    print("=" * 50)
    print("ffmpeg自动安装脚本")
    print("=" * 50)
    
    # 下载
    zip_file = download_ffmpeg()
    if not zip_file:
        print("\n下载失败，请手动下载:")
        print("https://www.gyan.dev/ffmpeg/builds/ffmpeg-release-essentials.zip")
        print("并解压到 C:\\ffmpeg")
        return False
    
    # 解压
    if not extract_ffmpeg(zip_file):
        return False
    
    # 添加到PATH
    if not add_to_path():
        return False
    
    # 验证
    verify_installation()
    
    print("\n安装完成!")
    print("注意: 请重新打开终端以使PATH环境变量生效")

if __name__ == "__main__":
    main()