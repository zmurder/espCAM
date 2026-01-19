@echo off
chcp 65001 >nul
echo ==================================================
echo ffmpeg 环境变量配置脚本
echo ==================================================
echo.

REM 检查是否已安装ffmpeg
if exist "C:\ffmpeg\bin\ffmpeg.exe" (
    echo ✓ 检测到ffmpeg已安装在 C:\ffmpeg\bin
    goto :check_path
) else (
    echo ✗ 未检测到ffmpeg
    echo.
    echo 请先下载并安装ffmpeg:
    echo 1. 访问 https://www.gyan.dev/ffmpeg/builds/
    echo 2. 下载 ffmpeg-release-essentials.zip
    echo 3. 解压后将 bin 文件夹复制到 C:\ffmpeg\bin
    echo.
    pause
    exit /b 1
)

:check_path
echo.
echo 正在检查PATH环境变量...

REM 检查是否已在PATH中
reg query "HKLM\SYSTEM\CurrentControlSet\Control\Session Manager\Environment" /v Path | findstr /i "C:\ffmpeg\bin" >nul
if %errorlevel% == 0 (
    echo ✓ ffmpeg已在PATH环境变量中
    echo.
    goto :verify
) else (
    echo ffmpeg尚未添加到PATH环境变量
    echo.
    goto :add_path
)

:add_path
echo 正在添加ffmpeg到PATH环境变量...

REM 获取当前PATH
for /f "tokens=2*" %%A in ('reg query "HKLM\SYSTEM\CurrentControlSet\Control\Session Manager\Environment" /v Path ^| findstr Path') do set "CURRENT_PATH=%%B"

REM 添加ffmpeg路径
set "NEW_PATH=%CURRENT_PATH%;C:\ffmpeg\bin"

REM 写入注册表
reg add "HKLM\SYSTEM\CurrentControlSet\Control\Session Manager\Environment" /v Path /t REG_EXPAND_SZ /d "%NEW_PATH%" /f >nul

REM 通知系统环境变量已更改
setx PATH "%PATH%" >nul

echo ✓ 已成功添加 C:\ffmpeg\bin 到系统PATH
echo.
echo ==================================================
echo 重要提示: 请重新打开所有终端窗口以使更改生效
echo ==================================================
echo.
goto :verify

:verify
echo 正在验证ffmpeg安装...
echo.

REM 尝试运行ffmpeg
C:\ffmpeg\bin\ffmpeg.exe -version >nul 2>&1
if %errorlevel% == 0 (
    echo ✓ ffmpeg安装成功并可用!
    echo.
    C:\ffmpeg\bin\ffmpeg.exe -version | findstr /i "ffmpeg version"
) else (
    echo ✗ ffmpeg验证失败
    echo 请检查安装是否正确
)

echo.
echo ==================================================
echo 配置完成!
echo ==================================================
echo.
echo 使用说明:
echo 1. 关闭当前终端窗口
echo 2. 重新打开终端
echo 3. 运行: ffmpeg -version
echo 4. 运行: python convert_audio.py input.mp3 output.pcm
echo.
pause