# CLAUDE.md

请始终使用简体中文与我对话，并在回答时保持专业、简洁。

本文件为 Claude Code (claude.ai/code) 在本仓库中工作时提供指导。

## 项目概述

ESP32-S3 WiFi 相机应用，集成了**坐姿检测模型推理**。运行于 ESP32-S3-EYE 开发板，支持摄像头采集、I2S 音频播放（MAX98357 DAC）和双模式 WiFi（SoftAP + Station）。

核心功能流程：
摄像头采集图像 → 模型推理（7关键点检测）→ 姿态判断（眼睛-肩膀距离比）→ 音频提示音播放

## 构建命令

```bash
idf.py build              # 编译项目
idf.py -p PORT flash     # 烧录到设备（将 PORT 替换为串口名）
idf.py -p PORT monitor   # 打开串口监视器
idf.py set-target esp32s3 # 设置目标芯片
idf.py menuconfig         # 打开 Kconfig 菜单
```

## 架构

```
app_main.c                    # 入口点 - 初始化所有子系统
├── camera_init()              # 摄像头（通过 esp32-camera 组件）
├── wifi_manager_init()        # WiFi 硬件 + AP/STA 模式
│   ├── wifi_init_softap()     # SoftAP，SSID 基于 MAC 地址（esp32cam_XXXXXX）
│   └── wifi_init_sta()        # Station 模式（编译时配置或 NVS 存储的凭据）
├── wifi_config_manager_init() # 用于 WiFi 配置的强制门户（captive portal）
├── led_init()                 # 状态 LED（GPIO2，低电平有效）
├── audio_player_init()        # I2S 音频（GPIO19/20/47，16kHz，MAX98357）
├── posture_model_init()       # ESP-DL 模型初始化（从 rodata 加载）
├── posture_inference_task()   # 模型推理任务（摄像头→推理→姿态判断→音频提示）
├── dns_server.c               # DNS 重定向用于强制门户
└── start_udp_camera()         # UDP 图像传输任务（独立运行）
    └── send_image_via_udp()   # 发送到 UDP_SERVER_IP:8080，音频在 :8081
```

### 双 WiFi 模式

- **SoftAP**：创建 SSID 为 `esp32cam_XXXXXX`（基于 MAC 地址）的 AP
- **Station**：通过编译时 SSID 或 NVS 存储的凭据连接到指定 AP
- **NAPT**：在 SoftAP 上启用，将 STA 流量路由通过 AP

### UDP 协议

- **端口 8080**：摄像头帧数据（分包传输，每包最大 1400 字节）
- **端口 8081**：来自 PC 的音频流（8-bit PCM）
- 目标 IP：`UDP_SERVER_IP`（默认 `192.168.5.3`，可在 `udp_camera_client.c` 中修改）

### 音频播放

- I2S 标准模式，16kHz 采样率，单声道 16-bit
- GPIO：BCLK=19, WS=20, DOUT=47
- 播放 WiFi 状态提示音（连接成功/失败/重置），音频数据位于 `res/wifi_*.c`

### 坐姿检测模型

**参考官方示例：** `ref/esp-dl-master/examples/yolo11_pose`

模型文件：`model/litepose_esp32s3_test.espdl`

**模型加载方式：** 嵌入 rodata（`MODEL_LOCATION_IN_FLASH_RODATA`）
- 模型文件通过 `EMBED_FILES` 嵌入 app 二进制
- 分区表：`partitions.csv`（16MB Flash，factory=4MB）
- 优点：简单，修改模型需重烧 app

**输入：**
- 形状：[1, 3, 240, 320]（batch=1, RGB, 高240, 宽320）
- 预处理：JPEG 解码（`dl::image::sw_decode_jpeg`），HWC→CHW 转换

**输出：**
- 形状：[1, 7, 60, 80]（7 个关键点的热力图，每个 60×80 像素）
- 值范围：[0, 1]（Sigmoid 输出）

**7 个关键点顺序：** 0=左眼, 1=右眼, 2=左耳, 3=右耳, 4=鼻子, 5=左肩, 6=右肩

**姿态判断逻辑：** 计算眼睛连线与肩膀连线的距离比值，判断是否偏离正常范围（0.3~0.8），超阈值时播放提示音。

### 主要文件

| 文件                           | 功能                               |
| ------------------------------ | ---------------------------------- |
| `main/app_main.c`            | 入口点，初始化顺序                 |
| `main/cam.c`                 | 通过 esp32-camera 驱动初始化摄像头 |
| `main/posture_model.h`        | 坐姿模型头文件（宏定义、接口声明） |
| `main/posture_model.cpp`      | ESP-DL 模型加载、推理、姿态判断   |
| `main/wifi_manager.c`        | WiFi AP/STA 初始化和事件处理       |
| `main/wifi_config_manager.c` | 强制门户，NVS 凭据存储             |
| `main/udp_camera_client.c`   | UDP 图像/音频发送和接收任务        |
| `main/audio_player.c`        | I2S 播放（状态提示音和音频流）     |
| `main/led.c`                 | LED 呼吸/闪烁模式                  |
| `main/dns_server.c`          | DNS 服务器用于强制门户重定向       |
| `res/wifi_*.c`               | 内嵌的音频数据（WiFi 状态提示音）  |
| `model/litepose_esp32s3_test.espdl` | 量化后的坐姿检测模型（7关键点） |
| `sdkconfig.defaults`         | 默认配置（PSRAM、摄像头型号）      |
| `ref/esp-dl-master/examples/yolo11_pose` | ESP-DL 官方姿态检测示例 |
| `ref/how_to_load_test_profile_model.rst` | ESP-DL 模型部署官方文档 |

### 开发板配置

`CAMERA_MODEL_ESP32S3_EYE` 在 `sdkconfig.defaults` 中定义。摄像头引脚（XCLK=GPIO15, SIOD=GPIO4, SIOC=GPIO5 等）在 `main/camera_app.h` 中定义。

### 内存限制（重要）

- **IRAM（TCM，16KB）**：极度紧张，99.99% 已用，几乎无法添加新 IRAM 代码
- **PSRAM（DIRAM，341KB）**：充裕，约 232KB 可用，用于模型参数和中间结果
- **模型参数策略**：使用 `param_copy=false` 将参数保留在 Flash，减少 PSRAM 占用
- **推理必须在 PSRAM 上分配内存**，避免使用 IRAM

### LED 状态

- 熄灭：正常空闲
- 快闪：WiFi 配置模式 / 连接失败
- 呼吸：UDP 相机正在传输 / 姿态检测运行中
- 常亮：摄像头初始化失败
