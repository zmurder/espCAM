---
name: ESP32-S3 WiFi Camera Project
description: ESP32-S3 双WiFi模式（AP+STA）相机应用，通过UDP传输视频流到PC
type: project
---

# 项目概述
- ESP32-S3 WiFi 相机，通过 UDP 向 PC 传输视频流
- 开发板：ESP32-S3-EYE
- 音频：I2S MAX98357 DAC，16kHz 采样
- 编译命令：`idf.py build`

## 架构
- `main/app_main.c` - 入口，初始化各子系统
- `main/udp_camera_client.c` - UDP 图像发送，端口 8080；音频接收，端口 8081
- `main/wifi_manager.c` - WiFi AP+STA 双模式
- `main/wifi_config_manager.c` - Captive portal 配网
- `main/audio_player.c` - I2S 音频播放
- `main/cam.c` - 摄像头初始化

## 内存问题（重要）
- IRAM（16KB）使用率 99.99%，几乎已满
- 已关闭的 IRAM 优化项：ESP_WIFI_IRAM_OPT、ESP_WIFI_RX_IRAM_OPT、GPTIMER_ISR_HANDLER_IN_IRAM、SPI_MASTER_ISR_IN_IRAM、SPI_SLAVE_ISR_IN_IRAM、GDMA_ISR_HANDLER_IN_IRAM
- IRAM 主要被 IDF 核心占用（中断向量表、WiFi 基础代码、esp_timer 等），应用层优化空间极小
- DIRAM（PSRAM，341KB）使用率约 32%，充裕
- PSRAM 实际总大小 8MB

## UDP 配置
- 目标 IP：`192.168.5.3`（`UDP_SERVER_IP` 在 udp_camera_client.c:21）
- 图像端口：8080
- 音频端口：8081
- 每个 UDP 包最大 1400 字节（MTU 限制）
- 包头格式：12 字节 (chunk_id + total_chunks + image_size，均为 big-endian uint32)

## 上位机接收脚本
- `simple_udp_receiver.py` - 简单版，适合快速测试
- `udp_image_receiver.py` - 完整版，有丢包检测和进度显示
- 两个脚本协议完全兼容 ESP32 端

## 坐姿检测模型（已集成）
- 模型：LitePose，量化后的 .espdl 文件位于 `model/litepose_esp32s3_test.espdl`
- 输入：[1, 3, 240, 320] RGB，Letterbox Resize
- 输出：[1, 7, 60, 80] 7 关键点热力图
- 关键点：0=左眼, 1=右眼, 2=左耳, 3=右耳, 4=鼻子, 5=左肩, 6=右肩
- 姿态判断：眼睛距离/肩膀距离 比值
- IRAM 极度紧张，需使用 param_copy=false，推理在 PSRAM 上进行

## SDK 配置
- 目标芯片：esp32s3
- Flash：2MB
- PSRAM：8MB OCTAL SPIRAM
- CPU 频率：160MHz
- `sdkconfig` 存在，`sdkconfig.defaults` 是模板
