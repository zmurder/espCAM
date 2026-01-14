| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C6 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- |

# ESP32 WiFi Camera with UDP Image Transmission

This project demonstrates an ESP32 camera system that:
- Acts as both WiFi Access Point and Station simultaneously
- Captures images from the camera
- Transmits images via UDP to a remote PC server
- Supports NAT routing functionality

## Features
- **Dual WiFi Mode**: SoftAP (creates hotspot) + Station (connects to external network)
- **Camera Support**: Compatible with ESP32CAM-AITHINKER board (OV3660 sensor)
- **UDP Image Transmission**: Sends JPEG images to PC server every 5 seconds
- **分包传输**: Supports packet transmission to handle MTU limitations
- **NAT Router**: Can be used as WiFi NAT router with NAPT enabled

## How to use example
### Configure the project

You can configure the WiFi credentials in two ways:

#### Method 1: Edit sdkconfig.defaults (Recommended)
For frequently changed settings like WiFi credentials, it's recommended to edit the `sdkconfig.defaults` file directly:

```bash
# Edit the WiFi configuration in sdkconfig.defaults
CONFIG_ESP_WIFI_AP_SSID="your_ap_ssid"
CONFIG_ESP_WIFI_AP_PASSWORD="your_ap_password"
CONFIG_ESP_WIFI_REMOTE_AP_SSID="your_wifi_ssid"
CONFIG_ESP_WIFI_REMOTE_AP_PASSWORD="your_wifi_password"
```

#### UDP Server Setup
This ESP32 project acts as a UDP client that sends images to a PC server. You need to set up a UDP server on your PC to receive the images:

1. **UDP Server IP**: Set the target PC IP address in `main/udp_camera_client.c` line 14:
   ```c
   #define UDP_SERVER_IP "192.168.5.3"  // 替换为你的PC的IP地址
   ```

2. **UDP Server Port**: Default port is 8080 (line 15 in udp_camera_client.c)

3. **PC Server Software**: You can use any UDP server application or write a simple Python script to receive the images.

### Python UDP图像接收器

本项目包含两个Python接收器程序：

#### 1. 简单接收器 (`simple_udp_receiver.py`)
适合快速测试，功能简单直接：

```bash
# 运行简单接收器
python simple_udp_receiver.py
```

特点：
- 监听所有网络接口的8080端口
- 自动创建`received_images`目录保存图像
- 显示详细的接收进度信息
- 支持JPEG格式验证

#### 2. 高级接收器 (`udp_image_receiver.py`)
功能完整，支持分包处理和错误恢复：

```bash
# 运行高级接收器
python udp_image_receiver.py
```

特点：
- 完整的分包传输处理
- 图像完整性验证
- 自动重组分包数据
- 详细的错误处理和日志
- 计算图像接收间隔

### 使用步骤

1. **确认PC IP地址**：
   ```bash
   # Windows
   ipconfig
   
   # Linux/Mac
   ifconfig
   ```

2. **修改ESP32代码中的目标IP**：
   编辑 `main/udp_camera_client.c` 第14行：
   ```c
   #define UDP_SERVER_IP "192.168.1.100"  // 替换为你的PC IP地址
   ```

3. **启动Python接收器**：
   ```bash
   python simple_udp_receiver.py
   # 或
   python udp_image_receiver.py
   ```

4. **烧录并运行ESP32**：
   ```bash
   idf.py -p COM3 flash monitor  # COM3替换为你的串口
   ```

5. **查看接收的图像**：
   - 简单接收器：`received_images/` 目录
   - 高级接收器：`received_images/` 目录

#### Method 2: Use menuconfig
Open the project configuration menu (`idf.py menuconfig`).

In the `Example Configuration` menu:

* Set the Wi-Fi SoftAP configuration.
    * Set `WiFi AP SSID`.
    * Set `WiFi AP Password`.

* Set the Wi-Fi STA configuration.
    * Set `WiFi Remote AP SSID`.
    * Set `WiFi Remote AP Password`.

Optional: If necessary, modify the other choices to suit your needs.

### Build and Flash

Build the project and flash it to the board, then run the monitor tool to view the serial output:

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

## Example Output

There is the console output for this example:

```
I (680) WiFi SoftAP: ESP_WIFI_MODE_AP
I (690) WiFi SoftAP: wifi_init_softap finished. SSID:espCAM_WIFI password:12345678 channel:1
I (690) WiFi Sta: ESP_WIFI_MODE_STA
I (690) WiFi Sta: wifi_init_sta finished.
I (700) phy_init: phy_version 4670,719f9f6,Feb 18 2021,17:07:07
I (800) wifi:mode : sta (58:bf:25:e0:41:00) + softAP (58:bf:25:e0:41:01)
I (800) wifi:enable tsf
I (810) wifi:Total power save buffer number: 16
I (810) wifi:Init max length of beacon: 752/752
I (810) wifi:Init max length of beacon: 752/752
I (820) WiFi Sta: Station started
I (820) wifi:new:<1,1>, old:<1,1>, ap:<1,1>, sta:<1,1>, prof:1
I (820) wifi:state: init -> auth (b0)
I (830) wifi:state: auth -> assoc (0)
E (840) wifi:Association refused temporarily, comeback time 1536 mSec
I (2380) wifi:state: assoc -> assoc (0)
I (2390) wifi:state: assoc -> run (10)
I (2400) wifi:connected with ZTE-Zhao, aid = 1, channel 1, 40U, bssid = 84:f7:03:60:86:1d
I (2400) wifi:security: WPA2-PSK, phy: bgn, rssi: -14
I (2410) wifi:pm start, type: 1

I (2410) wifi:AP's beacon interval = 102400 us, DTIM period = 2
I (3920) WiFi Sta: Got IP:192.168.5.2
I (3920) esp_netif_handlers: sta ip: 192.168.5.2, mask: 255.255.255.0, gw: 192.168.5.1
I (3920) WiFi Sta: connected to ap SSID:ZTE-Zhao password:zyd83210
I (4520) camera: Camera Init Success
I (4520) UDP_CAMERA: 捕获并发送图像...
I (4530) camera: Camera Capture Success. Frame size: 5324 bytes
I (4530) UDP_CAMERA: 开始发送图像，大小: 5324 bytes, 分 4 包
I (4540) UDP_CAMERA: 图像发送成功
```

## Troubleshooting

For any technical queries, please open an [issue](https://github.com/espressif/esp-idf/issues) on GitHub. We will get back to you soon.
