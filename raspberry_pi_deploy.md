# 树莓派部署指南

接收器（simple_udp_receiver.py + web_ui.py）为纯 Python 跨平台程序，可长期运行在树莓派上：
收图/落库/OTA 自动检测/Web 控制台全部可用，配合 systemd 实现**开机自启 + 崩溃自动拉起**。

## 1. 依赖与文件

```bash
sudo apt install python3-pil        # 唯一第三方依赖 Pillow（web_ui 为纯标准库）
mkdir -p ~/code/esp32_cam && cd ~/code/esp32_cam
```

从 PC 拷入（scp/WinSCP 均可）：

| 文件 | 必须 | 说明 |
|---|---|---|
| `simple_udp_receiver.py` | ✅ | 接收器主程序 |
| `web_ui.py` | ✅ | Web 控制台（主程序 import 它，缺了起不来） |
| `posture_history.db` | 可选 | 想延续 PC 上的历史统计就一并拷来，否则首帧自动新建 |
| `build/espCAM_WIFI.bin` | 按需 | OTA 固件：放 `ota/` 目录（见第 6 节） |

**数据文件都落在启动时的工作目录**：`posture_history.db`、`received_images/<设备ID>/`、
`sched_per_dev.json`、`recv_settings.json`、`ota/`——换目录=换一份数据。

## 2. 安装 systemd 服务

仓库里的 `espcam.service` 是服务配置文件（内容：用户/工作目录/启动命令/崩溃 5s 重启）。
先确认其中 `User=` 与 `WorkingDirectory=` 与实际一致，然后：

```bash
sudo cp espcam.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now espcam    # 注册开机自启 + 立即启动
```

## 3. 开机自启管理

| 命令 | 作用 |
|---|---|
| `sudo systemctl enable espcam` | **开启**开机自启（不影响当前运行） |
| `sudo systemctl disable espcam` | **关闭**开机自启（重启后不再自动启动，不停当前运行） |
| `sudo systemctl enable --now espcam` | 开启自启并立即启动 |
| `sudo systemctl disable --now espcam` | 关闭自启并立即停止 |
| `systemctl is-enabled espcam` | 查询自启状态（enabled / disabled） |

注意：`stop` 只停当前运行（重启后仍会自启）；`disable` 只关自启（不停当前运行）——两者独立。

## 4. 日常运维

```bash
systemctl status espcam         # 运行状态
journalctl -u espcam -f         # 实时日志（Ctrl+C 退出）
journalctl -u espcam --since today   # 今天的日志
sudo systemctl restart espcam   # 更新代码后重启
sudo systemctl stop espcam      # 停止
```

systemd 下 stdin 不可用，接收器自动进入**无交互模式**：收图/落库/OTA 自动检测照常，
键盘命令不可用——全部功能由 Web 控制台覆盖：`http://<树莓派IP>:20005`。

## 5. 网络

- 树莓派与 ESP32 需**同一子网**（设备发现靠 UDP 广播：受限广播 + /24 定向广播）
- 需放行端口：UDP 20000–20003（图/结果/发现）、TCP 20004（OTA 固件下载）、TCP 20005（Web）
- 若启用 ufw：`sudo ufw allow 20000:20005/udp && sudo ufw allow 20004:20005/tcp`

## 6. OTA 固件升级（树莓派上的注意点）

固件仍在 PC 上编译（`idf.py build`），树莓派只做分发：

```bash
# PC：编译后拷到树莓派
scp build/espCAM_WIFI.bin zyd@<树莓派IP>:/home/zyd/code/esp32_cam/ota/
```

然后浏览器打开 Web 控制台点"升级"即可（接收器的 `ota` 命令会尝试从本机 `build/` 拷贝，
树莓派上没有该目录——直接放 `ota/` 就绕过了这一步，自动检测对比 sha8 照常工作）。

## 7. 更新接收器代码

```bash
# PC 拷入新的 .py 后
sudo systemctl restart espcam
journalctl -u espcam -f         # 确认正常启动
```
