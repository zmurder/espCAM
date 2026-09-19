# CLAUDE.md

请始终使用简体中文与我对话，并在回答时保持专业、简洁。

本文件为 Claude Code (claude.ai/code) 在本仓库中工作时提供指导。

## 项目概述

ESP32-S3 WiFi 相机应用，集成了**坐姿检测模型推理**。运行于 ESP32-S3-EYE 开发板，支持摄像头采集、I2S 音频播放（MAX98357 DAC）和双模式 WiFi（SoftAP + Station）。

核心功能流程：
摄像头采集图像 → 模型推理（6关键点检测：双眼/双耳/双肩）→ 姿态判断（双肩倾斜 + 眼肩前倾比）→ 音频提示音播放

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
    └── send_image_via_udp()   # 发送到 UDP_SERVER_IP:20000，音频在 :20001
```

### 双 WiFi 模式

- **SoftAP**：创建 SSID 为 `esp32cam_XXXXXX`（基于 MAC 地址）的 AP
- **Station**：通过编译时 SSID 或 NVS 存储的凭据连接到指定 AP
- **NAPT**：在 SoftAP 上启用，将 STA 流量路由通过 AP

### UDP 协议

- **端口 20000**：摄像头帧数据（分包传输，每包最大 1400 字节）
- **端口 20001**：来自 PC 的音频流（8-bit PCM）
- **端口 20002**：检测结果（result + ratio + 6 关键点，78 字节）
- **端口 20003**：自动发现 + 检测时间段调度 + 提醒模式设置 + OTA 触发（文本协议，与 `udp_discovery_task` 对应）
  - `ESPCAM_DISCOVER` → ESP 学习源 IP 并回 `ESPCAM_ACK <设备ID> <固件版本号> <elf_sha8> <提醒模式>`（设备ID=WiFi MAC 后 3 字节 hex；PC 解析进**设备表**逐台显示版本、对比 ota/ 固件 sha 自动触发升级；第 5 段提醒模式 0|1 供 Web 设备表"提醒"列展示，旧固件无此段 PC 显示"—"；旧 PC 只看前 4 段不受影响）；PC 端每 2s 广播（双保险：受限广播 + /24 定向广播），PC IP 变化后 2s 内自动跟随
  - `ESPCAM_SCHED_GET` → ESP 回 `ESPCAM_SCHED_STATE synced active n HH:MM HH:MM ...`（当前调度状态；PC 端状态变化立即打印、不变则 60s 一条心跳，行尾附 ESP 固件版本）
  - `ESPCAM_SCHED_SET n HH:MM HH:MM ...` → 设置检测时段（n≤5，n=0 清空），ESP 存 NVS 并回 SCHED_STATE 确认；解析失败回 `ESPCAM_SCHED_ERR 原因`；PC 端可**按设备**设置：`s 09:00-11:30,14:00-18:00` / `s clear` 广播全部设备，`s @<序号|ID> ...` 单播指定设备（与 ota 命令同一套设备引用；ESP 端各设备 NVS 独立，本就支持每台不同时段）。记忆文件 `sched_per_dev.json`（default=全部统一 + devices=各台专属）：手动"全部"会统一并清掉专属；启动时 default 广播一次、有专属的设备 ACK 后单播覆盖（重启/重新上线自动恢复）；无记忆文件时 default 取 `POSTURE_SCHED_SLOTS` 常量（旧语义）；所有下发均本地预校验。SCHED_STATE 的时段/时间同步状态存入设备表（Web 设备列表展示），`[sched]`/`[result]`/`[image]` 日志行带 `[devid]` 前缀（按源 IP 归属，多设备不混淆）
  - `ESPCAM_OTA_START http://IP:端口/espCAM_WIFI.bin` → ESP 回 `ESPCAM_OTA_STARTED` 并启动下载任务，下载中每 256KB 推 `ESPCAM_OTA_PROGRESS <已收字节> <总字节>`（PC 打印百分比进度）；完成后回 `ESPCAM_OTA_DONE` 重启进新固件；失败回 `ESPCAM_OTA_ERR 原因`（当前固件不受影响）；重复触发回 `ESPCAM_OTA_ERR busy`（防重入，正常）。URL 由 PC 填本机当前 IP，**IP 动态无需固定**；PC 已学到 ESP IP 时**单播一份**（双保险广播两份曾致 ESP 竞态创建两个下载任务写坏镜像，ESP 端置位提前到建任务前已根治）
  - `ESPCAM_ALERT_SET <0|1>` → 设置不良提醒模式（1=连续播，0=只播一次），ESP 写 NVS 并回 `ESPCAM_ALERT_STATE <0|1>` 确认；`ESPCAM_ALERT_GET` → 查询回同格式 STATE；PC 端命令 `alert` 查看 / `alert repeat|once` 广播全部 / `alert @<序号|ID> repeat|once` 单播指定设备（设备引用与 ota/s 同一套）；记忆并入 `sched_per_dev.json`（alert_default + alert_devices，语义同时段：全部=统一清专属、单台=专属覆盖；启动 default 广播一次、专属设备 ACK 后补发）。存储于 `posture_sched.c`（NVS psched/alert_rep，默认连续播）
  - SNTP 对时成功时 ESP 主动向已学习 IP 推送一次 SCHED_STATE（**不能在 SNTP 回调里直接 sendto**——回调运行在 lwIP tcpip_thread 上下文，自等待会永久死锁挂起之后所有 socket 操作；正确做法：回调只置标志 `time_sync_pop_event()`，由 20003 任务在收包循环顶部轮询取走后调 `udp_sched_push_state()`，该 socket 设 1s 接收超时保证无流量时轮询仍运行）
- 目标 IP：`UDP_SERVER_IP`（`udp_camera_client.c`）仅为编译期默认值，运行时由 20003 发现机制自动覆盖

### 坐姿历史记录（PC 端）

- `simple_udp_receiver.py` 把每帧检测结果（时间戳 + result + ratio + devid，kps 不入库）逐帧写入 SQLite：`posture_history.db` 表 `posture_log(ts, result, ratio, devid)`；devid 按结果包源 IP 归属设备（多设备分开统计），旧库自动 ALTER 迁移（旧行 devid=NULL 只计入"全部设备"）；自动建库，写库失败不影响接收
- `posture_report.py` 查询该库出报告：`python posture_report.py day|week|month|quarter|year`（中文同义：天/周/月/季度/年；可选 `--db 库路径` `--no-show`）
  - 窗口=自然周期（今日/本周一/本月 1 日/季度首月/1 月 1 日）~ 现在；分桶：day→按小时、week/month→按天、quarter→按周、year→按月
  - 输出单张并列柱形图（PNG + 交互窗口）：每桶检测次数（蓝）+ 不良次数（橙）
  - 控制台统计：检测/不良帧数、不良率（不良帧/有效判断帧，NOT_DET/UNRELIABLE/TOO_FAR 不计入分母）、不良事件次数与最长持续（连续 BAD 间隔>5s 分段）

### OTA 固件升级 + 版本号（多设备）

- **分区表**（`partitions.csv`，16MB flash）：双 OTA 槽 ota_0/ota_1 各 5MB + otadata 8KB + nvs/phy + storage(fat) 约 5.9MB；OTA 写入另一槽，重启切换，坏镜像由 bootloader 拒绝并回滚
- **升级流程**：PC 接收器输入 `ota`（自动把 `build/espCAM_WIFI.bin` 拷入 `ota/`）→ 本地起 HTTP 服务（`:20004` 只共享 `ota/`）→ **单播** `ESPCAM_OTA_START` 到目标设备（URL 带本机当前 IP，**动态无需固定**；多设备下绝不广播——广播会同时打到所有设备）→ ESP 流式下载写另一 app 分区（`esp_ota_write` 首块校验镜像头、`esp_ota_end` 校验完整性）→ `esp_ota_set_boot_partition` + 重启
- **升级期间 ESP 暂停坐姿检测**（`udp_ota_in_progress()`）：不抢 WiFi 空口带宽、避免推理 flash 访问与擦写互斥，下载更快；重启进新固件自动恢复
- **多设备**：PC 维护设备表（devid → IP/版本/sha/失败计数/最近应答），`ota list` 列出在线设备（序号/ID/IP/版本/是否待升级）；**逐台串行升级**（一台确认/失败/超时再推下一台，避免同时下载挤占带宽），下载中每 256KB 推进度；自动检测按设备逐台对比 sha、60s 冷却、3 次熔断（`ota on` 重开）
- **命令**：`ota` 或 `ota all`（全部待升级设备）/ `ota list`（设备表）/ `ota <序号|ID前缀>`（指定设备）/ `ota on` / `ota off`（自动检测开关，默认开）；命令分发在 `_ota_cmd()`（input 交互与 Web UI 共用入口）；HTTP 服务对 ESP 重启/断开导致的连接中断只打一行提示（不打 traceback）
- **版本号**：`年月日_当日序号`（如 `20260916_1`），**手动维护**——发布新固件前改 `main/app_version.h` 的 `APP_VERSION` 宏（同日递增、跨日重置）；忘记改不会漏升级（自动检测依据是 elf_sha256，与版本号无关）
- **版本显示**：ESP 每条日志前缀 `[版本号]`（`esp_log_set_vprintf` 钩子，`app_main.c`，串口监视器可见）；PC 端从 `ESPCAM_ACK` 解析版本（`[version] <devid> 固件版本`），`[sched]` 状态行行尾附在线设备版本摘要
- 分区表从单 factory 改为双 OTA 后需重新 `idf.py flash`（app 首次烧入 ota_0）

### Web 控制台（PC 端，:20005）

- `web_ui.py`：内嵌于接收器进程的网页控制台（标准库 http.server，零依赖），接收器启动即自动开启；浏览器打开 `http://localhost:20005`（同局域网手机用 `http://<本机IP>:20005`）
- **功能**：设备表（ID/IP/固件/检测时段/时间同步/**提醒**/最新|待升级|离线，升级中那台显示下载进度条）+ 顶部 OTA 横幅（百分比/等待重启确认/排队数）+ 升级全部/逐台升级/自动检测开关 + "提醒默认"按钮（全部设备统一连续播/只播一次）；**坐姿统计**（选设备 × 时间范围 今日/本周/本月/今年/全部 → KPI 数字块 + 分桶柱形图 + 不良率折线 + 数据表，桶粒度：今日按小时、周/月按天、年/全部按月，口径与 posture_report.py 一致：不良率=不良帧/有效帧（桶级率随设备选择走：全部=合并、单台=该台）、不良事件按连续 BAD 间隔>5s 分段，60s 自动刷新+切换条件立即刷新）；检测时段设置（选全部设备=广播统一 / 选单台=只改它，等价 `s` / `s @ID` 命令，卡内显示各设备当前调度状态）；**图像保存设置**（保存开关与抽稀间隔——0=不保存、1=每帧、N=每 N 帧；关键点叠加开关；**仅有人帧**开关——该设备最近结果连续 50 帧无效（没人/太远/不可信）即停存空场景帧、恢复有人立即续存，PC 端判定不动 ESP；全部设备默认 + 按设备覆盖，等价 `img` 命令含 `img people on|off`，持久化 `recv_settings.json`）；**不良提醒模式**（设备表"提醒"列下拉选择 连续播/只播一次，等价 `alert @ID repeat|once`；旧固件显示 —）；运行日志面板（print 镜像环形缓冲 300 行，可按设备过滤——行含 `[devid]` 即匹配）
- **架构**：网页按钮 → `POST /api/cmd` → 接收器 `handle_command()`（与控制台键盘输入**完全同一条路径**，行为一致）；状态 → `GET /api/state`（`_web_state()` 快照）2s 轮询；统计 → `GET /api/stats?range=&device=`（`_web_stats()` 查历史库）60s 低频轮询；`web_ui.start(state_provider, cmd_dispatcher)` + `set_stats_provider()` 注入回调，与接收器解耦无循环依赖
- **图表规范**（dataviz）：两系列分组柱形图 + 不良率折线同图，配色 #2a78d6（检测）/#eb6834（不良）/#1baf7a（不良率曲线）——参考调色板前三槽，validate_palette.js 全项通过（worst adjacent CVD ΔE 9.2；#1baf7a 对表面 2.74:1 的 WARN 由数据表/tooltip/右侧刻度缓解）；柱厚≤24px、顶部 4px 圆角、组内 2px 间隙、发丝实线网格、图例常驻（曲线用线形标记）、率曲线固定 0-100% 满量程+右侧独立小刻度（不随计数轴缩放）、null 桶断线、2px 线+白描边圆点、悬停整组提亮+提示框（值为主）、"数据表"折叠表格为免悬停孪生、文本用墨色不用系列色
- 接收器原有键盘命令照常可用（`handle_command` 共用）；关掉网页不影响任何后台功能（收图/落库/OTA 自动检测）

### 音频播放

- I2S 标准模式，16kHz 采样率，单声道 16-bit
- GPIO：BCLK=19, WS=20, DOUT=47
- 播放 WiFi 状态提示音（连接成功/失败/重置），音频数据位于 `res/wifi_*.c`

### 坐姿检测模型

**模型文件：** `model/pose_model.espdl`（约 526KB，6 关键点，2026-09-19 更新；`main/CMakeLists.txt` 的 `MODEL_FILE_PATH` 指定，EMBED_FILES 嵌入 rodata；输入 exponent 由代码运行时从模型元数据自动读取，换模型无需改量化参数；旧版本模型已删除，需要时从 git 历史找回）

**模型规格（部署/改动指南详见 `model/esp32_deploy/model_deploy_update.md`）：**

| 属性 | 值 |
| :--- | :--- |
| 关键点数 | 6（双眼 / 双耳 / 双肩） |
| 参数量 | ~0.13M |
| 量化方式 | PTQ int8（对称量化，POWER_OF_2 exponent） |
| 输入形状 | [1, 240, 320, 3] (NHWC, RGB) |
| 输入 exponent | -6（当前模型；随导出可能变化，代码运行时自动读取） |
| 输出形状 | [1, 120, 160, 6] heatmap + Sigmoid |

**6 个关键点顺序：** 0=左眼, 1=右眼, 2=左耳, 3=右耳, 4=左肩, 5=右肩

**前处理（`dl::image::ImagePreprocessor`，与训练侧对齐）：**
- center crop：QVGA 320×240 已是 4:3，直接全图输入
- resize 到 240×320（ImagePreprocessor 按模型 input shape 自动处理）
- ImageNet 归一化：mean=[123.675, 116.28, 103.53]，std=[58.395, 57.12, 57.375]
- RGB（`rgb_swap=false`），HWC→CHW，按 input exponent 量化到 int8

**后处理（`main/posture_model.cpp`）：**
- heatmap 每通道 int8 argmax + 峰邻域正值加权质心（亚像素细化，±1 格 → ~±0.3 格，降低小目标定位噪声），`conf = max_int8 × 2^exponent`（Sigmoid 输出，已在 [0,1]）
- 输出 layout 按 `output_shape` 自适应（NHWC/NCHW 均支持，启动日志会打印 `chan_dim`）
- 置信度阈值：眼/耳 `CONF_THRESH=0.4`，双肩单独 `SHOULDER_CONF_THRESH=0.3`（趴近时肩 conf 偏低但仍需作基准）

**姿态判断逻辑（4 条标准任意成立即不良，阈值均为宏，见 `main/posture_model.cpp`）：**
- 可信前提：双肩均可见 && 双眼或双耳可见，否则 `POSTURE_NOT_DETECTED`
- 距离门限：双肩间距（归一化）< `SHOULDER_DIST_MIN`（默认 0.19）→ `POSTURE_TOO_FAR` 本帧不判断（人太远时 heatmap 定位噪声占比过大，远距离误报根源；阈值需据远距离日志标定）
- 几何合理性预检（防误检误报）：同组连线（双肩/双眼/双耳）|倾斜角| > `*_LINE_TILT_MAX`（默认 40°，近垂直属明显误检）→ 双肩误检或眼+耳全误检判 `POSTURE_UNRELIABLE`（本帧不判断、不播提示音）；仅一组头部误检则跳过该组条件、用另一组照常判断
- `POSTURE_BAD_NECK`（前倾）：眼肩垂直距离/双眼距 < `EYE_FORWARD_RATIO_MIN`，或 耳肩垂直距离/双耳距 < `EAR_FORWARD_RATIO_MIN`
- `POSTURE_BAD_SHOULDER`（歪头）：双眼-双肩相对倾斜角 > `EYE_HEAD_TILT_WARN`，或 双耳-双肩相对倾斜角 > `EAR_HEAD_TILT_WARN`
- 头部定位：双眼优先，眼不可见时用双耳兜底
- 不良语音提示：连续 `POSTURE_ALERT_CONSECUTIVE`（`app_main.c`，默认 2）帧不良才触发 `audio_player_play_posture_alert()` 播放 `res/bad_pose.mp3`（中断重新计数；UDP 逐帧 result 不受影响）。**提醒模式**（`posture_alert_repeat_enabled()`，NVS 持久化，20003/网页可设，默认连续播）：连续播=不良持续则每帧播完接着播（播放阻塞式天然背靠背）；只播一次=每轮不良事件仅首次播、恢复后才可再触发

### 检测时间段调度（`main/posture_sched.c`）

- 最多 5 个每日重复时段，[start, end) 左闭右开，支持跨午夜（start > end 如 22:00-06:30）；0 段 = 全天检测
- 设置经 20003 端口下发（见上），写入 NVS（namespace `psched`，key `slots`），断电重启后仍生效
- **时间未同步（SNTP 未完成）或未设置时段时全部检测**（降级策略）；同步完成瞬间推送状态（经标志位由 20003 任务代发，见 20003 协议说明——SNTP 回调上下文禁止直接 sendto）
- 窗口外推理任务不取帧不推理不发送（5s 低频轮询等待进窗），进入/离开窗口有边沿日志

### 主要文件

| 文件                           | 功能                               |
| ------------------------------ | ---------------------------------- |
| `main/app_main.c`            | 入口点，初始化顺序                 |
| `main/cam.c`                 | 通过 esp32-camera 驱动初始化摄像头 |
| `main/posture_model.h`        | 坐姿模型头文件（宏定义、接口声明） |
| `main/posture_model.cpp`      | ESP-DL 模型加载、推理、姿态判断   |
| `main/wifi_manager.c`        | WiFi AP/STA 初始化和事件处理       |
| `main/wifi_config_manager.c` | 强制门户，NVS 凭据存储             |
| `main/udp_camera_client.c`   | UDP 图像/音频发送和接收任务、20003 发现/调度/OTA |
| `main/time_sync.c`           | SNTP 网络对时（拿到 IP 自动同步，CST-8） |
| `main/posture_sched.c`       | 检测时间段调度 + 不良提醒模式（NVS 持久化、跨午夜、未同步全检测） |
| `simple_udp_receiver.py`     | PC 端接收器：图像/结果接收叠加、发现+调度下发（交互命令 s）、OTA 命令、历史落库（SQLite）；图像按设备分目录抽稀保存 `received_images/<设备ID>/`（`img` 命令/Web 可设叠加开关、抽稀间隔与仅有人帧开关：全部默认+按设备覆盖，recv_settings.json 持久化） |
| `web_ui.py`                  | Web 控制台（:20005）：设备表/OTA 按钮/坐姿统计图表（设备×时间范围）/时段设置/日志面板，复用接收器命令路径 |
| `posture_report.py`          | 坐姿历史报告：多时间窗统计 + 趋势图（matplotlib） |
| `raspberry_pi_deploy.md`     | 树莓派部署指南（依赖/systemd 服务/自启管理/OTA 注意点） |
| `espcam.service`             | systemd 服务配置（开机自启+崩溃拉起，配合上面的指南使用） |
| `main/audio_player.c`        | I2S 播放（状态提示音和音频流）     |
| `main/bad_pose.h`            | 坐姿不良提示音数据（由 `res/bad_pose.mp3` 转换） |
| `main/led.c`                 | LED 呼吸/闪烁模式                  |
| `main/dns_server.c`          | DNS 服务器用于强制门户重定向       |
| `res/wifi_*.c`               | 内嵌的音频数据（WiFi 状态提示音）  |
| `model/pose_model.espdl` | 当前使用的量化坐姿检测模型（6 关键点：双眼/双耳/双肩） |
| `model/esp32_deploy/`        | ESP-DL 部署参考代码 + 模型接口变更改动指南（`model_deploy_update.md`） |
| `sdkconfig.defaults`         | 默认配置（PSRAM、摄像头型号、Watchdog） |
| `partitions.csv`             | 分区表：双 OTA 槽（ota_0/ota_1 各 5MB）+ otadata + storage |
| `main/app_version.h`         | 固件版本号（手动维护，年月日_当日序号） |
| `ref/esp-dl-master/examples/yolo11_pose` | ESP-DL 官方姿态检测示例 |
| `ref/how_to_load_test_profile_model.rst` | ESP-DL 模型部署官方文档 |

### 开发板配置

`CAMERA_MODEL_ESP32S3_EYE` 在 `sdkconfig.defaults` 中定义。摄像头引脚（XCLK=GPIO15, SIOD=GPIO4, SIOC=GPIO5 等）在 `main/camera_app.h` 中定义。

### 内存限制（重要）

- **IRAM（TCM，16KB）**：极度紧张，99.99% 已用，几乎无法添加新 IRAM 代码
- **PSRAM（DIRAM，341KB）**：充裕，约 232KB 可用，用于模型参数和中间结果
- **模型参数策略**：使用 `param_copy=true` 把参数拷到 PSRAM（8MB PSRAM 充裕）；`false` 时每个卷积都要从 flash 读权重，推理会慢到 40s+ 触发 watchdog
- **推理必须在 PSRAM 上分配内存**，避免使用 IRAM

### Watchdog 配置

模型推理历史耗时较长（旧 4 点模型 ~10 秒），需在 menuconfig 中增加超时；新 6 点模型（ReLU + nearest 上采样）预期 150–300ms，40s 上限仍有充足余量：
- 路径：`Component config` → `Task Watchdog` → `Task watchdog timeout (s)`
- 推荐值：**40 秒**（或修改 `sdkconfig.defaults` 添加 `CONFIG_ESP_TASK_WDT_TIMEOUT_S=40`）

### LED 状态

- 熄灭：正常空闲
- 快闪：WiFi 配置模式 / 连接失败
- 呼吸：UDP 相机正在传输 / 姿态检测运行中
- 常亮：摄像头初始化失败
