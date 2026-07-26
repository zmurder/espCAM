# ESP32 部署代码更新指南（4 点 → 6 点模型重构）

> 本文档记录模型从「旧 4 点（0.9M）」更新到「新 6 点（0.13M，含双耳）」时，
> ESP32 端部署代码需要同步修改的地方，以及完整的烧录验证流程。
> 适用于本次更新，也作为以后「模型接口变化时 ESP32 端怎么改」的参考。

## 1. 模型变更概述

| 项 | 旧模型 | 新模型 |
|---|---|---|
| 关键点数 | 4（左/右眼、左/右肩） | **6（+左/右耳）** |
| 顺序 | left_eye, right_eye, left_shoulder, right_shoulder | **left_eye, right_eye, left_ear, right_ear, left_shoulder, right_shoulder** |
| 参数量 | 0.9M | **0.13M（1/7）** |
| 上采样 | ConvTranspose（InsertZeros，ESP32 慢） | **Resize nearest**（无大中间张量） |
| 激活 | HardSwish | **ReLU** |
| ESP32 推理 | ~10 秒 | **预期 150-300ms** |
| 输入 | (1,3,240,320) RGB + ImageNet 归一化 | **不变** |
| 输出 | (1,4,120,160) heatmap + Sigmoid | **(1,6,120,160) + Sigmoid**（仅通道数变） |

**接口不变的部分**：输入分辨率、heatmap 分辨率(120×160)、Sigmoid 归一化、纯 argmax decode、exponent 定点输出。所以推理链路（pose_inference）不动，只改后处理 + 关键点定义。

## 2. ESP32 端必须修改的文件

### 2.1 `components/pose_deploy/pose_postprocess.h`

- `NUM_KP = 4` → **`6`**
- 新增索引常量（关键点顺序变了，**肩索引从 2/3 变成 4/5**，这是最容易出错的地方）：
  ```cpp
  constexpr int IDX_LEFT_EYE=0, IDX_RIGHT_EYE=1;
  constexpr int IDX_LEFT_EAR=2, IDX_RIGHT_EAR=3;       // 新增耳朵
  constexpr int IDX_LEFT_SHOULDER=4, IDX_RIGHT_SHOULDER=5;  // 原 2/3 → 4/5
  ```
- `PoseResult` 结构体：把 `eye_tilt_deg/eye_dist_px` 改成 `head_tilt_deg/head_dist_px`，加 `int head_source`（0=眼,1=耳,-1=无）。
- `HEATMAP_H=120 / HEATMAP_W=160 / CONF_THRESH=0.6` **不变**。

### 2.2 `components/pose_deploy/pose_postprocess.cc`

- argmax 循环 `NUM_KP` 自动跟随（6 点），无需改逻辑。
- **双肩倾斜用 idx 4/5**（原 2/3）：
  ```cpp
  if (r.kps[IDX_LEFT_SHOULDER].valid && r.kps[IDX_RIGHT_SHOULDER].valid) { ... }
  ```
- **头部定位「眼优先、眼不可见用耳兜底」**（新增，低头看不到眼时用耳）：
  ```cpp
  bool eyes_ok = ...IDX_LEFT_EYE/RIGHT_EYE...;
  bool ears_ok = ...IDX_LEFT_EAR/RIGHT_EAR...;
  if (eyes_ok)      { 用眼; head_source=0; }
  else if (ears_ok) { 用耳; head_source=1; }   // 低头兜底
  else              { head_source=-1; }
  ```
- `reliable` = 双肩可见 && n_valid≥3（坐姿判断依赖双肩）。

### 2.3 `main/main.cc`

- 头注释：4 通道 → 6 通道。
- 坐姿日志：`眼倾/眼距` → `头倾/头距(来自眼或耳)`：
  ```cpp
  const char* head_src = r.head_source==0?"眼": r.head_source==1?"耳":"无";
  ESP_LOGI(TAG, "坐姿正常 肩倾%.1f° 头倾%.1f°(来自%s) 肩宽%.0fpx 头距%.0fpx", ...);
  ```

## 3. **不需要**改的部分

- `pose_inference.h / pose_inference.cc`：模型加载、前处理（ImageNet 归一化 mean=[123.675,116.28,103.53] std=[58.395,57.12,57.375]）、推理调用 —— 与关键点数无关，heatmap tensor shape 自动跟随 `.espdl`。
- `CMakeLists.txt`（顶层 + 组件）：不动。
- 摄像头配置（FRAMESIZE_QVGA 320×240、RGB888）：不动。

## 4. 部署步骤

```bash
# 1. 拷贝新模型到 SD 卡（ESP32 代码读的是 pose_model.espdl，覆盖旧的）
cp output/pose_model_6kp.espdl  <SD卡>/pose_model.espdl
cp output/pose_model_6kp.info   <SD卡>/pose_model.info

# 2. rebuild 固件（关键点数和坐姿逻辑变了，必须 rebuild，不能只换模型）
cd esp32_deploy
idf.py set-target esp32s3
idf.py build flash monitor
```

> 若用 RODATA 嵌入固件方式：把 `pose_model_6kp.espdl` 转 C 数组（见 README），`#define LOAD_FROM_RODATA`。

## 5. 验证

烧录后看 monitor 日志：

1. **推理耗时**（首要目标）：旧 ~10000ms → 新预期 **150-300ms**。
2. **坐姿日志**：`坐姿正常 肩倾X.X° 头倾X.X°(来自眼) ...`；低头（眼不可见）时应显示 `(来自耳)`。
3. **6 点 conf**：正常坐姿眼/耳/肩 conf 都应 >0.6（坐姿算法 conf 阈值）。

若推理仍 >300ms：确认 `.info` 是新模型（~150KB，旧的 1.2MB 说明没换）；确认日志算子无 ConvTranspose。

## 6. 通用：以后模型接口再变时 ESP32 怎么改

| 模型变化 | ESP32 改动点 |
|---|---|
| 关键点数 / 顺序变 | `pose_postprocess.h` NUM_KP + IDX_* 常量；`pose_postprocess.cc` 坐姿用的索引；`main.cc` 日志 |
| heatmap 分辨率变（如 120×160→96×128） | `pose_postprocess.h` HEATMAP_H/W |
| 输入分辨率变（如 320×240→192×256） | `main.cc` 摄像头 FRAMESIZE + 坐标计算里的 320/240 常量 |
| 去 Sigmoid（输出不再 [0,1]） | `pose_postprocess.cc` conf 计算 + `CONF_THRESH` 重新校准 |
| 增减坐姿判断指标 | `PoseResult` 结构体 + postprocess 计算 + main 日志 |

> 核心原则：**推理链路（pose_inference）对模型内部结构无感知**，只要输入/输出 tensor shape 和语义不变，就只换 `.espdl`；一旦关键点数/分辨率/激活这些"对外契约"变了，才需要改后处理代码并 rebuild。
