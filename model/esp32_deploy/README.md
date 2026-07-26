# ESP32-S3 坐姿检测 — 部署工程

把训练好的姿态模型（`pose_model.espdl`）部署到 ESP32-S3，从 OV3660 摄像头实时采集 320×240 图像，推理输出 **6 关键点（双眼/双耳/双肩）**，判断坐姿（倾斜度/距离；低头看不到眼时用双耳定位头部）。

> **模型从 4 点升级到 6 点（含双耳）+ 结构重构（0.84M→0.13M，推理 10s→预期 150-300ms）**，部署代码改动详见 [`model_deploy_update.md`](model_deploy_update.md)。

## 模型信息

| 项 | 值 |
|---|---|
| 文件 | `pose_model.espdl` (~150KB) + `pose_model.info` |
| 量化 | int8 对称（ESP-PPQ, target=esp32s3） |
| 输入 | `(1,3,240,320)` RGB + ImageNet 归一化 |
| 输出 | `(1,6,120,160)` heatmap（左眼/右眼/左耳/右耳/左肩/右肩） + Sigmoid |
| 参数量 | 0.13M（重构版，上采样用 Resize 无 ConvTranspose，激活 ReLU） |
| 精度 | cam 实拍 PCK@0.1 = 96.50%（量化后，损失 1.15%）；眼/耳近满分、肩膀 ~0.93 |

## 目录结构

```
esp32_deploy/
├── CMakeLists.txt                 # 顶层（ESP-IDF project）
├── README.md
├── components/pose_deploy/        # 推理 + 后处理组件
│   ├── CMakeLists.txt
│   ├── pose_inference.h/.cc       # 加载模型 + 前处理 + 推理
│   └── pose_postprocess.h/.cc     # heatmap argmax + conf 阈值 + 坐姿判断
└── main/
    ├── CMakeLists.txt
    └── main.cc                    # 摄像头采集 + 推理 + 坐姿判断示例
```

## 依赖

- **ESP-IDF v5.x**（设好 `IDF_PATH` 并 `. $IDF_PATH/export.sh`）
- **esp-dl** 组件：`idf.py add-dependency "espressif/esp-dl"`
- **esp32-camera** 组件：`idf.py add-dependency "espressif/esp32-camera"`
- 硬件：ESP32-S3（带 PSRAM）+ OV3660 摄像头 +（SD 卡，若从 SD 加载模型）

## 准备模型文件

量化产物在训练项目 `output/` 下，需拷到 ESP32：

**方式 A：SD 卡（推荐，便于换模型）**
```bash
# 把 espdl + info 拷到 SD 卡根目录
cp output/pose_model.espdl /mnt/sdcard/
cp output/pose_model.info  /mnt/sdcard/
```
`main.cc` 默认用此方式（`MODEL_LOCATION_IN_SDCARD`，路径 `/sdcard/pose_model.espdl`）。

**方式 B：嵌入固件 RODATA（省 SD 卡，换模型需重烧）**
```bash
# 把 .espdl 转成 C 数组
xxd -i output/pose_model.espdl > model/pose_model_espdl.h
```
在 `main.cc` 里 `#define LOAD_FROM_RODATA`，并在 `main/CMakeLists.txt` 用 `EMBED_FILES` 嵌入。

## 编译烧录

```bash
cd esp32_deploy
idf.py set-target esp32s3
idf.py menuconfig    # 按需配置 PSRAM、摄像头引脚、SD 卡
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## 前处理对齐训练（重要）

模型推理效果取决于前处理与训练一致。`pose_inference.cc` 已对齐 `dataset.py`：

| 步骤 | 训练 | 部署 |
|---|---|---|
| 裁剪 | `center_crop_resize`（中心裁最大 4:3） | `ImagePreprocessor.preprocess(img, crop_area)` |
| 尺寸 | 320×240 | 同（model input shape） |
| 归一化 | ImageNet mean=[0.485,0.456,0.406] std=[0.229,0.224,0.225]（[0,1]） | 同，转 [0,255]：mean=[123.675,116.28,103.53] std=[58.395,57.12,57.375] |
| 颜色 | RGB | `rgb_swap=false` |
| 量化 | float | ESP-DL 按 input exponent 自动 quantize int8 |

> 摄像头若输出非 4:3，需在 `main.cc` 算 `crop_area` 做 center crop，否则会拉伸变形导致精度下降。

## 后处理（`pose_postprocess.cc`）

纯 heatmap argmax，无 offset（ESP32 C++ 实现简单）：

1. **argmax**：每个关键点通道在 120×160 上找最大值位置 → 归一化坐标 [0,1]
   - 直接在 int8 上做（量化单调，结果等价 float argmax）
2. **conf**：`max_int8 × 2^exponent` dequantize → Sigmoid 输出 [0,1]
3. **conf 阈值 0.6**：<0.6 标记 `valid=false`
   - 实测：conf≥0.6 时眼睛 PCK 100%、肩膀 97–99.7%；conf<0.6 的 13% 肩膀是难检样本（侧身/遮挡/边缘），argmax 会跑偏，必须过滤
4. **坐姿指标**（6 点，索引 0/1眼、2/3耳、4/5肩）：
   - `shoulder_tilt_deg`：双肩连线倾斜角（idx 4/5，双肩水平=0°）
   - `head_tilt_deg`：头部连线倾斜角——**眼(0/1)优先，眼不可见(低头)则用耳(2/3)兜底**，`head_source` 标记来自眼还是耳
   - `shoulder_width_px` / `head_dist_px`：像素距离
5. **坐姿判断**：`reliable`(双肩可见 且 ≥3 点 valid) 且 `|shoulder_tilt_deg|>10°` → 判歪斜

## 调参

`pose_postprocess.h`：
- `CONF_THRESH = 0.6`：置信度阈值。调低→更多点可用但可能引入跑偏点；调高→更稳但覆盖率降
- `TILT_WARN_DEG = 10.0`：坐姿歪斜告警角度阈值

`pose_inference.cc`：
- `rgb_swap`：模型用 RGB(false)。若摄像头只能给 BGR 则改 true

## 摄像头适配

`main.cc` 的 `camera_config` 引脚是示例，**按你的开发板原理图改**。ESP32-S3 + OV3660 常见配置：
- `PIXFORMAT_RGB888`：与训练 RGB 对齐，最简单（帧率略低，坐姿检测 5fps 够用）
- 若用 `PIXFORMAT_RGB565` 省带宽，需在送 `img_t` 前转 RGB888，或确认 esp-dl `ImagePreprocessor` 支持 RGB565 输入

`img_t` 字段名/格式枚举按你的 esp-dl 版本确认（`esp-dl/vision/image/dl_image_process.hpp`、`dl_image_define.hpp`）。

## 性能优化（可选）

- **双核推理**：`pose_inference.cc` 的 `model_->run()` 默认单核；ESP32-S3 双核可传 `RUNTIME_MODE_DUAL_CORE` 加速
- **帧率**：`vTaskDelay(200ms)` ≈ 5fps；坐姿变化慢，足够。需要更快可减小延迟
- **内存**：模型参数默认拷到 PSRAM；PSRAM 紧张时 `Model(..., param_copy=false)` 直接从 FLASH 读（牺牲速度）

## 验证

烧录后串口应输出类似：
```
I (xxxx) pose: 坐姿正常 肩倾1.5° 头倾0.8°(来自眼) 肩宽95px 头距48px
I (xxxx) pose: 坐姿正常 肩倾1.2° 头倾0.5°(来自耳) 肩宽92px 头距40px   ← 低头时用耳
W (xxxx) pose: >> 坐姿歪斜! 双肩倾斜 13.2° (阈值10°) 肩宽90px
```

若 `姿态不可靠(仅N点可信)` 频繁出现，说明 conf 普遍偏低，检查：
1. 前处理是否对齐（颜色/归一化/裁剪）
2. 摄像头画面是否清晰、人是否在画面中
3. 降低 `CONF_THRESH` 试探（但可能引入跑偏点）
