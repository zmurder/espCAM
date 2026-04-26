# LitePose ESP32 模型规格文档

## 模型信息

| 属性 | 值 |
| :--- | :--- |
| 模型文件 | `litepose_esp32s3.espdl` |
| 参数量 | 96,648 |
| 模型大小 | 144KB |
| 量化方式 | PTQ 8bit |
| 目标平台 | ESP32-S3 |

## 模型输入

| 属性 | 值 |
| :--- | :--- |
| 形状 | `[1, 3, 240, 320]` (batch=1, channels=3, height=240, width=320) |
| 数据类型 | `int8_t` (量化) 或 `float` |
| 值范围 | RGB: [0, 255] → 归一化: [0, 1] |

### 前处理步骤

```cpp
// 1. Letterbox Resize（保持宽高比，padding居中）
//    输入: 任意分辨率图像
//    输出: 320x240，空白区域用 (114,114,114) 填充

// 2. 归一化
float input[3][240][320];
for (c in 0..2, h in 0..240, w in 0..320) {
    input[c][h][w] = pixel[c][h][w] / 255.0f;  // [0, 1]
}

// 3. 量化（8bit）
int8_t quant_input[1][3][240][320];
int exponent = -7;  // Scale = 2^exponent
for (c, h, w) {
    quant_input[0][c][h][w] = dl::quantize<int8_t>(input[c][h][w], exponent);
}
```

### ESP-DL 前处理参考

```cpp
#include "dl_image_preprocessor.hpp"

// 使用 ImagePreprocessor 进行 letterbox resize
dl::ImagePreprocessor preprocessor;
preprocessor.set_target_size({320, 240});
preprocessor.set_pad_value({114, 114, 114});
preprocessor.process(input_image, output_tensor);
```

## 模型输出

| 属性 | 值 |
| :--- | :--- |
| 形状 | `[1, 7, 60, 80]` (batch=1, keypoints=7, height=60, width=80) |
| 数据类型 | `int8_t` (量化) 或 `float` |
| 值范围 | [0, 1] (Sigmoid 输出) |

### 关键点定义

| ID | 名称 | 说明 |
| :--- | :--- | :--- |
| 0 | left_eye | 左眼 |
| 1 | right_eye | 右眼 |
| 2 | left_ear | 左耳 |
| 3 | right_ear | 右耳 |
| 4 | nose | 鼻子 |
| 5 | left_shoulder | 左肩 |
| 6 | right_shoulder | 右肩 |

## 后处理（ESP32上实现）

### 步骤1：反量化输出

```cpp
// 获取量化输出
int8_t quant_output[1][7][60][80];
model_output->get_data(quant_output);
int exponent = model_output->exponent;  // 获取 scale

// 反量化到浮点
float heatmap[7][60][80];
for (k in 0..6, h in 0..60, w in 0..80) {
    heatmap[k][h][w] = dl::dequantize(quant_output[0][k][h][w], exponent);
}
```

### 步骤2：Argmax 提取关键点

```cpp
struct Keypoint {
    float x, y;  // 热力图坐标 [0, 60) 和 [0, 80)
    float score; // 置信度
};

Keypoint keypoints[7];

for (int k = 0; k < 7; k++) {
    int max_idx = 0;
    float max_val = heatmap[k][0];

    // 遍历热力图找最大值
    for (int i = 1; i < 60 * 80; i++) {
        if (heatmap[k][i] > max_val) {
            max_val = heatmap[k][i];
            max_idx = i;
        }
    }

    // 转换为 (y, x) 坐标
    keypoints[k].y = max_idx / 80;  // 行 = y
    keypoints[k].x = max_idx % 80;  // 列 = x
    keypoints[k].score = max_val;
}
```

### 步骤3：坐标转换（原图坐标系）

```cpp
// 假设原图分辨率为 orig_w x orig_h
// Letterbox 参数（需要记录）
int pad_left, pad_top;
float scale;  // 缩放比例

for (int k = 0; k < 7; k++) {
    // 1. 热力图坐标转换为 letterbox 坐标
    float letterbox_x = keypoints[k].x;
    float letterbox_y = keypoints[k].y;

    // 2. 去除 padding，得到缩放后的图像坐标
    float scaled_x = (letterbox_x - pad_left) / scale;
    float scaled_y = (letterbox_y - pad_top) / scale;

    // 3. 转换回原始图像坐标
    keypoints[k].x = scaled_x / orig_w;  // 归一化 [0, 1]
    keypoints[k].y = scaled_y / orig_h;  // 归一化 [0, 1]
}
```

## 完整后处理示例

```cpp
#include "dl_model.hpp"
#include "dl_tensor.hpp"

void pose_postprocess(dl::Model* model, dl::TensorBase* input,
                     int orig_w, int orig_h,
                     int pad_left, int pad_top, float scale,
                     Keypoint keypoints[7]) {
    // 1. 模型推理
    model->run(input);

    // 2. 获取输出
    auto outputs = model->get_outputs();
    dl::TensorBase* output = outputs.begin()->second;

    // 3. 反量化
    int8_t* quant_data = (int8_t*)output->get_data();
    int exponent = output->exponent;

    // 4. Argmax 提取关键点
    for (int k = 0; k < 7; k++) {
        int max_idx = 0;
        float max_val = dl::dequantize(quant_data[k * 60 * 80], exponent);

        for (int i = 1; i < 60 * 80; i++) {
            float val = dl::dequantize(quant_data[k * 60 * 80 + i], exponent);
            if (val > max_val) {
                max_val = val;
                max_idx = i;
            }
        }

        // 热力图坐标
        float h_x = (max_idx % 80);
        float h_y = (max_idx / 80);

        // 原图坐标（归一化）
        keypoints[k].x = (h_x - pad_left) / scale / orig_w;
        keypoints[k].y = (h_y - pad_top) / scale / orig_h;
        keypoints[k].score = max_val;
    }
}
```

## 注意事项

1. **Batch Size**: ESP-DL 仅支持 batch_size=1
2. **内存覆盖**: 模型推理后，input 内存可能被 output 覆盖，需要先处理 output
3. **量化一致性**: 输入输出量化 exponent 由 ESP-PPQ 自动计算，需从 `.json` 文件获取或从 `TensorBase` 获取
4. **Letterbox 参数**: 前处理时需记录 `pad_left`, `pad_top`, `scale`，后处理时需要使用

## 参考文件

- `checkpoints/litepose_esp32s3.json` - 量化配置（含 exponent）
- `checkpoints/litepose_esp32s3.info` - 模型调试信息