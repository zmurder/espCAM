# ESP-DL 坐姿检测系统集成

## 当前状态

已完成，使用 **rodata 嵌入** 方式加载模型。

## 架构

### 模型加载方式（rodata 嵌入）
- 模型文件通过 `EMBED_FILES` + `target_add_aligned_binary_data` 嵌入 app 分区
- 16MB Flash，factory 分区 8MB
- 修改模型需重烧 app

### 关键文件

| 文件 | 用途 |
|------|------|
| `main/posture_model.cpp` | 模型推理（rodata 加载） |
| `main/posture_model.h` | 有 `extern "C"` |
| `main/CMakeLists.txt` | `EMBED_FILES` + `target_add_aligned_binary_data` |
| `partitions.csv` | factory=8MB, nvs=24K, phy=4K |
| `sdkconfig.defaults` | 16MB Flash + 自定义分区表 |
| `main/idf_component.yml` | esp-dl, esp32-camera, mdns |

## 关键发现（教训）

1. **extern "C" 必须加** — 被 C 代码调用的 C++ 函数需要
2. **JPEG 解码用高层 API** — `dl::image::sw_decode_jpeg()`（只需 REQUIRES esp-dl）
3. **Tensor 类名是 TensorBase** — 不是 Tensor
4. **REQUIRES 会禁用自动扫描** — 保持空，依赖 idf_component.yml
5. **esptool_py_flash_to_partition 需要固定 offset** — rodata 嵌入更简单

## 待验证

1. `idf.py fullclean && idf.py reconfigure && idf.py build`
2. `idf.py -p PORT flash monitor` — 模型加载是否正常
