---
name: ESP-DL 模型部署指南
description: ESP-DL 框架在 ESP32-S3 上部署深度学习模型的完整流程
type: reference
---

# ESP-DL 模型部署参考

## 本地文档
- `ref/how_to_load_test_profile_model.rst` — 官方教程完整原文

## 核心加载方式（3种）

### 1. 从 rodata 加载（最简单）
模型嵌入 FLASH `.rodata` 段，随应用一起烧录。

```cpp
#include "dl_model_base.hpp"
extern const uint8_t model_espdl[] asm("_binary_model_xxx_start");
dl::Model *model = new dl::Model((const char *)model_espdl, fbs::MODEL_LOCATION_IN_FLASH_RODATA);
```

CMakeLists.txt 需添加：
```cmake
idf_build_get_property(component_targets __COMPONENT_TARGETS)
if ("___idf_espressif__esp-dl" IN_LIST component_targets)
   idf_component_get_property(espdl_dir espressif__esp-dl COMPONENT_DIR)
elseif("___idf_esp-dl" IN_LIST component_targets)
   idf_component_get_property(espdl_dir esp-dl COMPONENT_DIR)
endif()
set(cmake_dir ${espdl_dir}/fbs_loader/cmake)
include(${cmake_dir}/utilities.cmake)
set(embed_files your_model_path/model_name.espdl)
idf_component_register(...)
target_add_aligned_binary_data(${COMPONENT_LIB} ${embed_files} BINARY)
```

### 2. 从 partition 加载（推荐开发时）
模型存储在独立 FLASH 分区，可单独更新模型不重烧应用。

partition.csv 添加：
```
factory,  app,  factory,  0x010000,  4000K,
model,   data,  spiffs,        ,  4000K,
```

烧录时用：`idf.py app-flash` 仅烧录应用，不重烧模型。

```cpp
dl::Model *model = new dl::Model("model", fbs::MODEL_LOCATION_IN_FLASH_PARTITION);
```

### 3. 从 SDCard 加载
FLASH 紧张时使用，加载较慢。

## 关键 API

| 方法 | 功能 |
|------|------|
| `model->test()` | 板端验证推理正确性（需导出时启用 test_values） |
| `model->profile_memory()` | 打印 IRAM/PSRAM/FLASH 内存使用明细 |
| `model->profile_module()` | 打印每层推理延迟（微秒） |
| `model->profile()` | 综合分析（推荐使用） |

## 内存与性能权衡

| 参数 | `param_copy=true` | `param_copy=false` |
|------|-------------------|---------------------|
| RAM 使用 | 高 | 低 |
| 推理性能 | 快 | 慢 |
| 适用场景 | 性能优先 | RAM 紧张时 |

## 在本项目的限制

**IRAM 已满（99.99%，16383/16384 字节）**

- ESP-DL 推理框架本身需要 IRAM，IRAM 无剩余空间
- PSRAM（DIRAM）充裕：232KB 剩余 / 341KB 总计
- **方案A（推荐）：PC 端推理** — 图像已通过 UDP 传到 PC，PC 用 Python 推理，ESP32 无需改动
- **方案B：ESP-DL** — 需要大幅裁剪模型 + 量化，且必须使用 `param_copy=false` 把参数保留在 FLASH

## 相关资源
- 官方文档：`ref/how_to_load_test_profile_model.rst`
- ESP-DL GitHub：https://github.com/espressif/esp-dl
- ESP32-S3 人体活动识别部署：https://blog.csdn.net/espressif/article/details/131123326
