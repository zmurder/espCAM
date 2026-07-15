#pragma once
#include "dl_model_base.hpp"
#include "dl_image_preprocessor.hpp"
#include <vector>

namespace pose {

/**
 * @brief 姿态模型推理封装：加载 .espdl + 前处理（与训练对齐）+ 推理
 *
 * 前处理对齐训练（dataset.py 的 center_crop_resize + ImageNet normalize）：
 *   - center crop 最大 4:3 区域（crop_area 指定，4:3 输入则全图）
 *   - resize 到 320×240（ImagePreprocessor 自动按 model input shape）
 *   - ImageNet 归一化 mean=[0.485,0.456,0.406] std=[0.229,0.224,0.225]
 *     （ESP-DL 要 [0,255] 范围，已 ×255）
 *   - RGB 输入（rgb_swap=false，训练用 RGB）
 *   - 自动 quantize 到 int8（按 model input exponent）
 */
class PoseInference {
public:
    /**
     * @param model_path  .espdl 路径（SDCARD）或 rodata 地址（RODATA）
     * @param location    模型位置，默认 SDCARD；嵌入固件用 MODEL_LOCATION_IN_FLASH_RODATA
     */
    PoseInference(const char* model_path,
                  fbs::model_location_type_t location = fbs::MODEL_LOCATION_IN_SDCARD);
    ~PoseInference();

    /**
     * @brief 推理一帧
     * @param img        摄像头帧（dl::image::img_t）
     * @param crop_area  center crop 区域 {x,y,w,h}；非 4:3 图像需先 center crop 到 4:3。
     *                   空则全图（适用于 320×240 等 4:3 输入）
     * @return heatmap 输出 TensorBase*（int8, shape [1,4,120,160]），所有权归 model
     */
    dl::TensorBase* infer(const dl::image::img_t& img,
                          std::vector<int> crop_area = {});

private:
    dl::Model* model_ = nullptr;
    dl::image::ImagePreprocessor* preprocessor_ = nullptr;
};

} // namespace pose
