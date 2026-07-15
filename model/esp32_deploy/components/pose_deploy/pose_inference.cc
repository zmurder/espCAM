#include "pose_inference.h"

namespace pose {

PoseInference::PoseInference(const char* model_path, fbs::model_location_type_t location) {
    // 加载模型（graph + 参数），自动内存规划
    model_ = new dl::Model(model_path, location);

    // ImageNet 归一化：训练用 [0,1] 范围 mean=[0.485,0.456,0.406] std=[0.229,0.224,0.225]
    // ESP-DL ImagePreprocessor 要求 [0,255] 范围，故 ×255
    //   mean = [123.675, 116.28, 103.53]
    //   std  = [58.395, 57.12, 57.375]
    // rgb_swap=false：模型训练时输入为 RGB
    // ImagePreprocessor 内部会按 model input 的 exponent 自动 quantize 到 int8
    preprocessor_ = new dl::image::ImagePreprocessor(
        model_,
        {123.675f, 116.28f, 103.53f},
        {58.395f, 57.12f, 57.375f},
        false);
}

PoseInference::~PoseInference() {
    delete preprocessor_;
    delete model_;
}

dl::TensorBase* PoseInference::infer(const dl::image::img_t& img, std::vector<int> crop_area) {
    // 前处理：crop_area 裁剪 -> resize 到 320×240 -> normalize -> quantize
    preprocessor_->preprocess(img, crop_area);
    // 推理（run 默认用已填入的 model input）
    model_->run();
    // 取唯一输出 heatmap (1,4,120,160)
    return model_->get_output();
}

} // namespace pose
