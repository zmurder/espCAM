# pose_deploy 组件：ESP32-S3 姿态检测推理 + 后处理
# 依赖 esp-dl 组件（需在项目 components/ 下放置 esp-dl，或作为 managed component）
idf_component_register(
    SRCS
        "pose_postprocess.cc"
        "pose_inference.cc"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp-dl
)
