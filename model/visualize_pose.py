"""
坐姿检测可视化脚本
在图像上绘制关键点和骨架，验证模型效果
"""

import os
import cv2
import torch
import numpy as np
from PIL import Image
import torchvision.transforms as transforms

from model import LitePoseNet


# 关键点定义
KEYPOINT_NAMES = [
    'left_eye', 'right_eye', 'left_ear', 'right_ear',
    'nose', 'left_shoulder', 'right_shoulder'
]

# 关键点颜色 (BGR)
KEYPOINT_COLORS = [
    (255, 0, 0),    # 左眼 - 蓝色
    (0, 255, 0),    # 右眼 - 绿色
    (255, 255, 0),  # 左耳 - 青色
    (0, 255, 255),  # 右耳 - 黄色
    (255, 0, 255),  # 鼻子 - 紫色
    (0, 128, 255),  # 左肩 - 橙色
    (0, 0, 255),    # 右肩 - 红色
]

# 骨架连接 (索引从0开始)
SKELETON_CONNECTIONS = [
    (0, 4),  # 左眼-鼻子
    (1, 4),  # 右眼-鼻子
    (0, 2),  # 左眼-左耳
    (1, 3),  # 右眼-右耳
    (2, 4),  # 左耳-鼻子
    (3, 4),  # 右耳-鼻子
    (0, 1),  # 左眼-右眼
    (5, 6),  # 左肩-右肩
    (4, 5),  # 鼻子-左肩
    (4, 6),  # 鼻子-右肩
]


def load_model(checkpoint_path, device='cpu'):
    """加载模型"""
    model = LitePoseNet(num_keypoints=7, heatmap_size=(60, 80))
    checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
    model.load_state_dict(checkpoint['model_state_dict'])
    model.eval()
    return model


def letterbox_resize(image, target_width, target_height, pad_value=(114, 114, 114)):
    """
    Letterbox resize - 保持宽高比缩放图片，空白区域用pad_value填充

    Args:
        image: PIL Image
        target_width: 目标宽度
        target_height: 目标高度
        pad_value: 填充颜色

    Returns:
        resized: 处理后的 PIL Image (target_height, target_width, C)
        pad_left, pad_top: 填充的偏移量
        scale: 缩放比例
        orig_w, orig_h: 原始图像尺寸
    """
    orig_w, orig_h = image.size
    orig_array = np.array(image)

    # 计算缩放比例（取较小的，保证填满目标尺寸）
    scale = min(target_width / orig_w, target_height / orig_h)

    # 新的宽高
    new_w = int(orig_w * scale)
    new_h = int(orig_h * scale)

    # 缩放图片
    resized = image.resize((new_w, new_h), Image.BILINEAR)
    resized_array = np.array(resized)

    # 创建目标尺寸的画布
    canvas = np.full((target_height, target_width, 3), pad_value, dtype=np.uint8)

    # 计算填充偏移（居中）
    pad_left = (target_width - new_w) // 2
    pad_top = (target_height - new_h) // 2

    # 将缩放后的图片放到画布中央
    canvas[pad_top:pad_top + new_h, pad_left:pad_left + new_w] = resized_array

    return Image.fromarray(canvas), pad_left, pad_top, scale, orig_w, orig_h


def preprocess_image(image_path, img_width=320, img_height=240):
    """
    预处理图像 - Letterbox resize保持宽高比（与训练一致）
    返回: (1, 3, 240, 320) 的tensor, 原始图像, letterbox参数
    """
    img = Image.open(image_path).convert('RGB')
    original_size = img.size  # (宽, 高)

    # Letterbox resize到目标尺寸（与训练一致）
    img_letterbox, pad_left, pad_top, scale, orig_w, orig_h = letterbox_resize(
        img, img_width, img_height
    )

    # 转换为 tensor 并归一化
    img_tensor = transforms.ToTensor()(img_letterbox)
    img_tensor = transforms.Normalize(
        mean=[0.485, 0.456, 0.406],
        std=[0.229, 0.224, 0.225]
    )(img_tensor).unsqueeze(0)  # 添加batch维度

    # 转换颜色空间：RGB -> BGR (因为后面用 OpenCV 处理)
    img_bgr = cv2.cvtColor(np.array(img_letterbox), cv2.COLOR_RGB2BGR)

    # 返回letterbox参数用于坐标还原
    letterbox_params = {
        'pad_left': pad_left,
        'pad_top': pad_top,
        'scale': scale,
        'orig_w': orig_w,
        'orig_h': orig_h,
        'img_width': img_width,
        'img_height': img_height
    }

    return img_tensor, img_bgr, original_size, letterbox_params


def extract_keypoints_from_heatmap(heatmaps, img_width=320, img_height=240):
    """
    从热力图中提取关键点坐标（letterbox坐标系）

    Args:
        heatmaps: (7, 60, 80) 的热力图 (numpy数组)
        img_width: letterbox目标宽度
        img_height: letterbox目标高度

    Returns:
        keypoints: (7, 2) 关键点坐标 [x, y] - 在letterbox坐标系中
        confidences: (7,) 置信度
    """
    keypoints = []
    confidences = []
    heatmap_h, heatmap_w = heatmaps.shape[1], heatmaps.shape[2]

    for k in range(7):
        hm = heatmaps[k]  # (60, 80)

        # 使用简单的 argmax
        max_idx = int(np.argmax(hm))
        max_val = float(hm.max())
        y = max_idx // heatmap_w
        x = max_idx % heatmap_w

        # 转换到letterbox坐标系 (320x240)
        letterbox_x = int(x * img_width / heatmap_w)
        letterbox_y = int(y * img_height / heatmap_h)

        keypoints.append([letterbox_x, letterbox_y])
        confidences.append(max_val)

    return np.array(keypoints, dtype=np.float32), np.array(confidences, dtype=np.float32)


def letterbox_to_original_coords(keypoints, letterbox_params):
    """
    将letterbox坐标系中的关键点转换回原始图像坐标系

    Args:
        keypoints: (7, 2) 关键点坐标 [x, y] - letterbox坐标系
        letterbox_params: preprocess_image返回的letterbox参数

    Returns:
        keypoints: (7, 2) 关键点坐标 [x, y] - 原始图像坐标系
    """
    pad_left = letterbox_params['pad_left']
    pad_top = letterbox_params['pad_top']
    scale = letterbox_params['scale']
    orig_w = letterbox_params['orig_w']
    orig_h = letterbox_params['orig_h']

    # 从letterbox坐标转换到原始图像坐标
    # 先去掉padding偏移，再缩放回原图
    orig_keypoints = keypoints.copy()
    orig_keypoints[:, 0] = (keypoints[:, 0] - pad_left) / scale
    orig_keypoints[:, 1] = (keypoints[:, 1] - pad_top) / scale

    return orig_keypoints


def draw_keypoints(image, keypoints, confidences=None, thickness=2):
    """在图像上绘制关键点（不绘制连线）"""
    img = image.copy()
    h, w = img.shape[:2]

    # 只绘制关键点，不绘制骨架连接
    for i, (x, y) in enumerate(keypoints):
        if confidences is not None and confidences[i] < 0.2:
            continue

        pt = (int(x), int(y))
        if 0 <= pt[0] < w and 0 <= pt[1] < h:
            color = KEYPOINT_COLORS[i]
            # 绘制实心圆
            cv2.circle(img, pt, thickness + 4, color, -1)
            # 绘制白色边框
            cv2.circle(img, pt, thickness + 6, (255, 255, 255), 2)

            # 绘制标签
            label = KEYPOINT_NAMES[i]
            if confidences is not None:
                label += f" {confidences[i]:.2f}"
            cv2.putText(img, label, (pt[0] + 8, pt[1] - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    return img


def draw_heatmaps(image, heatmaps, alpha=0.5):
    """在图像上叠加显示热力图"""
    img = image.copy()
    h, w = img.shape[:2]  # h=240, w=320

    # 合并所有热力图
    combined_hm = np.zeros_like(heatmaps[0])  # shape (60, 80) = (h, w)
    for hm in heatmaps:
        combined_hm = np.maximum(combined_hm, hm)

    # 归一化到 0-255
    if combined_hm.max() > 0:
        combined_hm = (combined_hm / combined_hm.max() * 255).astype(np.uint8)

    # 应用颜色映射
    heatmap_color = cv2.applyColorMap(combined_hm, cv2.COLORMAP_JET)
    # heatmap_color shape: (60, 80, 3)

    # 调整热力图大小到原始图像尺寸 (h=240, w=320)
    # cv2.resize expects (width, height), so (w, h) = (320, 240)
    heatmap_resized = cv2.resize(heatmap_color, (w, h))

    # 叠加
    output = cv2.addWeighted(img, 1 - alpha, heatmap_resized, alpha, 0)

    return output


def process_image(model, image_path, device='cpu', output_path=None, show_heatmap=False):
    """处理单张图像并可视化"""
    # 预处理（letterbox resize，与训练一致）
    img_tensor, letterbox_img_bgr, original_size, letterbox_params = preprocess_image(image_path)
    orig_w, orig_h = original_size  # PIL Image.size returns (width, height)

    # 读取原始图像用于绘制最终结果
    original_img = cv2.imread(image_path)
    if original_img is None:
        original_img = letterbox_img_bgr.copy()

    # 推理
    with torch.no_grad():
        img_tensor = img_tensor.to(device)
        heatmaps = model(img_tensor)  # (1, 7, 60, 80)

    # 提取关键点（letterbox坐标系）
    heatmaps_np = heatmaps[0].cpu().numpy()  # (7, 60, 80)
    keypoints_letterbox, confidences = extract_keypoints_from_heatmap(
        heatmaps_np,
        img_width=320,
        img_height=240
    )

    # 转换到原始图像坐标系
    keypoints_original = letterbox_to_original_coords(keypoints_letterbox, letterbox_params)

    # 绘制结果（在原始图像上）
    result_img = draw_keypoints(original_img, keypoints_original, confidences)

    if show_heatmap:
        heatmap_img = draw_heatmaps(letterbox_img_bgr, heatmaps_np)
        return result_img, heatmap_img, keypoints_original, confidences

    return result_img, keypoints_original, confidences


def main():
    import argparse

    parser = argparse.ArgumentParser(description='可视化坐姿检测结果')
    parser.add_argument('--image', '-i', type=str, required=True,
                       help='输入图像路径')
    parser.add_argument('--checkpoint', '-c', type=str,
                       default='checkpoints/best_model.pth',
                       help='模型检查点路径')
    parser.add_argument('--output', '-o', type=str, default=None,
                       help='输出图像路径')
    parser.add_argument('--heatmap', action='store_true',
                       help='同时显示热力图')
    parser.add_argument('--save', action='store_true',
                       help='保存结果图像')
    parser.add_argument('--device', type=str, default='cpu',
                       choices=['cpu', 'cuda'],
                       help='推理设备')

    args = parser.parse_args()

    # 检查图像是否存在
    if not os.path.exists(args.image):
        print(f"错误: 图像不存在: {args.image}")
        return

    # 加载模型
    print(f"加载模型: {args.checkpoint}")
    model = load_model(args.checkpoint, device=args.device)

    # 处理图像
    print(f"处理图像: {args.image}")

    if args.heatmap:
        result_img, heatmap_img, keypoints, confidences = process_image(
            model, args.image, device=args.device, show_heatmap=True
        )

        # 并排显示
        combined = np.hstack([result_img, heatmap_img])

        # 添加标签
        cv2.putText(combined, "Keypoint Detection", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(combined, "Heatmap Overlay", (330, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    else:
        result_img, keypoints, confidences = process_image(
            model, args.image, device=args.device, show_heatmap=False
        )
        combined = result_img

    # 打印关键点信息
    print("\n关键点检测结果:")
    print("-" * 50)
    for i, name in enumerate(KEYPOINT_NAMES):
        x, y = keypoints[i]
        conf = confidences[i]
        print(f"{name:16s}: ({x:6.1f}, {y:6.1f})  置信度: {conf:.3f}")

    # 保存或显示
    if args.save or args.output:
        output_path = args.output or args.image.replace('.', '_result.')
        cv2.imwrite(output_path, combined)
        print(f"\n结果已保存: {output_path}")

    # 显示图像
    cv2.imshow('Pose Detection Result', combined)
    print("\n按任意键关闭...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
