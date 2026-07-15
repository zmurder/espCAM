#!/usr/bin/env python3
"""
pose_model.onnx 离线推理 + 关键点可视化
=========================================
完整复现 ESP32 的检测流程（前处理/后处理），用于精度排查：
  - 若此脚本在测试图上检测准确 → 模型+前处理 OK，ESP32 问题在摄像头输入(角度/位置)
  - 若此脚本也检测差          → 模型或前处理(rgb_swap/归一化)有问题

用法:
  pip install onnxruntime numpy pillow matplotlib
  python visualize_pose.py [图片路径]            # 默认用 received_images 最新一张

前处理(对齐训练 dataset.py / ESP32 pose_inference.cc):
  center_crop 到 4:3 → resize 320x240 → RGB → ImageNet 归一化 → NCHW
后处理: heatmap argmax + conf(直接 fp32 sigmoid 输出)，自适应 NCHW/NHWC 布局
"""
import os
import sys
import glob
import numpy as np
import onnxruntime as ort
from PIL import Image, ImageDraw
import matplotlib.pyplot as plt

MODEL_PATH = "model/pose_model.onnx"
INPUT_H, INPUT_W = 240, 320
IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)   # [0,1]
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)
KP_NAMES = ["left_eye", "right_eye", "left_shoulder", "right_shoulder"]
KP_COLORS = ["red", "blue", "orange", "cyan"]
CONF_THRESH = 0.6


def center_crop_resize(img: Image.Image) -> Image.Image:
    """中心裁剪到 4:3 再 resize 到 320x240，对齐训练 center_crop_resize"""
    w, h = img.size
    target = 4.0 / 3.0  # w/h
    cur = w / h
    if cur > target:        # 太宽，裁左右
        nw = int(round(h * target))
        x0 = (w - nw) // 2
        img = img.crop((x0, 0, x0 + nw, h))
    elif cur < target:      # 太高，裁上下
        nh = int(round(w / target))
        y0 = (h - nh) // 2
        img = img.crop((0, y0, w, y0 + nh))
    return img.resize((INPUT_W, INPUT_H), Image.BILINEAR)


def preprocess(img_path):
    img = Image.open(img_path).convert("RGB")
    img_crp = center_crop_resize(img)
    arr = np.asarray(img_crp, dtype=np.float32) / 255.0          # HWC [0,1]
    arr = (arr - IMAGENET_MEAN) / IMAGENET_STD                   # ImageNet norm
    arr = np.transpose(arr, (2, 0, 1))[None].astype(np.float32)  # 1xCxHxW
    return arr, img_crp


def postprocess(heatmaps):
    """自适应 NCHW [1,4,H,W] 或 NHWC [1,H,W,4]"""
    s = heatmaps.shape
    print(f"  heatmap shape = {s}")
    kps = []
    if len(s) == 4 and s[1] == 4:        # NCHW
        C, H, W = s[1], s[2], s[3]
        for k in range(C):
            hm = heatmaps[0, k]
            idx = int(np.argmax(hm))
            y, x = divmod(idx, W)
            kps.append((x / W, y / H, float(hm[y, x])))
    elif len(s) == 4 and s[-1] == 4:     # NHWC
        H, W, C = s[1], s[2], s[3]
        for k in range(C):
            hm = heatmaps[0, :, :, k]
            idx = int(np.argmax(hm))
            y, x = divmod(idx, W)
            kps.append((x / W, y / H, float(hm[y, x])))
    else:
        raise RuntimeError(f"无法识别的输出布局: {s}")
    return kps


def main():
    # 选图：命令行参数 > received_images 最新 > model/320240.jpg
    if len(sys.argv) > 1:
        img_path = sys.argv[1]
    else:
        cand = sorted(glob.glob("received_images/*.jpg"))
        img_path = cand[-1] if cand else "model/320240.jpg"
    print(f"图片: {img_path}")
    print(f"模型: {MODEL_PATH}")

    sess = ort.InferenceSession(MODEL_PATH, providers=["CPUExecutionProvider"])
    inp = sess.get_inputs()[0]
    out = sess.get_outputs()[0]
    print(f"输入: name={inp.name} shape={inp.shape}")
    print(f"输出: name={out.name} shape={out.shape}")

    x, img_crp = preprocess(img_path)
    heatmaps = sess.run([out.name], {inp.name: x})[0]

    kps = postprocess(heatmaps)
    print("关键点:")
    for i, (xk, yk, c) in enumerate(kps):
        flag = "OK" if c >= CONF_THRESH else "low"
        print(f"  {KP_NAMES[i]:14s} norm=({xk:.3f},{yk:.3f}) conf={c:.3f}  {flag}")

    # 在 resize 后的 320x240 图上画关键点（坐标相对此图）
    disp = img_crp.copy()
    draw = ImageDraw.Draw(disp)
    for i, (xk, yk, c) in enumerate(kps):
        px, py = xk * INPUT_W, yk * INPUT_H
        r = 6 if c >= CONF_THRESH else 4
        draw.ellipse([px - r, py - r, px + r, py + r], fill=KP_COLORS[i], outline="white")
        draw.text((px + 8, py - 8), f"{KP_NAMES[i][:2]}:{c:.2f}", fill="yellow")

    # 同时把 4 点连线画出来（眼-眼，肩-肩）
    le, re, ls, rs = kps
    draw.line([le[0]*INPUT_W, le[1]*INPUT_H, re[0]*INPUT_W, re[1]*INPUT_H], fill="white", width=2)
    draw.line([ls[0]*INPUT_W, ls[1]*INPUT_H, rs[0]*INPUT_W, rs[1]*INPUT_H], fill="white", width=2)

    out_png = "visualize_result.png"
    disp.save(out_png)
    print(f"已保存可视化: {out_png}")

    plt.figure(figsize=(8, 6))
    plt.imshow(disp)
    plt.title("pose_model.onnx keypoints (white=eyes/shoulders link)")
    plt.axis("off")
    try:
        plt.show()
    except Exception:
        print("(无GUI，已保存png)")


if __name__ == "__main__":
    main()
