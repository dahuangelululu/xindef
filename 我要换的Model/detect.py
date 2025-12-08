# -*- coding: UTF-8 -*-
"""
detect_gauge.py — 表计/指针场景的纯净版检测脚本
- 去除了所有车牌识别相关依赖（plate_recognition/*）
- 仅保留：检测 → 四点关键点还原 → 角度/数值计算（add.jiaodu.out / out_jiandan）→ 可视化/落盘
- 兼容中文路径读取与目录递归遍历

用法示例：
python detect_gauge.py \
    --detect_model runs/train/exp56/weights/best.pt \
    --image_path dataset/images/val \
    --img_size 640 --conf-thres 0.5 --iou-thres 0.5 \
    --output result1/V4
"""
import argparse
import time
import os
import cv2
import torch
import copy
import numpy as np

from add.jiaodu import *

from models.experimental import attempt_load
from utils.datasets import letterbox
from utils.general import check_img_size, non_max_suppression_face, scale_coords
from utils.plots import plot_one_box
from utils.torch_utils import time_synchronized
from utils.cv_puttext import cv2ImgAddText  # 可选：需要中文标注时使用

# 颜色表（BGR）
CLRS = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255)]


def cv_imread(path):
    """兼容中文路径的 imread"""
    data = np.fromfile(str(path), dtype=np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def all_images(p):
    """递归遍历目录下的所有图片文件；若 p 是单文件则仅返回该文件"""
    from pathlib import Path
    exts = {'.jpg', '.jpeg', '.png', '.bmp'}
    p = Path(p)
    if p.is_file():
        return [str(p)]
    return [str(x) for x in p.rglob('*') if x.suffix.lower() in exts]


def load_model(weights, device):
    return attempt_load(weights, map_location=device)


def scale_coords_landmarks(img1_shape, coords, img0_shape, ratio_pad=None):
    """将网络输入尺度的四点关键点坐标还原到原图尺度（严格四点，8 维）"""
    if ratio_pad is None:
        gain = min(img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1])
        pad = (img1_shape[1] - img0_shape[1] * gain) / 2, (img1_shape[0] - img0_shape[0] * gain) / 2
    else:
        gain = ratio_pad[0][0]
        pad = ratio_pad[1]

    coords[:, [0, 2, 4, 6]] -= pad[0]
    coords[:, [1, 3, 5, 7]] -= pad[1]
    coords[:, :8] /= gain

    coords[:, 0].clamp_(0, img0_shape[1]); coords[:, 1].clamp_(0, img0_shape[0])
    coords[:, 2].clamp_(0, img0_shape[1]); coords[:, 3].clamp_(0, img0_shape[0])
    coords[:, 4].clamp_(0, img0_shape[1]); coords[:, 5].clamp_(0, img0_shape[0])
    coords[:, 6].clamp_(0, img0_shape[1]); coords[:, 7].clamp_(0, img0_shape[0])
    return coords


def dict_from_det(xyxy, landmarks, class_num):
    """把一次检测结果打包为结构化 dict（表计任务不做车牌识别/分割）"""
    x1, y1, x2, y2 = map(int, xyxy)
    rect = [x1, y1, x2, y2]
    pts = np.zeros((4, 2), dtype=np.int32)
    for i in range(4):
        pts[i] = (int(landmarks[2*i]), int(landmarks[2*i + 1]))
    return {'rect': rect, 'landmarks': pts.tolist(), 'class': int(class_num)}


def detect(model, orgimg, device, img_size, conf_thres=0.5, iou_thres=0.5):
    img0 = copy.deepcopy(orgimg)
    assert orgimg is not None, 'Image Not Found'
    h0, w0 = orgimg.shape[:2]
    r = img_size / max(h0, w0)
    if r != 1:
        interp = cv2.INTER_AREA if r < 1 else cv2.INTER_LINEAR
        img0 = cv2.resize(img0, (int(w0 * r), int(h0 * r)), interpolation=interp)

    imgsz = check_img_size(img_size, s=model.stride.max())
    img = letterbox(img0, new_shape=imgsz)[0]
    img = img[:, :, ::-1].transpose(2, 0, 1).copy()  # BGR->RGB, HWC->CHW
    img = torch.from_numpy(img).to(device).float() / 255.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)

    names = model.module.names if hasattr(model, 'module') else model.names

    # 推理
    t1 = time_synchronized()
    pred = model(img)[0]
    t2 = time_synchronized()

    # NMS
    pred = non_max_suppression_face(pred, conf_thres, iou_thres)

    # 绘制到原图
    for det in pred:
        if len(det):
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], orgimg.shape).round()
            det[:, 5:13] = scale_coords_landmarks(img.shape[2:], det[:, 5:13], orgimg.shape).round()

            for j in range(det.size(0)):
                xyxy = det[j, :4].tolist()
                conf = float(det[j, 4].cpu().numpy())
                landmarks = det[j, 5:13].tolist()
                cls = int(det[j, 13].cpu().numpy()) if det.size(1) > 13 else 0

                # 角度/数值计算（使用 add.jiaodu 中的 out/out_jiandan，按你项目实际选择）
                try:
                    angle_val = out(landmarks)  # 若你的实现使用 out_jiandan，可切换为 out_jiandan(landmarks)
                except Exception:
                    angle_val = None

                # 可视化
                label = f"{names[cls] if isinstance(names, (list, tuple)) and 0 <= cls < len(names) else str(cls)} {conf:.2f}"
                if angle_val is not None:
                    label += f" result:{angle_val:.3f}"

                for i in range(4):
                    cv2.circle(orgimg, (int(landmarks[2*i]), int(landmarks[2*i+1])), 4, CLRS[i], -1)
                plot_one_box(xyxy, orgimg, label=label, color=CLRS[cls % len(CLRS)], line_thickness=2)

    return orgimg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--detect_model', type=str, default='runs/train/exp56/weights/best.pt', help='model.pt path')
    parser.add_argument('--image_path', type=str, default='data/circle-312/images/val', help='image or folder')
    parser.add_argument('--img_size', type=int, default=640, help='inference size')
    parser.add_argument('--conf-thres', dest='conf_thres', type=float, default=0.5, help='confidence threshold')
    parser.add_argument('--iou-thres', dest='iou_thres', type=float, default=0.5, help='IoU threshold for NMS')
    parser.add_argument('--output', type=str, default='result1/V4', help='save directory')
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = load_model(args.detect_model, device)

    # 收集图片
    files = all_images(args.image_path)

    count = 0
    total_time = 0.0
    for p in files:
        img = cv_imread(p)
        if img is None:
            print('[WARN] read fail:', p); continue
        if img.ndim == 3 and img.shape[-1] == 4:
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

        t0 = time.time()
        out_img = detect(model, img, device, args.img_size, args.conf_thres, args.iou_thres)
        dt = time.time() - t0
        if count:  # 跳过首帧统计
            total_time += dt

        save_p = os.path.join(args.output, os.path.basename(p))
        cv2.imwrite(save_p, out_img)
        print(f"[{count:04d}] {p} -> {save_p}  time={dt:.3f}s")
        count += 1

    # 简单统计
    if count > 1:
        avg = total_time / (count - 1)
        print(f"sumTime={total_time:.3f}s  avg={avg:.3f}s  FPS={1.0/avg:.2f}")
    else:
        print("processed images:", count)


if __name__ == '__main__':
    main()
