#!/usr/bin/env python3
"""双面分割的合成数据自检。

按给定箱体位姿渲染一张彩色图与对齐深度图（红色箱体、已知真值），
再跑 vision.detect，直接打印面位姿、箱体中心与尺寸相对真值的误差。
不依赖相机与 ROS。

用法（在 src/view 下）：

    python3 scripts/dual_face_synth.py
    python3 scripts/dual_face_synth.py --yaw -40 --x 0.45 --size 0.4
"""

from __future__ import annotations

import argparse
import itertools
import os
import sys

import cv2
import numpy as np

# 脚本放在 scripts/ 下，把上一级（包根）加进 sys.path 才能 import view
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from view import target, vision               # noqa: E402
from view.config import DetectorConfig        # noqa: E402

WIDTH, HEIGHT = 640, 480
INTRINSICS = dict(fx=608.24, fy=607.68, cx=334.65, cy=238.16)

# 箱体在自身坐标系下的六个面：法线与四个角（顺序不重要，用于投影填充）
FACES = {
    'front': (np.array([0.0, 0.0, -1.0]),
              np.array([[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1]], float)),
    'back': (np.array([0.0, 0.0, 1.0]),
             np.array([[1, -1, 1], [-1, -1, 1], [-1, 1, 1], [1, 1, 1]], float)),
    'left': (np.array([-1.0, 0.0, 0.0]),
             np.array([[-1, -1, -1], [-1, 1, -1], [-1, 1, 1], [-1, -1, 1]], float)),
    'right': (np.array([1.0, 0.0, 0.0]),
              np.array([[1, -1, 1], [1, 1, 1], [1, 1, -1], [1, -1, -1]], float)),
    'bottom': (np.array([0.0, -1.0, 0.0]),
               np.array([[-1, -1, -1], [-1, -1, 1], [1, -1, 1], [1, -1, -1]], float)),
    'top': (np.array([0.0, 1.0, 0.0]),
            np.array([[-1, 1, 1], [-1, 1, -1], [1, 1, -1], [1, 1, 1]], float)),
}


def rotation_y(degrees: float) -> np.ndarray:
    c, s = np.cos(np.radians(degrees)), np.sin(np.radians(degrees))
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rotation_x(degrees: float) -> np.ndarray:
    c, s = np.cos(np.radians(degrees)), np.sin(np.radians(degrees))
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def render_box(size: float, rotation: np.ndarray, center: np.ndarray,
               depth_noise: float = 0.001,
               seed: int = 0) -> tuple[np.ndarray, np.ndarray]:
    """渲染红色箱体，返回 (bgr, depth_mm)。"""
    rng = np.random.default_rng(seed)
    half = size / 2.0
    color = np.full((HEIGHT, WIDTH, 3), 40, np.uint8)
    depth = np.zeros((HEIGHT, WIDTH), np.float32)

    fx, fy = INTRINSICS['fx'], INTRINSICS['fy']
    cx, cy = INTRINSICS['cx'], INTRINSICS['cy']
    us, vs = np.meshgrid(np.arange(WIDTH), np.arange(HEIGHT))
    rays = np.stack([(us - cx) / fx, (vs - cy) / fy, np.ones_like(us, float)],
                    axis=-1)

    for normal_box, corners_box in FACES.values():
        normal = rotation @ normal_box
        corners = (corners_box * half) @ rotation.T + center
        face_center = corners.mean(axis=0)
        # 只画法线朝相机的面，和真实相机看到的一致
        if float(normal @ (np.zeros(3) - face_center)) <= 0.0:
            continue
        offset = -float(normal @ face_center)
        # 先除深度再加主点，否则主点会被 1/z 放大
        scaled = corners @ np.array([[fx, 0.0, 0.0],
                                     [0.0, fy, 0.0],
                                     [0.0, 0.0, 1.0]])
        polygon = np.round(scaled[:, :2] / scaled[:, 2:3]
                           + np.array([cx, cy])).astype(np.int32)
        face_mask = np.zeros((HEIGHT, WIDTH), np.uint8)
        cv2.fillPoly(face_mask, [polygon], 255)

        denominator = rays @ normal
        valid = (face_mask > 0) & (denominator < -1e-6)
        lam = -offset / denominator
        z = lam[valid]
        keep = z > 0.05
        coords = np.argwhere(valid)[keep]
        depth[coords[:, 0], coords[:, 1]] = z[keep]
        color[coords[:, 0], coords[:, 1]] = (0, 0, 220)

    depth += rng.normal(scale=depth_noise, size=depth.shape) * (depth > 0)
    depth_mm = np.clip(depth * 1000.0, 0, 65535).astype(np.uint16)
    return color, depth_mm


def check_target(detection, size_m: float):
    """算法层验收：目标姿态 +Z = -n、右手正交，且选中入射角最小的面。"""
    sample = target.sample_from_detection(detection, size_m * 1000.0)
    if sample is None:
        return False, '无可用目标（PnP 失败）'
    problems = []
    rotation = sample.rotation
    if not np.allclose(rotation[:, 2], -sample.normal, atol=1e-6):
        problems.append('目标 +Z 不等于 -normal')
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        problems.append('目标姿态不是正交右手系')
    if not np.all(np.isfinite(sample.center)):
        problems.append('目标面心含非有限值')
    if detection.mode in ('dual', 'triple') and detection.faces:
        incidences = [target.incidence_of(face.center, face.z_axis)
                      for face in detection.faces]
        if sample.face_index != int(np.argmin(incidences)):
            problems.append('没有选中法线最正对相机的面')
        if sample.plane_rms_m > 0.01:
            problems.append('选中面的平面 RMS 超过 10mm')
    if problems:
        return False, '；'.join(problems)
    return True, ('%s 面心=%s mm 入射角=%.1f° +Z=-n 已验证'
                  % (sample.mode,
                     np.round(sample.center * 1000.0).astype(int),
                     sample.incidence_deg))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='双面分割合成自检')
    parser.add_argument('--size', type=float, default=0.35, help='箱体边长，米')
    parser.add_argument('--x', type=float, default=0.0, help='箱体中心 x，米')
    parser.add_argument('--y', type=float, default=0.00, help='箱体中心 y，米')
    parser.add_argument('--z', type=float, default=0.75, help='箱体中心 z，米')
    parser.add_argument('--yaw', type=float, default=-25.0, help='绕相机 Y 轴转角，度')
    parser.add_argument('--pitch', type=float, default=0.0,
                        help='绕相机 X 轴转角，度；非零会露出顶面（三面可见）')
    parser.add_argument('--noise', type=float, default=1.0, help='深度噪声标准差，毫米')
    parser.add_argument('--seed', type=int, default=0, help='噪声种子')
    parser.add_argument('--gate', type=float, default=65.0,
                        help='第三面入射角门槛，度；调大可强制三面融合')
    parser.add_argument('--save', default='', help='可选：把渲染图写到该路径')
    args = parser.parse_args(argv)

    rotation = rotation_y(args.yaw) @ rotation_x(args.pitch)
    center = np.array([args.x, args.y, args.z])
    color, depth = render_box(args.size, rotation, center,
                              depth_noise=args.noise / 1000.0,
                              seed=args.seed)
    red = (color[:, :, 2] > 150) & (color[:, :, 1] < 80)
    if (red[:2].any() or red[-2:].any() or red[:, :2].any()
            or red[:, -2:].any()):
        print('警告: 目标触到画面边缘，面尺寸会被截断，测量值偏小')
    if args.save:
        cv2.imwrite(args.save, color)

    camera = vision.CameraModel(
        np.array([[INTRINSICS['fx'], 0.0, INTRINSICS['cx']],
                  [0.0, INTRINSICS['fy'], INTRINSICS['cy']],
                  [0.0, 0.0, 1.0]]),
        np.zeros(5), (WIDTH, HEIGHT))
    config = DetectorConfig(face_max_incidence_deg=args.gate)
    detection = vision.detect(vision.Frame(color, 0.0),
                              vision.Frame(depth, 0.0, 0.001), camera, config)

    print('模式: %s  面数: %d' % (detection.mode, len(detection.faces)))
    target_ok, target_text = check_target(detection, args.size)
    print('目标: %s' % target_text)
    for index, face in enumerate(detection.faces, start=1):
        normal = face.z_axis
        angle = np.degrees(np.arccos(np.clip(abs(float(normal @
                                                       np.array([0.0, 0.0, 1.0]))),
                                            0.0, 1.0)))
        print('  F%d 中心=%s mm 尺寸=%.0fx%.0f mm 法线=%s 与光轴夹角=%.1f°'
              % (index, np.round(face.center * 1000.0).astype(int),
                 face.width * 1000.0, face.height * 1000.0,
                 np.round(normal, 3), angle))

    if detection.box is None:
        print('单面回退：未找到两个相邻面（第二面太窄或内点不足时属正常）')
        print('判定(目标姿态): %s' % ('通过' if target_ok else '失败'))
        return 0 if target_ok else 1

    box = detection.box
    center_error = np.linalg.norm(box.center - center) * 1000.0
    print('箱体中心=%s mm  %s 误差=%.1f mm'
          % (np.round(box.center * 1000.0).astype(int),
             np.round(center * 1000.0).astype(int), center_error))
    print('箱体尺寸=%s mm 误差=%s mm'
          % (np.round(box.size * 1000.0).astype(int),
             np.round((box.size - args.size) * 1000.0).astype(int)))

    axes = np.column_stack([box.x_axis, box.y_axis, box.z_axis])
    truth = rotation
    # X 沿共用棱、Y/Z 沿两面法线，和箱体自身轴只是置换关系；哪个面被当成 A
    # 会改变置换顺序，所以按最优置换比对，只关心误差大小
    best = None
    for mapping in itertools.permutations(range(3)):
        errors = []
        for ours, theirs in enumerate(mapping):
            cosine = abs(float(axes[:, ours] @ truth[:, theirs]))
            errors.append(float(np.degrees(np.arccos(min(1.0, cosine)))))
        if best is None or max(errors) < max(best[1]):
            best = (mapping, errors)
    mapping, errors = best
    print('三条轴与真值的夹角误差（置换 %s）: %s'
          % (mapping, np.round(errors, 2)))

    ok = (detection.mode in ('dual', 'triple') and center_error < 10.0
          and float(np.max(np.abs(box.size - args.size))) < 0.01
          and max(errors) < 2.0 and target_ok)
    print('判定: %s' % ('通过' if ok else '失败'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
