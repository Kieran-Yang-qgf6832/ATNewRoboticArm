#!/usr/bin/env python3
"""仿真视觉验收：把 /target_face 的检测结果经 TF 转到 base_link，与已知真值比对。

只在 MuJoCo 仿真链路里用（配合 view_sim.launch.py）。它会：

1. 收若干帧有效的 TargetFace；
2. 用 TF 把面心 / 法线 / 体心从相机光学系变换到 base_link——这一步同时验证两段
   静态 TF 是否接对（姿态错 180°、平移错位都会直接体现在误差里）；
3. 与场景里写死的箱体真值比较，打印全部实测值并给出 PASS/FAIL。

    ros2 run view check_sim_vision
    ros2 run view check_sim_vision --ros-args -p samples:=60
    ros2 run view check_sim_vision --ros-args -p box_yaw_deg:=0.0   # 换场景时改真值

退出码：0 = 全部通过，1 = 任一项超差 / 样本不足 / TF 失败。

真值默认对应 arm_model/model/scene_vision.xml：边长 0.35 m 的立方体，中心
(0.8, 0, 0.175)，绕 z 轴逆时针 31°，静态落地。相机能看到两个侧面：
-X' 面（面心入射约 51.3°）与 +Y' 面（约 59.0°），因此要求 mode 必须是 dual。
位置来源应为 cloud（分面点云），单面 PnP 的 depth/pnp 来源说明分面没跑起来。
"""

from __future__ import annotations

import math
import sys
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from robot_msgs.msg import TargetFace
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """四元数转 3x3 旋转矩阵。"""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def transform_point(transform, point) -> np.ndarray:
    rotation = quaternion_matrix(transform.rotation.x, transform.rotation.y,
                                 transform.rotation.z, transform.rotation.w)
    translation = np.array([transform.translation.x, transform.translation.y,
                            transform.translation.z])
    return rotation @ np.asarray(point, dtype=np.float64).reshape(3) + translation


def transform_vector(transform, vector) -> np.ndarray:
    rotation = quaternion_matrix(transform.rotation.x, transform.rotation.y,
                                 transform.rotation.z, transform.rotation.w)
    return rotation @ np.asarray(vector, dtype=np.float64).reshape(3)


def angle_between(first, second) -> float:
    cosine = float(np.clip(np.dot(first, second), -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def expected_faces(center: np.ndarray, size: float, yaw_deg: float):
    """由箱体中心/边长/绕 z 转角推出两个可见面的 (面心, 外法线)。"""
    half = size / 2.0
    yaw = math.radians(yaw_deg)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    normals = [np.array([-cosine, -sine, 0.0]),    # 面 A：箱体 -X'
               np.array([-sine, cosine, 0.0])]     # 面 B：箱体 +Y'
    return [(center + half * normal, normal) for normal in normals]


class SimVisionCheck(Node):

    def __init__(self):
        super().__init__('check_sim_vision')
        self.declare_parameter('target_topic', 'target_face')
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('box_center', [0.8, 0.0, 0.175])
        self.declare_parameter('box_size', 0.35)
        self.declare_parameter('box_yaw_deg', 31.0)
        self.declare_parameter('samples', 30)
        self.declare_parameter('timeout', 20.0)
        self.declare_parameter('tolerance_box_center_m', 0.015)
        self.declare_parameter('tolerance_face_center_m', 0.020)
        self.declare_parameter('tolerance_normal_deg', 3.0)
        self.declare_parameter('tolerance_size_m', 0.015)

        self._parent = str(self.get_parameter('parent_frame').value)
        self._wanted = int(self.get_parameter('samples').value)
        self._timeout = float(self.get_parameter('timeout').value)
        self._box_center = np.array(
            self.get_parameter('box_center').value, dtype=np.float64)
        self._box_size = float(self.get_parameter('box_size').value)
        self._faces = expected_faces(
            self._box_center, self._box_size,
            float(self.get_parameter('box_yaw_deg').value))
        self._tol_box = float(self.get_parameter('tolerance_box_center_m').value)
        self._tol_face = float(self.get_parameter('tolerance_face_center_m').value)
        self._tol_normal = float(self.get_parameter('tolerance_normal_deg').value)
        self._tol_size = float(self.get_parameter('tolerance_size_m').value)

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._samples: list[dict] = []
        self._invalid = 0
        self._no_box = 0
        self._tf_failures = 0
        self._tf_error = ''

        topic = str(self.get_parameter('target_topic').value)
        self.create_subscription(TargetFace, topic, self._on_target, 10)
        self.get_logger().info(
            '等待 %d 帧有效目标（话题 %s，超时 %.0f s，父帧 %s）'
            % (self._wanted, topic, self._timeout, self._parent))

    # ---------------- 回调 ----------------

    def _on_target(self, msg: TargetFace) -> None:
        if not msg.valid:
            self._invalid += 1
            return
        if not msg.has_box:
            self._no_box += 1
            return
        frame = msg.header.frame_id
        if not frame:
            self._tf_failures += 1
            return
        try:
            # 静态 TF 对任意时刻有效；用最新时刻避免仿真时间与墙钟的口径问题
            transform = self._buffer.lookup_transform(
                self._parent, frame, rclpy.time.Time())
        except TransformException as exc:
            # 启动初期 TF 还没进缓冲是正常现象，不在每条消息上刷警告；
            # 真正接错时最终报告会给出失败计数和最后一次异常
            self._tf_failures += 1
            self._tf_error = str(exc)
            return

        transform = transform.transform
        position = np.array([msg.pose.position.x, msg.pose.position.y,
                             msg.pose.position.z])
        normal = np.array([msg.normal.x, msg.normal.y, msg.normal.z])
        box_center = np.array([msg.box_center.x, msg.box_center.y,
                               msg.box_center.z])
        box_size = np.array([msg.box_size.x, msg.box_size.y, msg.box_size.z])

        self._samples.append({
            'mode': msg.mode,
            'face_center': transform_point(transform, position),
            'normal': transform_vector(transform, normal),
            'box_center': transform_point(transform, box_center),
            'box_size': box_size,
            'rms': float(msg.plane_rms_m),
            'incidence': float(msg.incidence_deg),
            'reprojection': float(msg.reprojection_error_px),
            'source': msg.position_source,
        })

    # ---------------- 判定 ----------------

    def _report(self) -> bool:
        if len(self._samples) < self._wanted:
            print('样本不足：需要 %d 帧带箱体结果的有效目标，实际 %d 帧'
                  '（无效 %d，无箱体 %d，TF 失败 %d）'
                  % (self._wanted, len(self._samples), self._invalid,
                     self._no_box, self._tf_failures))
            if self._tf_failures:
                print('最后一次 TF 失败：%s' % self._tf_error)
            return False

        face_centers = np.array([sample['face_center'] for sample in self._samples])
        normals = np.array([sample['normal'] for sample in self._samples])
        box_centers = np.array([sample['box_center'] for sample in self._samples])
        box_sizes = np.array([sample['box_size'] for sample in self._samples])

        face_center = face_centers.mean(axis=0)
        normal = normals.mean(axis=0)
        normal = normal / max(float(np.linalg.norm(normal)), 1e-12)
        box_center = box_centers.mean(axis=0)
        box_size = box_sizes.mean(axis=0)

        # 双面模式下锁定面可能是两个可见面中的任意一个，按最近面配对
        expected = min(self._faces,
                       key=lambda item: float(np.linalg.norm(item[0] - face_center)))
        expected_center, expected_normal = expected
        face_error = float(np.linalg.norm(face_center - expected_center)) * 1000.0
        normal_error = angle_between(normal, expected_normal)
        box_error = float(np.linalg.norm(box_center - self._box_center)) * 1000.0
        size_errors = np.abs(box_size - self._box_size) * 1000.0

        modes = {}
        for sample in self._samples:
            modes[sample['mode']] = modes.get(sample['mode'], 0) + 1
        dual = modes.get(TargetFace.MODE_DUAL, 0)
        sources = {}
        for sample in self._samples:
            sources[sample['source']] = sources.get(sample['source'], 0) + 1

        print('')
        print('=' * 66)
        print('仿真视觉验收：%d 帧（dual %d，single %d，triple %d），父帧 %s'
              % (len(self._samples), dual, modes.get(TargetFace.MODE_SINGLE, 0),
                 modes.get(TargetFace.MODE_TRIPLE, 0), self._parent))
        print('锁定面: %s（观测面心最近）'
              % ('面 A(-X\')' if expected is self._faces[0] else '面 B(+Y\')'))
        print('-' * 66)
        print('面心误差      %8.2f mm   允许 %6.1f mm'
              % (face_error, self._tol_face * 1000.0))
        print('法线夹角误差  %8.2f deg  允许 %6.1f deg'
              % (normal_error, self._tol_normal))
        print('箱心误差      %8.2f mm   允许 %6.1f mm'
              % (box_error, self._tol_box * 1000.0))
        print('边长误差      %8.2f / %.2f / %.2f mm  允许 %6.1f mm'
              % (size_errors[0], size_errors[1], size_errors[2],
                 self._tol_size * 1000.0))
        print('-' * 66)
        print('观测面心      (%.4f, %.4f, %.4f)' % tuple(face_center))
        print('真值面心      (%.4f, %.4f, %.4f)' % tuple(expected_center))
        print('观测箱心      (%.4f, %.4f, %.4f)' % tuple(box_center))
        print('真值箱心      (%.4f, %.4f, %.4f)' % tuple(self._box_center))
        print('观测边长      (%.4f, %.4f, %.4f)' % tuple(box_size))
        print('平均平面 RMS  %.4f mm' % (1000.0 * np.mean([s['rms'] for s in self._samples])))
        print('平均入射角    %.2f deg' % np.mean([s['incidence'] for s in self._samples]))
        print('位置来源      %s' % ', '.join('%s x%d' % item for item in sources.items()))
        print('=' * 66)

        checks = [
            ('mode 必须为 dual', dual == len(self._samples)),
            ('面心误差', face_error <= self._tol_face * 1000.0),
            ('法线夹角', normal_error <= self._tol_normal),
            ('箱心误差', box_error <= self._tol_box * 1000.0),
            ('边长误差', float(size_errors.max()) <= self._tol_size * 1000.0),
        ]
        failed = [name for name, passed in checks if not passed]
        if failed:
            print('结果: FAIL（%s）' % '、'.join(failed))
        else:
            print('结果: PASS')
        return not failed


def main(args=None) -> int:
    rclpy.init(args=args)
    node = SimVisionCheck()
    try:
        deadline = time.monotonic() + node._timeout
        while rclpy.ok() and len(node._samples) < node._wanted:
            if time.monotonic() > deadline:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        passed = node._report()
    except (KeyboardInterrupt, ExternalShutdownException):
        passed = False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
