#!/usr/bin/env python3
"""手眼标定复测：把标靶固定在桌面上，换多个机械臂姿态看 `base_link -> calib_board` 是否恒定。

用法（先起 `ros2 launch launch_pack handeye.launch.py`，再起检测节点）：

    ros2 run view check_handeye

摆一个姿态停 1 s，再换下一个；相邻样本间隔超过 `gap_s` 就自动归为新的一组。
凑够 `groups` 组后打印每组位姿与组间离散度，并按 `position_tol_mm` / `angle_tol_deg`
给出通过与否（退出码 0 = 通过）。

为什么用这个口径：eye_on_base 下标靶装末端时没有独立的 `ee -> board`，FK 对视觉比不了；
把板子拆下来固定桌面后，标定正确则视觉推出来的板子位姿在 base 系里应当恒定，
漂移量直接就是外参误差。只看 rqt 里的采样残差抓不到轴向或内参这类系统性错误。
"""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def quaternion_to_rotation(quaternion) -> np.ndarray:
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rotation_angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    cos = (float(np.trace(a.T @ b)) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


class Group:
    """一个姿态下连续采到的若干帧。"""

    def __init__(self, number: int, position: np.ndarray, rotation: np.ndarray):
        self.number = number
        self.position = position.copy()      # 该组代表位姿：第一帧
        self.rotation = rotation.copy()
        self.max_position_mm = 0.0
        self.max_angle_deg = 0.0
        self.samples = 1

    def add(self, position: np.ndarray, rotation: np.ndarray) -> None:
        self.samples += 1
        self.max_position_mm = max(
            self.max_position_mm, float(np.linalg.norm(position - self.position)) * 1000.0)
        self.max_angle_deg = max(
            self.max_angle_deg, rotation_angle_deg(self.rotation, rotation))


class HandeyeVerifier(Node):

    def __init__(self):
        super().__init__('handeye_verifier')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('marker_frame', 'calib_board')
        self.declare_parameter('gap_s', 1.0)
        self.declare_parameter('groups', 5)
        self.declare_parameter('position_tol_mm', 5.0)
        self.declare_parameter('angle_tol_deg', 1.0)
        self.declare_parameter('rate', 5.0)

        self._base = str(self.get_parameter('base_frame').value)
        self._marker = str(self.get_parameter('marker_frame').value)
        self._gap = float(self.get_parameter('gap_s').value)
        self._required = int(self.get_parameter('groups').value)
        self._position_tol = float(self.get_parameter('position_tol_mm').value)
        self._angle_tol = float(self.get_parameter('angle_tol_deg').value)
        self._rate = max(0.5, float(self.get_parameter('rate').value))

        self._buffer = Buffer(cache_time=Duration(seconds=5))
        self._listener = TransformListener(self._buffer, self)
        self._groups: list[Group] = []
        self._last_stamp = 0.0
        self._missing = 0

        self.get_logger().info('复测 %s -> %s：摆一个姿态停 %.1f s，凑满 %d 组'
                               % (self._base, self._marker, self._gap, self._required))

    def _lookup(self) -> tuple[np.ndarray, np.ndarray] | None:
        try:
            transform: TransformStamped = self._buffer.lookup_transform(
                self._base, self._marker, Time(), Duration(seconds=0.1))
        except Exception:
            return None
        translation = transform.transform.translation
        position = np.array([translation.x, translation.y, translation.z])
        rotation = quaternion_to_rotation(transform.transform.rotation)
        return position, rotation

    def _record(self, position: np.ndarray, rotation: np.ndarray) -> None:
        now = time.monotonic()
        if self._groups and now - self._last_stamp <= self._gap:
            self._groups[-1].add(position, rotation)
        else:
            self._groups.append(Group(len(self._groups) + 1, position, rotation))
            self.get_logger().info(
                '第 %d 组：位置 (%.4f, %.4f, %.4f) m'
                % (len(self._groups), position[0], position[1], position[2]))
        self._last_stamp = now

    def _report(self) -> bool:
        worst_position = max(g.max_position_mm for g in self._groups)
        worst_angle = max(g.max_angle_deg for g in self._groups)
        # 组间一致性：以第一组的代表位姿为基准，这能抓出“每组内部稳、组间在漂”的情况
        reference = self._groups[0]
        spread_position = max(
            float(np.linalg.norm(g.position - reference.position)) * 1000.0
            for g in self._groups)
        spread_angle = max(
            rotation_angle_deg(reference.rotation, g.rotation) for g in self._groups)

        self.get_logger().info('---- 复测结果（%d 组） ----' % len(self._groups))
        for group in self._groups:
            self.get_logger().info(
                '组 %d：%2d 帧，组内位置抖动 %.1f mm，角度抖动 %.2f°'
                % (group.number, group.samples, group.max_position_mm,
                   group.max_angle_deg))
        self.get_logger().info(
            '组间离散：位置 %.1f mm（上限 %.1f），角度 %.2f°（上限 %.2f）；'
            '组内最差：位置 %.1f mm，角度 %.2f°'
            % (spread_position, self._position_tol, spread_angle, self._angle_tol,
               worst_position, worst_angle))

        passed = (len(self._groups) >= self._required
                  and spread_position <= self._position_tol
                  and spread_angle <= self._angle_tol)
        if passed:
            self.get_logger().info('结论：通过')
        elif len(self._groups) < self._required:
            self.get_logger().warning('结论：组数不足，继续摆姿态')
        else:
            self.get_logger().error('结论：不通过')
            self.get_logger().error(
                '位置超限先怀疑检测：板子是否平、是否 100%% 缩放打印（应 165 mm）；'
                '角度超限先怀疑 frame 方向与标定类型，再看相机内参')
        return passed

    def run(self) -> int:
        period = 1.0 / self._rate
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                sample = self._lookup()
                if sample is None:
                    self._missing += 1
                    if self._missing in (20, 200):
                        self.get_logger().warning(
                            '查不到 %s -> %s：确认 handeye.launch.py 已发布 base->camera，'
                            '且检测节点看到了标靶' % (self._base, self._marker))
                    time.sleep(period)
                    continue
                self._missing = 0
                self._record(*sample)
                if len(self._groups) > self._required:
                    break
                time.sleep(period)
        except KeyboardInterrupt:
            pass
        return 0 if self._report() else 1


def main(args=None):
    rclpy.init(args=args)
    node = HandeyeVerifier()
    code = 1
    try:
        code = node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
