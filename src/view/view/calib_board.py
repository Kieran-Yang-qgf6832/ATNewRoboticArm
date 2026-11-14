#!/usr/bin/env python3
"""ChArUco 标靶检测节点：为 eye_on_base 手眼标定提供 `<彩色帧> -> calib_board` 的动态 TF。

一键启动相机 + 检测 + easy_handeye2 采样界面：

    ros2 launch view calib.launch.py

相机已在别处启动时只起节点：

    ros2 run view calib_board_detector

为什么不用 `cv2.aruco.CharucoBoard`：calib.io 的 PDF 白格相位与 OpenCV 内置模板相反
（本板左上角是白格、ID 0 在 (列 0, 行 0)；OpenCV 模板左上角是黑格、ID 0 在列 1），
直接建板会得到 160 px 量级的重投影误差。这里按实物布局自己生成 44 个 marker 的
三维角点，用全部 176 个角点做 solvePnP，实测前视渲染图上 RMS 0.87 px。

板坐标系：原点在板左上角，X 向右、Y 向下、Z 垂直板面向内（与相机光学系同向，
板正对相机时旋转约为单位阵），单位米。

TF 契约：父帧取彩色图像 `header.frame_id`，时间戳用图像 stamp；检测失败时**不发**
——easy_handeye2 取样本时查最新 TF，广播旧值会让它安静地采到错误样本。
同一姿态连续 `gate_frames` 帧（平移 < `gate_translation_mm`、旋转 < `gate_rotation_deg`）
才放行，用来挡误检与机械臂抖动。
"""

from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster

from view import codec
from view.vision import CameraModel, Frame

# realsense2_camera 默认 camera_namespace=camera 且节点名 camera，
# 所以话题是双前缀 /camera/camera/...
DEFAULT_NS = '/camera/camera'

WINDOW_TITLE = 'calib board detector  [q] quit'

# 标定期间更看重低速下的角点质量：关掉快速检测、收紧最小周长
_MIN_MARKER_PERIMETER_RATE = 0.02


@dataclass(frozen=True)
class BoardLayout:
    """实物版面：默认就是 calib.io 200x150 / 8x11 / 15-11 那一张。"""

    squares_x: int = 11
    squares_y: int = 8
    square_mm: float = 15.0
    marker_mm: float = 11.0

    @property
    def width_mm(self) -> float:
        return self.squares_x * self.square_mm

    @property
    def height_mm(self) -> float:
        return self.squares_y * self.square_mm

    def validate(self) -> None:
        """几何必须先自洽，否则生成出来的角点是错的。"""
        if self.squares_x < 2 or self.squares_y < 2:
            raise ValueError('格数至少 2x2，收到 %dx%d'
                             % (self.squares_x, self.squares_y))
        if not 0 < self.marker_mm < self.square_mm:
            raise ValueError('marker 边长必须在 (0, 格边长) 内，收到 %.2f / %.2f'
                             % (self.marker_mm, self.square_mm))


@dataclass(frozen=True)
class BoardPose:
    """板 -> 相机的位姿；rms_px 是这帧的重投影误差。"""

    rotation: np.ndarray        # (3, 3)
    translation: np.ndarray     # (3,)，米
    rms_px: float
    markers: int


@dataclass(frozen=True)
class BoardDetection:
    """一帧的检测结果：位姿 + 原始角点，供预览复用，避免重复检测。"""

    pose: BoardPose
    corners: list
    ids: np.ndarray


def marker_object_points(layout: BoardLayout) -> dict[int, np.ndarray]:
    """按实物布局生成 {marker_id: (4, 3) 角点}。

    ID 从左上角开始按行扫描，只数白格；白格满足 (列 + 行) 为偶数。
    角点顺序与 `detectMarkers` 一致：左上、右上、右下、左下。
    """
    layout.validate()
    half = layout.marker_mm / 2000.0
    square = layout.square_mm / 1000.0
    points: dict[int, np.ndarray] = {}
    for row in range(layout.squares_y):
        for col in range(layout.squares_x):
            if (col + row) % 2:
                continue
            cx = col * square + square / 2.0
            cy = row * square + square / 2.0
            points[len(points)] = np.array([
                [cx - half, cy - half, 0.0],
                [cx + half, cy - half, 0.0],
                [cx + half, cy + half, 0.0],
                [cx - half, cy + half, 0.0],
            ], dtype=np.float32)
    return points


def make_detector_parameters() -> cv2.aruco.DetectorParameters:
    """打开亚像素精化；4.6 只有 create()，4.7+ 两种都有。"""
    create = getattr(cv2.aruco, 'DetectorParameters_create', None)
    params = create() if create is not None else cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 5
    params.cornerRefinementMaxIterations = 50
    params.cornerRefinementMinAccuracy = 0.01
    params.minMarkerPerimeterRate = _MIN_MARKER_PERIMETER_RATE
    return params


def rotation_angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    """两个旋转矩阵之间的夹角（度）。"""
    relative = a.T @ b
    cos = (float(np.trace(relative)) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


def quaternion_from_rotation(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """旋转矩阵 -> (x, y, z, w)。"""
    w = float(np.sqrt(max(0.0, 1.0 + rotation[0, 0] + rotation[1, 1] + rotation[2, 2])) / 2.0)
    x = float(np.sqrt(max(0.0, 1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])) / 2.0)
    y = float(np.sqrt(max(0.0, 1.0 - rotation[0, 0] + rotation[1, 1] - rotation[2, 2])) / 2.0)
    z = float(np.sqrt(max(0.0, 1.0 - rotation[0, 0] - rotation[1, 1] + rotation[2, 2])) / 2.0)
    # 符号由非对角项定，否则会得到 q 与 -q 的歧义
    x = np.copysign(x, rotation[2, 1] - rotation[1, 2])
    y = np.copysign(y, rotation[0, 2] - rotation[2, 0])
    z = np.copysign(z, rotation[1, 0] - rotation[0, 1])
    norm = np.linalg.norm([x, y, z, w])
    return (float(x / norm), float(y / norm), float(z / norm), float(w / norm))


def detect_board(image: np.ndarray, camera: CameraModel,
                 dictionary, object_points: dict[int, np.ndarray],
                 detector_params, min_markers: int) -> BoardDetection | None:
    """检测并解算板位姿；marker 不足或解算失败返回 None。"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary,
                                              parameters=detector_params)
    if ids is None or len(ids) < max(4, min_markers):
        return None

    known = [(int(marker), corner.reshape(4, 2))
             for marker, corner in zip(ids.ravel(), corners)
             if int(marker) in object_points]
    if len(known) < max(4, min_markers):
        return None

    object_array = np.array([object_points[marker] for marker, _ in known],
                            dtype=np.float64).reshape(-1, 3)
    image_array = np.array([corner for _, corner in known],
                           dtype=np.float64).reshape(-1, 2)
    solved, rvec, tvec = cv2.solvePnP(object_array, image_array,
                                      camera.matrix, camera.dist_coeffs,
                                      flags=cv2.SOLVEPNP_IPPE)
    if not solved:
        solved, rvec, tvec = cv2.solvePnP(object_array, image_array,
                                          camera.matrix, camera.dist_coeffs,
                                          flags=cv2.SOLVEPNP_ITERATIVE)
    if not solved:
        return None

    projected, _ = cv2.projectPoints(object_array, rvec, tvec,
                                     camera.matrix, camera.dist_coeffs)
    residual = np.linalg.norm(projected.reshape(-1, 2) - image_array, axis=1)
    rotation, _ = cv2.Rodrigues(rvec)
    pose = BoardPose(rotation=rotation,
                     translation=tvec.reshape(3),
                     rms_px=float(np.sqrt(np.mean(residual ** 2))),
                     markers=len(known))
    return BoardDetection(pose=pose, corners=corners, ids=ids)


class ConsistencyGate:
    """连续若干帧位姿接近才放行；跳变一次就从头计数。

    参考值固定在这段连续帧的**第一帧**而不是上一帧：慢速漂移的误检
    （比如半块板被遮挡，位姿一点点爬）逐帧比较是追不上的。
    """

    def __init__(self, frames: int, translation_m: float, rotation_deg: float):
        self._frames = max(1, int(frames))
        self._translation_m = float(translation_m)
        self._rotation_deg = float(rotation_deg)
        self._anchor: tuple[np.ndarray, np.ndarray] | None = None
        self._count = 0

    @property
    def count(self) -> int:
        return self._count

    def update(self, rotation: np.ndarray, translation: np.ndarray) -> bool:
        if self._anchor is None:
            self._anchor = (rotation.copy(), translation.copy())
            self._count = 1
        else:
            anchor_rotation, anchor_translation = self._anchor
            close = (np.linalg.norm(translation - anchor_translation) <= self._translation_m
                     and rotation_angle_deg(anchor_rotation, rotation) <= self._rotation_deg)
            if close:
                self._count += 1
            else:
                self._anchor = (rotation.copy(), translation.copy())
                self._count = 1
        return self._count >= self._frames


class LatestFrame:
    """后台 spin 线程与主循环之间的最新帧缓存。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._frame: Frame | None = None

    def update(self, frame: Frame) -> None:
        with self._lock:
            self._frame = frame

    def get(self) -> Frame | None:
        with self._lock:
            return self._frame


def stamp_seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def draw_overlay(image: np.ndarray, corners, ids, pose: BoardPose | None,
                 camera: CameraModel, layout: BoardLayout,
                 max_width: int) -> np.ndarray:
    """画检测结果、板坐标轴与重投影误差；纯绘制，不改原图。"""
    canvas = image.copy()
    if ids is not None and len(ids):
        cv2.aruco.drawDetectedMarkers(canvas, corners, ids)
    if pose is not None:
        cv2.drawFrameAxes(canvas, camera.matrix, camera.dist_coeffs,
                          cv2.Rodrigues(pose.rotation)[0], pose.translation.reshape(3, 1),
                          layout.square_mm / 1000.0 * 2.0)
        text = ('markers %d  rms %.2f px' % (pose.markers, pose.rms_px))
        cv2.putText(canvas, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0), 2, cv2.LINE_AA)
        origin = pose.translation
        cv2.putText(canvas, 'z %.3f m' % float(origin[2]), (8, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (215, 215, 215), 1, cv2.LINE_AA)
    else:
        cv2.putText(canvas, 'no board', (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 0, 255), 2, cv2.LINE_AA)
    if max_width and canvas.shape[1] > max_width:
        scale = max_width / float(canvas.shape[1])
        canvas = cv2.resize(canvas, None, fx=scale, fy=scale,
                            interpolation=cv2.INTER_AREA)
    return canvas


class CalibBoardDetector(Node):

    def __init__(self):
        super().__init__('calib_board_detector')

        self.declare_parameter('color_topic', DEFAULT_NS + '/color/image_raw')
        self.declare_parameter('camera_info_topic', DEFAULT_NS + '/color/camera_info')
        self.declare_parameter('parent_frame_fallback', 'camera_color_optical_frame')
        self.declare_parameter('child_frame_id', 'calib_board')
        self.declare_parameter('image_qos', 'sensor')
        # 版面：默认值就是 calib.io 200x150 / 8x11 / 15-11 那一张
        self.declare_parameter('squares_x', 11)
        self.declare_parameter('squares_y', 8)
        self.declare_parameter('square_mm', 15.0)
        self.declare_parameter('marker_mm', 11.0)
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        # 4 是 solvePnP 的下限；再少位姿就不可信了
        self.declare_parameter('min_markers', 4)
        # 只告警不拦截：这是质量指标，不是门禁
        self.declare_parameter('max_reprojection_px', 1.5)
        self.declare_parameter('gate_frames', 3)
        self.declare_parameter('gate_translation_mm', 5.0)
        self.declare_parameter('gate_rotation_deg', 1.0)
        self.declare_parameter('publish_rate', 15.0)
        self.declare_parameter('display', True)
        self.declare_parameter('display_max_width', 1280)

        self._child_frame_id = str(self.get_parameter('child_frame_id').value)
        self._parent_frame_fallback = str(
            self.get_parameter('parent_frame_fallback').value)
        self._max_reprojection_px = float(
            self.get_parameter('max_reprojection_px').value)
        self._publish_rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self._display = bool(self.get_parameter('display').value)
        self._display_max_width = int(self.get_parameter('display_max_width').value)

        self._layout = BoardLayout(
            squares_x=int(self.get_parameter('squares_x').value),
            squares_y=int(self.get_parameter('squares_y').value),
            square_mm=float(self.get_parameter('square_mm').value),
            marker_mm=float(self.get_parameter('marker_mm').value))
        try:
            self._object_points = marker_object_points(self._layout)
        except ValueError as exc:
            self.get_logger().error('版面参数非法: %s，节点退出' % exc)
            raise

        dictionary_name = str(self.get_parameter('dictionary').value)
        if not hasattr(cv2.aruco, dictionary_name):
            self.get_logger().error('未知字典 %s，节点退出' % dictionary_name)
            raise ValueError('unknown aruco dictionary: ' + dictionary_name)
        self._dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, dictionary_name))
        self._detector_params = make_detector_parameters()
        self._min_markers = int(self.get_parameter('min_markers').value)
        self._gate = ConsistencyGate(
            frames=int(self.get_parameter('gate_frames').value),
            translation_m=float(self.get_parameter('gate_translation_mm').value) / 1000.0,
            rotation_deg=float(self.get_parameter('gate_rotation_deg').value))

        qos_name = str(self.get_parameter('image_qos').value).strip().lower()
        if qos_name == 'reliable':
            self._image_qos = QoSProfile(depth=10,
                                         reliability=ReliabilityPolicy.RELIABLE,
                                         history=HistoryPolicy.KEEP_LAST)
        else:
            self._image_qos = qos_profile_sensor_data
            qos_name = 'sensor' if qos_name == 'sensor' else 'sensor（非法值 %r，已回落）' % qos_name

        self._color = LatestFrame()
        self._camera: CameraModel | None = None
        self._warned: set[str] = set()
        self._quality_warned = False
        self._published = False
        self._gui_available = bool(os.environ.get('DISPLAY')
                                   or os.environ.get('WAYLAND_DISPLAY'))
        self._tf_broadcaster = TransformBroadcaster(self)

        color_topic = str(self.get_parameter('color_topic').value)
        info_topic = str(self.get_parameter('camera_info_topic').value)
        self.create_subscription(Image, color_topic, self._on_color, self._image_qos)
        self.create_subscription(CameraInfo, info_topic, self._on_camera_info, 10)

        if self._display and not self._gui_available:
            self.get_logger().warning(
                '没有可用的图形环境（DISPLAY/WAYLAND_DISPLAY 未设置），预览已禁用')
        self.get_logger().info('彩色话题: ' + color_topic)
        self.get_logger().info('相机内参: ' + info_topic)
        self.get_logger().info(
            '标靶: %d 列 x %d 行，格 %.1f mm，marker %.1f mm（%.1f x %.1f mm），%s，%d 个'
            % (self._layout.squares_x, self._layout.squares_y,
               self._layout.square_mm, self._layout.marker_mm,
               self._layout.width_mm, self._layout.height_mm,
               dictionary_name, len(self._object_points)))
        self.get_logger().info('图像 QoS: %s' % qos_name)
        self.get_logger().info(
            'TF: %s -> %s，父帧优先取彩色 header.frame_id；连续 %d 帧一致才发布'
            % (self._parent_frame_fallback, self._child_frame_id,
               int(self.get_parameter('gate_frames').value)))

    # ---------------- 订阅回调 ----------------

    def _on_color(self, msg: Image):
        try:
            image, _ = codec.decode(msg)
        except codec.UnsupportedEncoding as exc:
            self._warn_once('encoding', str(exc))
            return
        self._color.update(Frame(image, stamp_seconds(msg.header.stamp),
                                 1.0, msg.header.frame_id))

    def _on_camera_info(self, msg: CameraInfo):
        matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        coeffs = np.array(msg.d, dtype=np.float64)
        self._camera = CameraModel(
            matrix, coeffs if coeffs.size else np.zeros(5),
            (int(msg.width), int(msg.height)))

    # ---------------- 主循环 ----------------

    def _warn_once(self, key: str, text: str) -> None:
        if key in self._warned:
            return
        self._warned.add(key)
        self.get_logger().warning(text)

    def _stamp_msg(self, color: Frame):
        if color.stamp > 0.0:
            return Time(seconds=color.stamp).to_msg()
        return self.get_clock().now().to_msg()

    def _process(self):
        color = self._color.get()
        if color is None:
            return None
        if self._camera is None:
            self._warn_once('no-intrinsics', '还没收到相机内参，检测暂停')
            return None
        if tuple(self._camera.size) != (color.image.shape[1], color.image.shape[0]):
            self._warn_once(
                'intrinsics-size',
                '内参分辨率 %s 与彩色图 %s 不一致，检测暂停'
                % (self._camera.size, (color.image.shape[1], color.image.shape[0])))
            return None

        detection = detect_board(color.image, self._camera, self._dictionary,
                                 self._object_points, self._detector_params,
                                 self._min_markers)
        if detection is None:
            self._published = False
            return color, None

        pose = detection.pose
        if pose.rms_px > self._max_reprojection_px and not self._quality_warned:
            self._quality_warned = True
            self.get_logger().warning(
                '重投影 %.2f px 超过 %.2f px：先量一下板子实际边长是否 165 mm，'
                '再看打印是否 100%% 缩放' % (pose.rms_px, self._max_reprojection_px))
        elif pose.rms_px <= self._max_reprojection_px:
            self._quality_warned = False

        if not self._gate.update(pose.rotation, pose.translation):
            self._published = False
            return color, detection

        header = TransformStamped().header
        header.stamp = self._stamp_msg(color)
        header.frame_id = color.frame_id or self._parent_frame_fallback
        qx, qy, qz, qw = quaternion_from_rotation(pose.rotation)
        transform = TransformStamped()
        transform.header = header
        transform.child_frame_id = self._child_frame_id
        transform.transform.translation.x = float(pose.translation[0])
        transform.transform.translation.y = float(pose.translation[1])
        transform.transform.translation.z = float(pose.translation[2])
        transform.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        self._tf_broadcaster.sendTransform(transform)

        if not self._published:
            self._published = True
            self.get_logger().info(
                '标靶已锁定：%d 个 marker，重投影 %.2f px，距离 %.3f m'
                % (pose.markers, pose.rms_px, float(pose.translation[2])))
        return color, detection

    def _preview_enabled(self) -> bool:
        return self._display and self._gui_available

    def _show(self, result) -> None:
        if result is None:
            return
        color, detection = result
        if detection is None:
            canvas = draw_overlay(color.image, None, None, None, self._camera,
                                  self._layout, self._display_max_width)
        else:
            canvas = draw_overlay(color.image, detection.corners, detection.ids,
                                  detection.pose, self._camera, self._layout,
                                  self._display_max_width)
        cv2.imshow(WINDOW_TITLE, canvas)

    def _spin_worker(self, executor):
        try:
            executor.spin()
        except (ExternalShutdownException, KeyboardInterrupt):
            pass

    def run(self):
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(self)
        spin_thread = threading.Thread(target=self._spin_worker, args=(executor,),
                                       daemon=True)
        spin_thread.start()
        try:
            while rclpy.ok():
                started = time.monotonic()
                result = self._process()
                if self._preview_enabled():
                    try:
                        self._show(result)
                    except cv2.error as exc:
                        self.get_logger().error('OpenCV 窗口不可用，关闭预览: %s' % exc)
                        self._gui_available = False
                    else:
                        pressed = cv2.waitKey(1) & 0xFF
                        if pressed in (ord('q'), 27):
                            break
                elapsed = time.monotonic() - started
                time.sleep(max(0.0, 1.0 / self._publish_rate - elapsed))
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            spin_thread.join(timeout=1.0)
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = CalibBoardDetector()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
