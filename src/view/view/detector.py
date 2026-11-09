#!/usr/bin/env python3
"""红色色块实时检测节点：HSV 二值化 -> 四边形拟合 -> PnP 求解位姿 -> 叠加绘制。

一键启动相机 + 检测节点：

    ros2 launch view view.launch.py

相机已在别处启动时只起节点：

    ros2 run view red_quad_detector

阈值全部是 ROS 参数，红色跨色相 0 度只需一组数（下限大于上限即按环绕处理），
运行中可直接改，改完立即生效：

    ros2 param set /red_quad_detector h_low 165
    ros2 param set /red_quad_detector h_high 12

四个角点按 square_size_mm（默认 350mm）的正方形求 PnP，板坐标系取原点在板中心、
X 朝板右、Y 朝板上、Z 垂直板面向外；重投影误差直接标在框上，板子不是正方形或
角点顺序错了，这个值会立刻变大。

窗口左边是叠加了四边形与位姿坐标轴的原图，右边是二值 mask，按 q 退出。

目标发布与预览是否开启无关（无头运行也照常发布）：

    ros2 topic echo /target_face
    ros2 run tf2_ros tf2_echo <彩色 frame_id> target_face

单面模式发布 quad #0 的面心，双面/三面模式发布法线最正对相机的那个面心；
目标帧 +Z = -normal（指向箱内、远离相机），体心只在 TargetFace 里作诊断。
下游必须先看 TargetFace.valid 与时间戳新鲜度，再决定能不能用于运动。
"""

from __future__ import annotations

import os
import threading
import time

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.exceptions import InvalidParameterTypeException
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion, TransformStamped, Vector3
from robot_msgs.msg import TargetFace
from tf2_ros import TransformBroadcaster

from view import codec, render, target, vision
from view import config as config_module
from view.config import DetectorConfig, ParameterError
from view.target import TrackerState, TargetSample
from view.vision import CameraModel, Frame

# realsense2_camera 默认 camera_namespace=camera 且节点名 camera，
# 所以话题是双前缀 /camera/camera/...
DEFAULT_NS = '/camera/camera'

# 降级原因 -> 终端提示；只在状态翻转时各打一条，不刷屏
ISSUE_TEXT = {
    vision.NO_DEPTH: '还没有对齐深度帧，暂时只画四边形',
    vision.DEPTH_STALE: '深度帧比彩色帧旧太多，距离暂停',
    vision.DEPTH_SHAPE: '对齐深度与彩色分辨率不一致，距离暂停',
    vision.NO_INTRINSICS: '还没收到相机内参，PnP 暂停',
    vision.INTRINSICS_SIZE: '内参分辨率与彩色图不一致，PnP 暂停',
    vision.PNP_FAILED: 'PnP 解算失败，该目标的位姿留空',
}


class LatestFrame:
    """后台 spin 线程与主循环之间的最新帧缓存。

    图像与时间戳必须成对读写，否则主循环可能拿到新图配旧时间戳，
    进而让深度帧的过期判断用错基准。
    """

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


class RedQuadDetector(Node):

    def __init__(self):
        super().__init__('red_quad_detector')

        # 话题名只在订阅时用一次，启动后改不了，因此不进 DetectorConfig
        self.declare_parameter('color_topic', DEFAULT_NS + '/color/image_raw')
        self.declare_parameter('camera_info_topic', DEFAULT_NS + '/color/camera_info')
        self.declare_parameter('aligned_depth_topic',
                               DEFAULT_NS + '/aligned_depth_to_color/image_raw')
        # 帧名、话题名与轴向约定是启动后固定的字符串参数，不进 FIELDS
        self.declare_parameter('parent_frame_fallback', 'camera_color_optical_frame')
        self.declare_parameter('child_frame_id', 'target_face')
        self.declare_parameter('target_topic', 'target_face')
        self.declare_parameter('camera_axis', 'optical')
        # 仿真里必须为 true：图像的 header.stamp 本身就是仿真时间，直接拿它发 TF，
        # 不要依赖 use_sim_time。原因：mujoco_ros2_control 以**物理频率（500 Hz）**
        # 发布 /clock，rclpy 的 TimeSource 订阅它时仅这一项就吃掉约一个核（实测
        # use_sim_time=true 时空节点 106% CPU、false 时 1%），叠加 GIL 争用后检测
        # 主循环从 30 Hz 掉到 1.3 Hz。实机用默认 false 即可。
        self.declare_parameter('stamp_from_image', False)
        # 图像订阅 QoS：sensor（默认，best-effort，与 RealSense 驱动一致）或
        # reliable。MuJoCo 相机插件用 depth=1 的 reliable 发布，best-effort 订阅
        # 实测只能收到 4.7~6.9 Hz（reliable 14.5 Hz），深度帧因此常常超过
        # max_depth_age 被判过期，分面退化为单面。仿真 launch 传 reliable。
        self.declare_parameter('image_qos', 'sensor')
        self._parent_frame_fallback = str(
            self.get_parameter('parent_frame_fallback').value)
        self._child_frame_id = str(self.get_parameter('child_frame_id').value)
        target_topic = str(self.get_parameter('target_topic').value)
        self._camera_axis = str(self.get_parameter('camera_axis').value)
        self._stamp_from_image = bool(
            self.get_parameter('stamp_from_image').value)
        qos_name = str(self.get_parameter('image_qos').value).strip().lower()
        if qos_name == 'reliable':
            self._image_qos = QoSProfile(depth=10,
                                         reliability=ReliabilityPolicy.RELIABLE,
                                         history=HistoryPolicy.KEEP_LAST)
        elif qos_name == 'sensor':
            self._image_qos = qos_profile_sensor_data
        else:
            self._image_qos = qos_profile_sensor_data
            qos_name = 'sensor（非法值 %r，已回落）' % qos_name
        # 覆盖值的类型必须与声明一致，否则 rclpy 直接抛异常、节点连启动日志都打不出来
        # （launch 曾把 min_area 当 INTEGER 传给声明成 DOUBLE 的参数）。类型不符就丢弃
        # 该覆盖、回落默认值，与 config_from_values 的“非法项不阻塞启动”保持同一策略。
        rejected: list[str] = []
        values: dict[str, object] = {}
        for name, spec in config_module.FIELDS.items():
            try:
                self.declare_parameter(name, spec.default)
            except InvalidParameterTypeException as exc:
                self.declare_parameter(name, spec.default, ignore_override=True)
                rejected.append(
                    '%s 需要 %s，覆盖值类型不符，已回落默认值 %r（%s）'
                    % (name, spec.kind.__name__, spec.default, exc))
                continue
            values[name] = self.get_parameter(name).value
        self._config, problems = config_module.config_from_values(values)
        problems = rejected + problems

        self._color = LatestFrame()
        self._depth = LatestFrame()
        self._camera: CameraModel | None = None
        self._issues: set[str] = set()
        self._bad_encodings: set[str] = set()
        self._warnings: set[str] = set()
        self._target_valid: bool | None = None
        self._gui_available = bool(os.environ.get('DISPLAY')
                                   or os.environ.get('WAYLAND_DISPLAY'))
        self._tracker = target.TargetTracker()
        self._target_pub = self.create_publisher(TargetFace, target_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        if self._camera_axis != 'optical':
            self.get_logger().error(
                'camera_axis 只支持 optical（收到 %s），目标 TF 已禁用'
                % self._camera_axis)

        color_topic = self.get_parameter('color_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        depth_topic = self.get_parameter('aligned_depth_topic').value
        self.create_subscription(Image, color_topic, self._on_color,
                                 self._image_qos)
        self.create_subscription(Image, depth_topic, self._on_depth,
                                 self._image_qos)
        self.create_subscription(CameraInfo, info_topic, self._on_camera_info, 10)
        self.add_on_set_parameters_callback(self._on_parameters)

        for problem in problems:
            self.get_logger().error('启动参数被拒绝，改用默认值: ' + problem)
        if self._config.display and not self._gui_available:
            self.get_logger().warning(
                '没有可用的图形环境（DISPLAY/WAYLAND_DISPLAY 未设置），预览已禁用')
        self.get_logger().info('彩色话题: ' + color_topic)
        self.get_logger().info('相机内参: ' + info_topic)
        self.get_logger().info('对齐深度: ' + depth_topic)
        self.get_logger().info('图像 QoS: %s；图像时间戳作为唯一时间基准: %s'
                               % (qos_name, self._stamp_from_image))
        self.get_logger().info(config_module.describe(self._config))
        self.get_logger().info('目标帧: %s -> %s，消息: %s，父帧优先取彩色 header.frame_id'
                               % (self._parent_frame_fallback,
                                  self._child_frame_id, target_topic))
        self.get_logger().info('窗口内按 q 退出（无图形环境时检测与发布照常运行）')

    # ---------------- 参数 ----------------

    def _on_parameters(self, parameters) -> SetParametersResult:
        """整批校验：任一项非法则本次设置整体不生效。"""
        changes = {param.name: param.value for param in parameters
                   if param.name in config_module.FIELDS}
        if not changes:
            return SetParametersResult(successful=True)
        try:
            updated = config_module.apply_overrides(self._config, changes)
        except ParameterError as exc:
            self.get_logger().error('参数被拒绝: %s' % exc)
            return SetParametersResult(successful=False, reason=str(exc))
        self._config = updated
        self.get_logger().info(config_module.describe(updated))
        return SetParametersResult(successful=True)

    # ---------------- 订阅回调 ----------------

    def _on_color(self, msg: Image):
        try:
            image, _ = codec.decode(msg)
        except codec.UnsupportedEncoding as exc:
            self._warn_encoding(str(exc))
            return
        self._color.update(Frame(image, stamp_seconds(msg.header.stamp),
                                 1.0, msg.header.frame_id))

    def _on_depth(self, msg: Image):
        try:
            image, scale = codec.decode(msg)
        except codec.UnsupportedEncoding as exc:
            self._warn_encoding(str(exc))
            return
        self._depth.update(Frame(image, stamp_seconds(msg.header.stamp), scale))

    def _warn_encoding(self, text: str) -> None:
        """同一种不支持的编码只报一次，避免每帧刷屏。"""
        if text in self._bad_encodings:
            return
        self._bad_encodings.add(text)
        self.get_logger().error(text)

    def _on_camera_info(self, msg: CameraInfo):
        matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        coeffs = np.array(msg.d, dtype=np.float64)
        # 整体替换成新对象，主循环读到的要么是旧内参要么是新内参
        self._camera = CameraModel(
            matrix,
            coeffs if coeffs.size else np.zeros(5),
            (int(msg.width), int(msg.height)))

    # ---------------- 检测与发布 ----------------

    def _process(self) -> tuple[Frame, vision.Detection, TrackerState] | None:
        color = self._color.get()
        if color is None:
            self._publish_no_color()
            return None
        detection = vision.detect(color, self._depth.get(), self._camera,
                                  self._config)
        self._report_issues(detection.issues)
        sample = target.sample_from_detection(detection,
                                              self._config.square_size_mm)
        state = self._tracker.update(
            sample, color.stamp,
            switch_frames=self._config.target_switch_frames,
            angle_margin_deg=self._config.target_angle_margin_deg,
            center_margin_m=self._config.target_center_margin_mm / 1000.0,
            incidence_margin_deg=self._config.target_incidence_margin_deg,
            lost_timeout=self._config.target_lost_timeout_ms / 1000.0)
        self._publish_target(color, state)
        return color, detection, state

    def _report_issues(self, issues: tuple[str, ...]) -> None:
        current = set(issues)
        for issue in sorted(current - self._issues):
            self.get_logger().warning(ISSUE_TEXT.get(issue, issue))
        for issue in sorted(self._issues - current):
            self.get_logger().info('已恢复: ' + ISSUE_TEXT.get(issue, issue))
        self._issues = current

    def _warn_once(self, key: str, text: str) -> None:
        if key in self._warnings:
            return
        self._warnings.add(key)
        self.get_logger().warning(text)

    def _stamp_msg(self, color: Frame):
        """TF 与消息统一用彩色帧 stamp；为 0 或明显偏离节点时钟时回落 now()。

        stamp_from_image=True 时无条件用图像时间戳（仿真时间），既不读节点时钟，
        也不做漂移检查——图像头就是唯一时间基准。
        """
        if self._stamp_from_image and color.stamp > 0.0:
            return Time(seconds=color.stamp).to_msg()
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9
        if (color.stamp <= 0.0
                or abs(color.stamp - now_s) > self._config.max_stamp_drift_s):
            self._warn_once(
                'stamp-drift',
                '彩色帧时间戳 %.3f 为 0 或与节点时钟偏差过大，目标改用节点时钟'
                % color.stamp)
            return now.to_msg()
        return Time(seconds=color.stamp).to_msg()

    def _empty_message(self, header: Header, issues: tuple[str, ...]) -> TargetFace:
        msg = TargetFace()
        msg.header = header
        msg.child_frame_id = self._child_frame_id
        msg.valid = False
        msg.mode = TargetFace.MODE_NONE
        msg.issues = list(issues)
        return msg

    def _publish_no_color(self) -> None:
        """还没有彩色帧：清空锁定，只发一条 valid=false 让下游知道节点活着。"""
        if not self._config.publish_target:
            return
        self._tracker.reset()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._parent_frame_fallback
        self._target_pub.publish(self._empty_message(header, ('no-color',)))
        self._report_target_state(False)

    @staticmethod
    def _transform_from_sample(sample: TargetSample, header: Header,
                               child_frame_id: str) -> TransformStamped:
        tf_msg = TransformStamped()
        tf_msg.header.stamp = header.stamp
        tf_msg.header.frame_id = header.frame_id
        tf_msg.child_frame_id = child_frame_id
        x, y, z = (float(value) for value in sample.center)
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = z
        qx, qy, qz, qw = target.quaternion_from_rotation(sample.rotation)
        tf_msg.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        return tf_msg

    def _message_from_state(self, state: TrackerState, header: Header,
                            valid: bool) -> TargetFace:
        sample = state.sample
        if sample is None:
            return self._empty_message(header, ('no-target',))
        msg = TargetFace()
        msg.header = header
        msg.child_frame_id = self._child_frame_id
        msg.valid = valid
        msg.mode = {
            'single': TargetFace.MODE_SINGLE,
            'dual': TargetFace.MODE_DUAL,
            'triple': TargetFace.MODE_TRIPLE,
        }.get(sample.mode, TargetFace.MODE_NONE)
        x, y, z = (float(value) for value in sample.center)
        msg.pose.position = Point(x=x, y=y, z=z)
        qx, qy, qz, qw = target.quaternion_from_rotation(sample.rotation)
        msg.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        msg.normal = Vector3(x=float(sample.normal[0]),
                             y=float(sample.normal[1]),
                             z=float(sample.normal[2]))
        msg.size = Vector3(x=float(sample.width), y=float(sample.height), z=0.0)
        msg.reprojection_error_px = float(sample.reprojection_error_px)
        msg.plane_rms_m = float(sample.plane_rms_m)
        msg.incidence_deg = float(sample.incidence_deg)
        msg.position_source = {
            'pnp': TargetFace.POS_PNP,
            'depth': TargetFace.POS_DEPTH_CORRECTED,
            'cloud': TargetFace.POS_DEPTH_CLOUD,
        }.get(sample.position_source, TargetFace.POS_PNP)
        msg.hysteresis_count = min(255, max(0, int(state.hysteresis_count)))
        issues = list(sample.issues)
        if not valid:
            issues.append('invalid')
        msg.issues = issues
        if sample.box_center is not None and sample.box_size is not None:
            msg.has_box = True
            bx, by, bz = (float(v) for v in sample.box_center)
            sx, sy, sz = (float(v) for v in sample.box_size)
            msg.box_center = Point(x=bx, y=by, z=bz)
            msg.box_size = Vector3(x=sx, y=sy, z=sz)
        return msg

    def _publish_target(self, color: Frame, state: TrackerState) -> None:
        if not self._config.publish_target:
            return
        header = Header()
        header.stamp = self._stamp_msg(color)
        header.frame_id = color.frame_id or self._parent_frame_fallback
        axis_ok = self._camera_axis == 'optical'
        parent_ok = bool(header.frame_id)
        if not axis_ok:
            self._warn_once('camera-axis', '相机轴向不是 optical，目标 TF 已禁用')
        if not parent_ok:
            self._warn_once('parent-frame',
                            '彩色帧没有 frame_id 且未配置兜底父帧，目标 TF 已禁用')
        valid = bool(state.valid and axis_ok and parent_ok)

        # 同帧先发 TF 再发消息；失效短超时内仍广播上一帧 TF，但消息 valid=false
        if state.publish_tf and state.sample is not None and axis_ok and parent_ok:
            tf_msg = self._transform_from_sample(state.sample, header,
                                                 self._child_frame_id)
            self._tf_broadcaster.sendTransform(tf_msg)
        self._target_pub.publish(self._message_from_state(state, header, valid))
        self._report_target_state(valid)

    def _report_target_state(self, valid: bool) -> None:
        if self._target_valid == valid:
            return
        self._target_valid = valid
        if valid:
            self.get_logger().info('目标 %s 已有效' % self._child_frame_id)
        else:
            self.get_logger().warning('目标 %s 已失效（valid=false）'
                                      % self._child_frame_id)

    # ---------------- 主循环 ----------------

    def _preview_enabled(self) -> bool:
        return self._gui_available and self._config.display

    def _show(self, result) -> None:
        if result is None:
            cv2.imshow(render.WINDOW_TITLE, render.waiting())
            return
        color, detection, state = result
        axis_length = max(self._config.square_size_mm / 1000.0 * 0.5, 0.01)
        canvas = render.compose(color.image, detection, self._camera,
                                axis_length, self._config.display_max_width,
                                target_sample=state.sample,
                                target_valid=state.valid)
        cv2.imshow(render.WINDOW_TITLE, canvas)

    def _spin_worker(self, executor):
        try:
            executor.spin()
        except (ExternalShutdownException, KeyboardInterrupt):
            pass

    def run(self):
        # 接收放在后台线程，否则渲染/按键会阻塞回调，帧率被压下来
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(self)
        spin_thread = threading.Thread(target=self._spin_worker, args=(executor,),
                                       daemon=True)
        spin_thread.start()
        try:
            while rclpy.ok():
                started = time.monotonic()
                # 检测与发布不依赖预览：无图形环境时也必须持续对外输出
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
                time.sleep(max(0.0, 1.0 / self._config.publish_rate - elapsed))
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            spin_thread.join(timeout=1.0)
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = RedQuadDetector()
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
