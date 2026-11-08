"""节点层冒烟：直接注入一帧，验证无头发布、TF/消息同 stamp 与失效门禁。"""

import cv2
import numpy as np
import pytest
import rclpy
from robot_msgs.msg import TargetFace
from sensor_msgs.msg import CameraInfo, Image

from view.detector import RedQuadDetector

WIDTH, HEIGHT = 640, 480
K = [610.0, 0.0, 320.0, 0.0, 612.0, 240.0, 0.0, 0.0, 1.0]
BOARD = np.array([[200, 80], [430, 140], [400, 330], [170, 260]], np.int32)


class _Capture:
    def __init__(self):
        self.messages = []
        self.transforms = []

    def publish(self, msg):
        self.messages.append(msg)

    def sendTransform(self, transform):      # noqa: N802 - tf2_ros 接口名
        self.transforms.append(transform)


@pytest.fixture(scope='module')
def detector():
    if not rclpy.ok():
        rclpy.init()
    node = RedQuadDetector()
    capture = _Capture()
    node._target_pub = capture
    node._tf_broadcaster = capture
    node._gui_available = False           # 模拟无图形环境
    yield node, capture
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _image_msg(node, array, encoding):
    msg = Image()
    msg.height, msg.width = array.shape[:2]
    msg.encoding = encoding
    msg.step = msg.width * (3 if encoding == 'bgr8' else 2)
    msg.data = np.ascontiguousarray(array).tobytes()
    msg.header.frame_id = 'test_optical_frame'
    msg.header.stamp = node.get_clock().now().to_msg()
    return msg


def _camera_info():
    info = CameraInfo()
    info.width, info.height = WIDTH, HEIGHT
    info.k = K
    info.d = [0.0] * 5
    return info


def _quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _inject_frame(node):
    bgr = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
    cv2.fillPoly(bgr, [BOARD], (0, 0, 255))
    depth = np.full((HEIGHT, WIDTH), 500, np.uint16)
    node._on_camera_info(_camera_info())
    node._on_depth(_image_msg(node, depth, '16UC1'))
    node._on_color(_image_msg(node, bgr, 'bgr8'))


def test_headless_publishes_matching_tf_and_message(detector):
    node, capture = detector
    _inject_frame(node)
    assert not node._preview_enabled()
    assert node._process() is not None
    assert len(capture.messages) == 1
    assert len(capture.transforms) == 1
    msg, transform = capture.messages[0], capture.transforms[0]

    assert isinstance(msg, TargetFace)
    assert msg.valid
    assert msg.mode == TargetFace.MODE_SINGLE
    assert msg.header.frame_id == transform.header.frame_id == 'test_optical_frame'
    assert msg.child_frame_id == transform.child_frame_id == 'target_face'
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (
        transform.header.stamp.sec, transform.header.stamp.nanosec)
    assert (msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w) == (
        transform.transform.rotation.x, transform.transform.rotation.y,
        transform.transform.rotation.z, transform.transform.rotation.w)
    assert msg.pose.position.z == pytest.approx(0.5, abs=0.02)
    assert msg.position_source == TargetFace.POS_DEPTH_CORRECTED
    normal = np.array([msg.normal.x, msg.normal.y, msg.normal.z])
    assert np.allclose(_quat_matrix(msg.pose.orientation)[:, 2], -normal,
                       atol=1e-6)


def test_no_color_only_emits_invalid_message(detector):
    node, capture = detector
    capture.messages.clear()
    capture.transforms.clear()
    node._publish_no_color()
    assert len(capture.messages) == 1
    assert not capture.messages[0].valid
    assert capture.messages[0].mode == TargetFace.MODE_NONE
    assert capture.transforms == []


def test_non_optical_axis_disables_tf_but_still_reports_invalid(detector):
    node, capture = detector
    _inject_frame(node)
    capture.messages.clear()
    capture.transforms.clear()
    original = node._camera_axis
    node._camera_axis = 'ros'
    try:
        assert node._process() is not None
    finally:
        node._camera_axis = original
    assert len(capture.messages) == 1
    assert len(capture.transforms) == 0
    assert not capture.messages[0].valid
    assert 'invalid' in capture.messages[0].issues
