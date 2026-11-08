import cv2
import numpy as np
import pytest

from view import vision
from view.config import DetectorConfig

K = np.array([[610.0, 0.0, 320.0], [0.0, 612.0, 240.0], [0.0, 0.0, 1.0]])
D = np.array([0.1, -0.2, 0.001, 0.0, 0.0])
BOARD = np.array([[200, 80], [430, 140], [400, 330], [170, 260]], np.int32)


def board_image():
    image = np.zeros((480, 640, 3), np.uint8)
    cv2.fillPoly(image, [BOARD], (0, 0, 255))
    return image


def corner_set(points):
    return {tuple(int(v) for v in np.round(p)) for p in np.asarray(points)}


# ---------------- 分割 ----------------

def test_channel_mask_handles_hue_wraparound():
    channel = np.array([[0, 5, 10, 100, 170, 179]], np.uint8)
    mask = vision.channel_mask(channel, 170, 10)
    assert mask.tolist() == [[True, True, True, False, True, True]]


def test_channel_mask_plain_range():
    channel = np.array([[0, 5, 10]], np.uint8)
    assert vision.channel_mask(channel, 2, 8).tolist() == [[False, True, False]]


def test_hsv_mask_finds_red_and_ignores_blue():
    image = board_image()
    cv2.rectangle(image, (0, 0), (60, 60), (255, 0, 0), -1)
    mask = vision.hsv_mask(image, 170, 10, 100, 255, 70, 255)
    assert mask[200, 300] == 255          # 红板上
    assert mask[30, 30] == 0              # 蓝块上


def test_clean_mask_removes_isolated_pixels():
    mask = np.zeros((60, 60), np.uint8)
    cv2.rectangle(mask, (20, 20), (40, 40), 255, -1)
    mask[2, 2] = 255
    cleaned = vision.clean_mask(mask, 5)
    assert cleaned[2, 2] == 0
    assert cleaned[30, 30] == 255


def test_clean_mask_with_zero_kernel_is_identity():
    mask = np.zeros((10, 10), np.uint8)
    mask[2, 2] = 255
    assert np.array_equal(vision.clean_mask(mask, 0), mask)


# ---------------- 几何 ----------------

def test_order_clockwise_starts_at_top_left():
    shuffled = np.array([[400, 300], [100, 100], [100, 300], [400, 100]], np.float32)
    ordered = vision.order_clockwise(shuffled)
    assert tuple(ordered[0]) == (100.0, 100.0)
    # 图像坐标 y 向下时，顺时针多边形的 shoelace 为正
    signed = 0.0
    for index in range(4):
        x1, y1 = ordered[index]
        x2, y2 = ordered[(index + 1) % 4]
        signed += x1 * y2 - x2 * y1
    assert signed > 0


def test_fit_quad_recovers_corners():
    quad = vision.fit_quad(BOARD.reshape(-1, 1, 2))
    assert quad.shape == (4, 2)
    assert corner_set(quad) == corner_set(BOARD)


def test_fit_quad_falls_back_to_rectangle_for_triangle():
    triangle = np.array([[0, 0], [100, 0], [50, 80]], np.int32).reshape(-1, 1, 2)
    quad = vision.fit_quad(triangle)
    assert quad is not None and quad.shape == (4, 2)


def test_detect_quads_sorts_by_area_and_filters():
    mask = np.zeros((480, 640), np.uint8)
    cv2.fillPoly(mask, [BOARD], 255)
    cv2.rectangle(mask, (40, 380), (110, 440), 255, -1)

    quads = vision.detect_quads(mask, 300)
    assert [quad.number for quad in quads] == [0, 1]
    assert quads[0].area > quads[1].area
    assert vision.detect_quads(mask, 1e6) == []


# ---------------- 测距 ----------------

def test_sample_distance_uses_median_of_valid_pixels():
    depth = np.full((20, 20), 500, np.uint16)
    depth[10, 10] = 0            # 无效值不参与
    assert vision.sample_distance(depth, (10, 10), 5, 0.001) == pytest.approx(0.5)


def test_sample_distance_returns_none_without_valid_pixels():
    assert vision.sample_distance(np.zeros((20, 20), np.uint16), (10, 10), 5, 0.001) is None
    assert vision.sample_distance(None, (10, 10), 5, 0.001) is None


def test_usable_depth_reports_reason():
    color = vision.Frame(np.zeros((4, 4, 3), np.uint8), 10.0)
    depth = vision.Frame(np.zeros((4, 4), np.uint16), 10.0, 0.001)
    assert vision.usable_depth(color, None, 0.5) == (None, vision.NO_DEPTH)
    assert vision.usable_depth(color, depth, 0.5) == (depth, None)

    stale = vision.Frame(np.zeros((4, 4), np.uint16), 5.0, 0.001)
    assert vision.usable_depth(color, stale, 0.5)[1] == vision.DEPTH_STALE

    wrong_shape = vision.Frame(np.zeros((5, 4), np.uint16), 10.0, 0.001)
    assert vision.usable_depth(color, wrong_shape, 0.5)[1] == vision.DEPTH_SHAPE


# ---------------- 位姿 ----------------

def test_intrinsics_issue_states():
    camera = vision.CameraModel(K, D, (640, 480))
    assert vision.intrinsics_issue(None, (640, 480)) == vision.NO_INTRINSICS
    assert vision.intrinsics_issue(camera, (640, 480)) is None
    assert vision.intrinsics_issue(camera, (1280, 720)) == vision.INTRINSICS_SIZE


def test_solve_square_pose_roundtrip():
    rvec_true = np.array([[0.12], [0.25], [0.06]])
    tvec_true = np.array([[0.03], [-0.02], [0.5]])
    projected, _ = cv2.projectPoints(vision.square_object_points(0.35), rvec_true,
                                     tvec_true, K, D)
    pose = vision.solve_square_pose(projected.reshape(4, 2), K, D, 0.35)
    assert pose is not None
    assert np.allclose(pose.tvec.ravel(), tvec_true.ravel(), atol=1e-3)
    assert pose.error < 1e-3
    assert pose.distance == pytest.approx(0.5013, abs=1e-3)


def test_solve_square_pose_without_camera_matrix():
    assert vision.solve_square_pose(np.zeros((4, 2)), None, D, 0.35) is None


# ---------------- 端到端 ----------------

def test_detect_on_synthetic_frame():
    camera = vision.CameraModel(K, D, (640, 480))
    color = vision.Frame(board_image(), 10.0)
    depth = vision.Frame(np.full((480, 640), 500, np.uint16), 10.0, 0.001)

    detection = vision.detect(color, depth, camera, DetectorConfig())
    assert len(detection.quads) == 1
    assert detection.issues == ()
    assert detection.distances[0] == pytest.approx(0.5)
    assert detection.poses[0] is not None
    assert detection.mask.shape == (480, 640)


def test_detect_without_depth_and_intrinsics_degrades():
    color = vision.Frame(board_image(), 10.0)
    detection = vision.detect(color, None, None, DetectorConfig())
    assert len(detection.quads) == 1
    assert vision.NO_DEPTH in detection.issues
    assert vision.NO_INTRINSICS in detection.issues
    assert detection.distances == [None]
    assert detection.poses == [None]


def test_detect_reports_stale_depth():
    camera = vision.CameraModel(K, D, (640, 480))
    color = vision.Frame(board_image(), 10.0)
    depth = vision.Frame(np.full((480, 640), 500, np.uint16), 8.0, 0.001)
    detection = vision.detect(color, depth, camera, DetectorConfig())
    assert vision.DEPTH_STALE in detection.issues
    assert detection.distances == [None]


def test_thresholds_control_detection():
    color = vision.Frame(board_image(), 10.0)
    no_red = DetectorConfig(h_low=30, h_high=40)
    assert vision.detect(color, None, None, no_red).quads == []
