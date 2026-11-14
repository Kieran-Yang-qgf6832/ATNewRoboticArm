"""标靶版面的纯算法测试：不启动 ROS 节点，只校验几何与门禁。

版面常数来自 calib.io_CHARUCO_200x150_8x11_15_11_DICT_4X4.pdf 的实测值
（200 dpi 渲染后量角点中心得到），改动这些数必须同时改 PDF 校验记录。
"""

import numpy as np
import pytest

from view import calib_board


def test_layout_defaults_match_pdf():
    layout = calib_board.BoardLayout()
    assert (layout.squares_x, layout.squares_y) == (11, 8)
    assert layout.width_mm == pytest.approx(165.0)
    assert layout.height_mm == pytest.approx(120.0)


def test_marker_count_and_first_id():
    points = calib_board.marker_object_points(calib_board.BoardLayout())
    # 11x8 = 88 格，白格 44 个，与 PDF 检出的 marker 数一致
    assert len(points) == 44
    # 左上角是白格，ID 0 在 (列 0, 行 0) 的中心
    first = points[0] * 1000.0
    np.testing.assert_allclose(first.mean(axis=0), [7.5, 7.5, 0.0], atol=1e-6)


def test_marker_corner_order_is_tl_tr_br_bl():
    points = calib_board.marker_object_points(calib_board.BoardLayout())
    corners = points[0] * 1000.0
    np.testing.assert_allclose(corners[0], [2.0, 2.0, 0.0], atol=1e-6)    # 左上
    np.testing.assert_allclose(corners[1], [13.0, 2.0, 0.0], atol=1e-6)   # 右上
    np.testing.assert_allclose(corners[2], [13.0, 13.0, 0.0], atol=1e-6)  # 右下
    np.testing.assert_allclose(corners[3], [2.0, 13.0, 0.0], atol=1e-6)   # 左下


def test_marker_ids_are_row_major_over_white_squares():
    points = calib_board.marker_object_points(calib_board.BoardLayout())
    # 第 0 行白格在偶数列：ID 0..5 的 x 依次为 7.5 + 30k
    for col, marker_id in enumerate(range(6)):
        x = points[marker_id].mean(axis=0)[0] * 1000.0
        assert x == pytest.approx(7.5 + 30.0 * col)
    # 第 1 行白格在奇数列：ID 6 从 22.5 mm 起
    assert points[6].mean(axis=0)[0] * 1000.0 == pytest.approx(22.5)
    assert points[6].mean(axis=0)[1] * 1000.0 == pytest.approx(22.5)


def test_layout_rejects_impossible_marker_size():
    with pytest.raises(ValueError):
        calib_board.marker_object_points(
            calib_board.BoardLayout(square_mm=15.0, marker_mm=15.0))


def test_quaternion_from_rotation_identity():
    x, y, z, w = calib_board.quaternion_from_rotation(np.eye(3))
    assert (x, y, z, w) == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_quaternion_from_rotation_quarter_turn_about_z():
    rotation = np.array([[0.0, -1.0, 0.0],
                         [1.0, 0.0, 0.0],
                         [0.0, 0.0, 1.0]])
    x, y, z, w = calib_board.quaternion_from_rotation(rotation)
    assert (x, y, z, w) == pytest.approx((0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)))


def test_gate_requires_consecutive_frames():
    gate = calib_board.ConsistencyGate(frames=3, translation_m=0.005,
                                       rotation_deg=1.0)
    pose = (np.eye(3), np.array([0.0, 0.0, 0.5]))
    assert not gate.update(*pose)
    assert not gate.update(*pose)
    assert gate.update(*pose)
    # 通过之后再跳变一次，必须从头计数
    moved = (np.eye(3), np.array([0.05, 0.0, 0.5]))
    assert not gate.update(*moved)
    assert not gate.update(*moved)
    assert gate.update(*moved)


def test_gate_ignores_small_jitter_but_resets_on_rotation():
    gate = calib_board.ConsistencyGate(frames=2, translation_m=0.005,
                                       rotation_deg=1.0)
    gate.update(np.eye(3), np.zeros(3))
    assert gate.update(np.eye(3), np.array([0.002, 0.0, 0.0]))
    angle = np.radians(3.0)
    rotated = np.array([[np.cos(angle), -np.sin(angle), 0.0],
                        [np.sin(angle), np.cos(angle), 0.0],
                        [0.0, 0.0, 1.0]])
    assert not gate.update(rotated, np.zeros(3))
