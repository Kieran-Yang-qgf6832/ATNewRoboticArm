"""目标面发布的纯算法测试：不启动 ROS 节点即可覆盖姿态、选面与锁定。"""

import cv2
import numpy as np

from view import planes, target, vision


def _unit(vector):
    vector = np.asarray(vector, dtype=np.float64)
    return vector / float(np.linalg.norm(vector))


def _face(center, normal, width=0.35, height=0.35):
    """构造一个最小合法 Face；点云只放中心点，RMS 为 0。"""
    center = np.asarray(center, dtype=np.float64)
    normal = _unit(normal)
    rotation = target.rotation_from_normal(normal)
    plane = planes.Plane(
        normal, -float(normal @ center), center.reshape(1, 3),
        np.zeros((1, 2), dtype=np.int32))
    return planes.Face(center, rotation[:, 0], rotation[:, 1], normal,
                       width, height, plane)


def _detection(faces=(), mode='dual', quads=(), distances=(), poses=(), box=None):
    return vision.Detection(
        mask=np.zeros((2, 2), np.uint8), quads=list(quads),
        distances=list(distances), poses=list(poses), issues=(),
        faces=tuple(faces), box=box, mode=mode)


def _sample(mode='single', incidence=10.0, normal=(0.0, 0.0, -1.0),
            center=(0.0, 0.0, 0.7)):
    normal = _unit(normal)
    return target.TargetSample(
        center=np.asarray(center, dtype=np.float64), normal=normal,
        rotation=target.rotation_from_normal(normal),
        width=0.35, height=0.35, mode=mode,
        reprojection_error_px=0.0, plane_rms_m=0.0,
        incidence_deg=incidence, position_source='cloud', face_index=0,
        box_center=None, box_size=None, issues=())


def _single_detection(distance=0.6):
    rotation = np.array([[1.0, 0.0, 0.0],
                         [0.0, -1.0, 0.0],
                         [0.0, 0.0, -1.0]])
    rvec, _ = cv2.Rodrigues(rotation)
    tvec = np.array([[0.05], [-0.02], [0.5]])
    pose = vision.SquarePose(rvec, tvec, 0.4, float(np.linalg.norm(tvec)))
    quad = vision.Quad(0, np.zeros((4, 2), np.float32), 1000.0, (320, 240))
    return _detection(mode='single', quads=[quad], distances=[distance],
                      poses=[pose])


# ---------------- 姿态 ----------------

def test_rotation_z_points_away_from_camera_and_is_right_handed():
    normal = _unit([0.1, -0.05, -1.0])
    rotation = target.rotation_from_normal(normal)
    assert np.allclose(rotation[:, 2], -normal, atol=1e-9)
    assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9)
    assert np.isclose(np.linalg.det(rotation), 1.0)
    assert np.allclose(np.cross(rotation[:, 0], rotation[:, 1]),
                       rotation[:, 2], atol=1e-9)


def test_rotation_handles_degenerate_up_projection():
    normal = _unit([0.0, -1.0, 0.0])   # 与图像上方向平行，投影退化
    rotation = target.rotation_from_normal(normal)
    assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9)
    assert np.allclose(rotation[:, 2], -normal, atol=1e-9)
    assert np.all(np.isfinite(rotation))


def test_quaternion_roundtrip():
    x, y, z, w = target.quaternion_from_rotation(
        target.rotation_from_normal(_unit([0.3, -0.2, -1.0])))
    assert np.isclose(x * x + y * y + z * z + w * w, 1.0)
    assert w >= 0.0
    rebuilt = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    expected = target.rotation_from_normal(_unit([0.3, -0.2, -1.0]))
    assert np.allclose(rebuilt, expected, atol=1e-9)


# ---------------- 选面与中心 ----------------

def test_select_face_prefers_frontal_over_nearer_grazing_face():
    frontal = _face([0.0, 0.0, 0.8], [0.0, 0.0, -1.0])
    # 这个面到光心更近，但明显掠射；按「最近面」会选错
    grazing = _face([0.5, 0.0, 0.6], [-1.0, 0.0, 0.0])
    assert float(np.linalg.norm(grazing.center)) < float(np.linalg.norm(frontal.center))
    assert target.select_face_index((grazing, frontal)) == 1
    sample = target.sample_from_detection(
        _detection(faces=(grazing, frontal)), 350.0)
    assert sample.face_index == 1
    assert np.allclose(sample.rotation[:, 2], -sample.normal, atol=1e-9)
    assert sample.position_source == 'cloud'


def test_correct_center_scales_pnp_by_depth_ratio():
    tvec = np.array([0.05, -0.02, 0.5])
    center, source = target.correct_center(tvec, 0.6)
    assert source == 'depth'
    assert np.allclose(center, tvec * 1.2)
    assert np.isclose(center[2], 0.6)
    fallback, source = target.correct_center(tvec, None)
    assert source == 'pnp'
    assert np.allclose(fallback, tvec)


def test_single_sample_uses_depth_corrected_center_and_pnp_normal():
    sample = target.sample_from_detection(_single_detection(distance=0.6), 350.0)
    assert sample is not None
    assert sample.mode == 'single'
    assert sample.position_source == 'depth'
    assert np.isclose(sample.center[2], 0.6)
    assert np.isclose(sample.center[0], 0.06)
    assert np.allclose(sample.rotation[:, 2], -sample.normal, atol=1e-9)
    assert np.isclose(sample.width, 0.35)
    assert np.isclose(sample.reprojection_error_px, 0.4)


def test_single_sample_without_pose_is_none():
    detection = _detection(mode='single',
                           quads=[vision.Quad(0, np.zeros((4, 2), np.float32),
                                              1000.0, (320, 240))],
                           distances=[0.6], poses=[None])
    assert target.sample_from_detection(detection, 350.0) is None


def test_split_sample_carries_box_diagnostics():
    box = planes.Box(np.array([0.0, 0.0, 0.7]),
                     np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]),
                     np.array([0.0, 0.0, 1.0]), np.array([0.35, 0.35, 0.35]))
    sample = target.sample_from_detection(
        _detection(faces=(_face([0.0, 0.0, 0.7], [0.0, 0.0, -1.0]),),
                   mode='dual', box=box), 350.0)
    assert sample.mode == 'dual'
    assert np.allclose(sample.box_center, box.center)
    assert np.allclose(sample.box_size, box.size)


# ---------------- 锁定与失效 ----------------

def test_tracker_follows_same_face_across_mode_switch():
    tracker = target.TargetTracker()
    first = _sample(mode='single', incidence=12.0)
    second = _sample(mode='dual', incidence=11.0, center=[0.01, 0.0, 0.705])
    assert tracker.update(first, 0.0).valid
    state = tracker.update(second, 0.033)
    assert state.sample is second
    assert state.valid and state.publish_tf


def test_tracker_switches_only_after_enough_better_frames():
    tracker = target.TargetTracker()
    locked = _sample(incidence=40.0, normal=[0.0, 0.0, -1.0])
    candidate = _sample(mode='dual', incidence=5.0,
                        normal=[-1.0, 0.0, 0.0], center=[0.35, 0.0, 0.7])
    tracker.update(locked, 0.0)
    for frame in range(1, 5):
        state = tracker.update(candidate, frame * 0.033, switch_frames=5)
        assert state.sample is locked
        assert state.hysteresis_count == frame
    switched = tracker.update(candidate, 5 * 0.033, switch_frames=5)
    assert switched.sample is candidate
    assert switched.hysteresis_count == 0


def test_tracker_switches_after_locked_face_turns_grazing():
    """锁定时的入射角很小，转动后当前最正对面是另一个面，旧值不能挡住切换。"""
    tracker = target.TargetTracker()
    locked = _sample(incidence=5.0, normal=[0.0, 0.0, -1.0])
    candidate = _sample(mode='dual', incidence=10.0,
                        normal=[-1.0, 0.0, 0.0], center=[0.35, 0.0, 0.7])
    tracker.update(locked, 0.0)
    for frame in range(1, 5):
        state = tracker.update(candidate, frame * 0.033,
                               switch_frames=5, incidence_margin_deg=5.0)
        assert state.sample is locked
    switched = tracker.update(candidate, 5 * 0.033,
                              switch_frames=5, incidence_margin_deg=5.0)
    assert switched.sample is candidate

    strict = target.TargetTracker()
    strict.update(locked, 0.0)
    for frame in range(1, 8):
        rejected = strict.update(candidate, frame * 0.033,
                                 switch_frames=5, incidence_margin_deg=1.0)
        assert rejected.sample is locked


def test_tracker_lost_timeout_stops_tf_then_recovers():
    tracker = target.TargetTracker()
    locked = _sample()
    tracker.update(locked, 0.0)
    stale = tracker.update(None, 0.1, lost_timeout=0.2)
    assert stale.sample is locked and not stale.valid and stale.publish_tf
    stopped = tracker.update(None, 0.5, lost_timeout=0.2)
    assert stopped.sample is None and not stopped.valid and not stopped.publish_tf
    recovered = tracker.update(_sample(incidence=8.0), 0.6)
    assert recovered.valid and recovered.publish_tf
    assert np.isclose(recovered.sample.incidence_deg, 8.0)
