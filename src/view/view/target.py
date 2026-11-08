"""视觉目标面的纯算法：选面、法线姿态、深度校正与锁定滞回。

不 import rclpy 或任何 ROS 消息，节点只把这里的结果编组成 TF 与 TargetFace。
约定：
- 所有坐标都在相机光学系（X 右、Y 下、Z 沿光轴朝前），单位米。
- normal 是面外单位法线，朝相机（normal·center < 0）；
  目标帧的 +Z = -normal，即指向箱内、远离相机，供末端工具对齐。
- 面上坐标系的 Y 取图像竖直向上在面内的投影，X = Y×Z，退化时用 X_cam 投影回退。
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from view import planes
from view.vision import Detection

# 光学系里图像竖直向上是 -Y；法线与它接近平行时改用图像右方向
CAMERA_UP = np.array([0.0, -1.0, 0.0])
CAMERA_RIGHT = np.array([1.0, 0.0, 0.0])
MIN_PROJECTION = 1e-6
MIN_NORM = 1e-9


@dataclass(frozen=True)
class TargetSample:
    """一帧选出的目标面，节点据此发布 TF 与消息。"""

    center: np.ndarray           # (3,) 面心，相机系
    normal: np.ndarray           # (3,) 单位法线，朝相机
    rotation: np.ndarray         # (3, 3) 列向量为 X/Y/Z，Z = -normal
    width: float
    height: float
    mode: str                    # single / dual / triple
    reprojection_error_px: float
    plane_rms_m: float
    incidence_deg: float
    position_source: str         # pnp / depth / cloud
    face_index: int              # 分面模式选中的面序号；单面为 -1
    box_center: np.ndarray | None
    box_size: np.ndarray | None
    issues: tuple[str, ...]


@dataclass(frozen=True)
class TrackerState:
    """节点每帧的发布决策。"""

    sample: TargetSample | None  # 用于 TF 与消息的位姿；None 表示完全失效
    valid: bool                  # 下游门禁
    hysteresis_count: int
    publish_tf: bool


def _unit(vector: np.ndarray) -> np.ndarray:
    vector = np.asarray(vector, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(vector))
    return vector / norm if norm > MIN_NORM else np.array([0.0, 0.0, 1.0])


def _any_perp(axis: np.ndarray) -> np.ndarray:
    """取一个与 axis 垂直的单位向量。"""
    helper = CAMERA_UP if abs(float(axis[1])) < 0.9 else CAMERA_RIGHT
    return _unit(np.cross(axis, helper))


def rotation_from_normal(normal: np.ndarray) -> np.ndarray:
    """由朝相机的法线构造面上坐标系，列向量为 X/Y/Z。

    X = Y×Z，故 X×Y = Z，是右手系；法线接近竖直时 Y 的投影退化，
    改用图像右方向在面内的投影，保证任何视角都能给出确定姿态。
    """
    z = -_unit(normal)
    y = CAMERA_UP - z * float(z @ CAMERA_UP)
    if float(np.linalg.norm(y)) < MIN_PROJECTION:
        y = CAMERA_RIGHT - z * float(z @ CAMERA_RIGHT)
    if float(np.linalg.norm(y)) < MIN_PROJECTION:
        y = _any_perp(z)
    y = _unit(y)
    x = _unit(np.cross(y, z))
    y = np.cross(z, x)           # 重新正交化，消掉浮点误差
    return np.column_stack([x, y, z])


def quaternion_from_rotation(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """旋转矩阵转 (x, y, z, w) 四元数；不引入 tf_transformations 依赖。"""
    m = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    m00, m01, m02 = m[0]
    m10, m11, m12 = m[1]
    m20, m21, m22 = m[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)
        q = np.array([(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s])
    elif m00 > m11 and m00 > m22:
        s = 2.0 * np.sqrt(1.0 + m00 - m11 - m22)
        q = np.array([0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s])
    elif m11 > m22:
        s = 2.0 * np.sqrt(1.0 + m11 - m00 - m22)
        q = np.array([(m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s])
    else:
        s = 2.0 * np.sqrt(1.0 + m22 - m00 - m11)
        q = np.array([(m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s])
    q = q / float(np.linalg.norm(q))
    if q[3] < 0.0:              # 统一 w >= 0，避免同一姿态两种表示来回跳
        q = -q
    return float(q[0]), float(q[1]), float(q[2]), float(q[3])


def incidence_of(center: np.ndarray, normal: np.ndarray) -> float:
    return planes.incidence_degrees(center, normal)


def plane_rms(face: planes.Face) -> float:
    """面内点到拟合平面的 RMS 距离（米），作为分面模式的质量指标。"""
    points = face.plane.points
    if len(points) == 0:
        return 0.0
    residual = points @ face.plane.normal + face.plane.offset
    return float(np.sqrt(np.mean(residual * residual)))


def select_face_index(faces: tuple[planes.Face, ...]) -> int:
    """选法线与视线夹角最小（最正对相机）的面，而不是深度最近的面。

    两面共棱时中心距离几乎相同，按深度排序极易逐帧翻转；入射角差异更大，
    而且越正对的面的点云越完整，面心与法线越可信。
    """
    angles = [incidence_of(face.center, face.z_axis) for face in faces]
    return int(np.argmin(angles))


def correct_center(tvec: np.ndarray, distance: float | None
                   ) -> tuple[np.ndarray, str]:
    """用 quad 中心深度中值校正 PnP 平移，返回 (中心, 位置来源)。

    PnP 把四角当成 square_size_mm 的正方形来解，tvec 的 Z 与假定边长成正比；
    深度图给的是沿光轴的 Z，所以按 Z 的比例缩放整个 tvec，方向与 X/Y 保持不变。
    """
    tvec = np.asarray(tvec, dtype=np.float64).reshape(3)
    if distance is None or not np.isfinite(distance) or distance <= 0.0:
        return tvec, 'pnp'
    if tvec[2] <= MIN_NORM:
        return tvec, 'pnp'
    return tvec * (float(distance) / tvec[2]), 'depth'


def sample_from_detection(detection: Detection,
                          square_size_mm: float) -> TargetSample | None:
    """从一帧检测结果里取出要发布的目标面；无可用位姿时返回 None。

    分面模式：选最正对相机的可见面，中心与法线来自真实尺度点云。
    单面模式：取面积最大的 quad #0，中心由深度中值校正 PnP，法线取板 Z。
    """
    box = detection.box
    box_center = None if box is None else np.asarray(box.center, dtype=np.float64)
    box_size = None if box is None else np.asarray(box.size, dtype=np.float64)

    if detection.mode in ('dual', 'triple') and detection.faces:
        index = select_face_index(detection.faces)
        face = detection.faces[index]
        center = np.asarray(face.center, dtype=np.float64).reshape(3)
        normal = _unit(face.z_axis)
        if float(normal @ center) > 0.0:
            normal = -normal
        return TargetSample(
            center=center, normal=normal,
            rotation=rotation_from_normal(normal),
            width=float(face.width), height=float(face.height),
            mode=detection.mode, reprojection_error_px=0.0,
            plane_rms_m=plane_rms(face),
            incidence_deg=incidence_of(center, normal),
            position_source='cloud', face_index=index,
            box_center=box_center, box_size=box_size,
            issues=tuple(detection.issues))

    if not detection.quads or not detection.poses or detection.poses[0] is None:
        return None
    pose = detection.poses[0]
    rotation, _ = cv2.Rodrigues(pose.rvec)
    normal = _unit(rotation[:, 2])
    distance = detection.distances[0] if detection.distances else None
    center, source = correct_center(pose.tvec, distance)
    if float(normal @ center) > 0.0:
        normal = -normal
    side = float(square_size_mm) / 1000.0
    return TargetSample(
        center=center, normal=normal,
        rotation=rotation_from_normal(normal),
        width=side, height=side, mode='single',
        reprojection_error_px=float(pose.error), plane_rms_m=0.0,
        incidence_deg=incidence_of(center, normal),
        position_source=source, face_index=-1,
        box_center=None, box_size=None,
        issues=tuple(detection.issues))


def same_face(first: TargetSample, second: TargetSample,
              angle_margin_deg: float, center_margin_m: float) -> bool:
    """判定两帧目标是不是同一个物理面，用于跨模式切换的几何一致性检查。"""
    cosine = abs(float(_unit(first.normal) @ _unit(second.normal)))
    angle = float(np.degrees(np.arccos(float(np.clip(cosine, 0.0, 1.0)))))
    if angle > angle_margin_deg:
        return False
    return float(np.linalg.norm(first.center - second.center)) <= center_margin_m


class TargetTracker:
    """目标面锁定与失效门禁。

    - 同一物理面：逐帧跟随更新（含 single↔dual 切换，靠几何一致性确认）。
    - 换成另一面：只有新面入射角连续 N 帧明显更优才切换，否则维持已锁定的 TF。
    - 检测失败：短超时内继续广播上一帧 TF 但把 valid 置 false；
      超过超时就停发 TF 并清空锁定，避免 tf2 缓冲里的旧变换被误用。
    """

    def __init__(self):
        self._locked: TargetSample | None = None
        self._candidate_count = 0
        self._lost_since: float | None = None

    def reset(self) -> None:
        self._locked = None
        self._candidate_count = 0
        self._lost_since = None

    def update(self, sample: TargetSample | None, stamp: float, *,
               switch_frames: int = 5,
               angle_margin_deg: float = 20.0,
               center_margin_m: float = 0.08,
               incidence_margin_deg: float = 5.0,
               lost_timeout: float = 0.2) -> TrackerState:
        if sample is None:
            if self._locked is None:
                return TrackerState(None, False, 0, False)
            if self._lost_since is None:
                self._lost_since = stamp
            if max(0.0, stamp - self._lost_since) <= lost_timeout:
                return TrackerState(self._locked, False, 0, True)
            self.reset()
            return TrackerState(None, False, 0, False)

        self._lost_since = None
        if self._locked is None:
            self._locked = sample
            return TrackerState(sample, True, 0, True)

        if same_face(sample, self._locked, angle_margin_deg, center_margin_m):
            self._locked = sample
            self._candidate_count = 0
            return TrackerState(sample, True, 0, True)

        # sample 已经是当前帧入射角最小的面；锁定面若仍是同一面会在上面被
        # same_face 捕获，所以「连续 N 帧不同面」本身就说明锁定面不再最正对。
        # 这里不能用「候选比锁定时的旧 incidence 更优」当门槛：箱体转动后
        # 锁定面的入射角会变大，旧值却停在很小，会把真正更正对的面永远挡住。
        # margin 只用来挡住明显更掠射的噪声候选面。
        worse = (sample.incidence_deg
                 > self._locked.incidence_deg + incidence_margin_deg)
        self._candidate_count = 0 if worse else self._candidate_count + 1
        if self._candidate_count >= max(1, int(switch_frames)):
            self._locked = sample
            self._candidate_count = 0
            return TrackerState(sample, True, 0, True)
        return TrackerState(self._locked, True, self._candidate_count, True)
