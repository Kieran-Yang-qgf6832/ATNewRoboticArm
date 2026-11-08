"""纯视觉算法：HSV 分割、四边形拟合、深度采样、平面 PnP。

本模块只依赖 numpy 与 OpenCV，不 import rclpy 或任何 ROS 消息类型，
因此可以被 pytest 直接驱动。节点只负责把 ROS 数据整理成 Frame 再调 detect()。
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from view import planes
from view.config import DetectorConfig

# 深度帧比彩色帧旧超过这个时间就不参与测距（避免把上一秒的距离标在当前的框上）
DEFAULT_DEPTH_MAX_AGE = 0.5

# 自适应 approxPolyDP：从 2% 周长起步逐步放宽，直到顶点数 <= 4
EPS_START_RATIO = 0.02
EPS_MAX_RATIO = 0.20
EPS_GROWTH = 1.35

# 降级原因，节点据此在状态翻转时打日志
NO_DEPTH = 'no-depth'
DEPTH_STALE = 'depth-stale'
DEPTH_SHAPE = 'depth-shape'
NO_INTRINSICS = 'no-intrinsics'
INTRINSICS_SIZE = 'intrinsics-size'
PNP_FAILED = 'pnp-failed'


@dataclass(frozen=True)
class Frame:
    """一帧图像；stamp 为 ROS 时间换算出的秒，scale 为原值到基本单位的系数。

    frame_id 来自消息头，发布 TF 时直接作为父帧，避免硬编码相机驱动命名。
    """

    image: np.ndarray
    stamp: float
    scale: float = 1.0
    frame_id: str = ''


@dataclass(frozen=True)
class CameraModel:
    """彩色相机内参及其对应的分辨率。"""

    matrix: np.ndarray
    dist_coeffs: np.ndarray
    size: tuple[int, int]


@dataclass(frozen=True)
class Quad:
    number: int
    points: np.ndarray          # (4, 2) float32，顺时针，从左上最近点开始
    area: float
    center: tuple[int, int]


@dataclass(frozen=True)
class SquarePose:
    rvec: np.ndarray            # (3, 1) 板 -> 相机的旋转（Rodrigues）
    tvec: np.ndarray            # (3, 1) 相机系下的平移，单位米
    error: float                # 平均重投影误差，单位像素
    distance: float             # 板中心到相机光心的距离，单位米


@dataclass(frozen=True)
class Detection:
    mask: np.ndarray
    quads: list[Quad]
    distances: list[float | None]
    poses: list[SquarePose | None]
    issues: tuple[str, ...] = ()
    faces: tuple[planes.Face, ...] = ()
    box: planes.Box | None = None
    mode: str = 'single'


def channel_mask(channel: np.ndarray, low: int, high: int) -> np.ndarray:
    """单通道阈值；下限大于上限按色相环绕处理（红色就是 H=170~10）。"""
    if low <= high:
        return (channel >= low) & (channel <= high)
    return (channel >= low) | (channel <= high)


def hsv_mask(bgr: np.ndarray, h_low: int, h_high: int,
             s_low: int, s_high: int, v_low: int, v_high: int) -> np.ndarray:
    """三个通道同时限幅，返回 0/255 的 uint8 掩码。

    用 inRange 而不是逐通道的 numpy 布尔链：后者要对 H/S/V 各比较一次再合并，
    整幅图要扫六七遍（848x480 上约 3ms），inRange 一次遍历就出结果。
    上下限都是闭区间，与 channel_mask 的语义一致。
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    low = np.array([h_low, s_low, v_low], dtype=np.uint8)
    high = np.array([h_high, s_high, v_high], dtype=np.uint8)
    if h_low <= h_high:
        return cv2.inRange(hsv, low, high)
    # 下限大于上限即色相环绕（红色就是 170~10），拆成两段再按位或
    return cv2.bitwise_or(
        cv2.inRange(hsv, low, np.array([179, s_high, v_high], dtype=np.uint8)),
        cv2.inRange(hsv, np.array([0, s_low, v_low], dtype=np.uint8), high))


def clean_mask(mask: np.ndarray, kernel_size: int) -> np.ndarray:
    """开运算去椒盐噪点，再闭运算补反光/阴影造成的缺口；kernel_size<=0 时不处理。"""
    if kernel_size <= 0:
        return mask
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def order_clockwise(points: np.ndarray) -> np.ndarray:
    """顶点排成顺时针，并把最靠左上的点转到第一个（便于四角一一对应）。"""
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    if len(pts) < 3:
        return pts
    center = pts.mean(axis=0)
    # 图像坐标系 y 轴朝下，atan2 角度递增即顺时针
    angle = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
    pts = pts[np.argsort(angle)]
    start = int(np.argmin(pts.sum(axis=1)))
    return np.roll(pts, -start, axis=0)


def fit_quad(contour: np.ndarray) -> np.ndarray | None:
    """凸包 + 自适应 approxPolyDP 拟合四边形；凑不出 4 点时回退最小外接矩形。"""
    hull = cv2.convexHull(contour)
    perimeter = cv2.arcLength(hull, True)
    if perimeter <= 0.0:
        return None

    ratio = EPS_START_RATIO
    approx = None
    while ratio <= EPS_MAX_RATIO:
        approx = cv2.approxPolyDP(hull, ratio * perimeter, True)
        if len(approx) <= 4:
            break
        ratio *= EPS_GROWTH

    if approx is not None and len(approx) == 4:
        quad = approx.reshape(4, 2).astype(np.float32)
    else:
        quad = cv2.boxPoints(cv2.minAreaRect(hull)).astype(np.float32)
    return order_clockwise(quad)


def detect_quads(mask: np.ndarray, min_area: float) -> list[Quad]:
    """按面积降序对每个足够大的连通域拟合四边形并编号。"""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    areas = [(cv2.contourArea(c), c) for c in contours]
    areas = [item for item in areas if item[0] >= min_area]
    areas.sort(key=lambda item: item[0], reverse=True)

    quads = []
    for area, contour in areas:
        points = fit_quad(contour)
        if points is None:
            continue
        cx, cy = points.mean(axis=0)
        quads.append(Quad(len(quads), points, float(area),
                          (int(round(cx)), int(round(cy)))))
    return quads


def sample_distance(depth: np.ndarray | None, center: tuple[int, int],
                    window: int, scale: float) -> float | None:
    """取中心小窗内有效深度的中值（米）；全无效时返回 None。"""
    if depth is None:
        return None
    height, width = depth.shape[:2]
    half = max(0, window // 2)
    x0, x1 = max(0, center[0] - half), min(width, center[0] + half + 1)
    y0, y1 = max(0, center[1] - half), min(height, center[1] + half + 1)
    if x0 >= x1 or y0 >= y1:
        return None
    patch = depth[y0:y1, x0:x1].astype(np.float32) * scale
    valid = patch[np.isfinite(patch) & (patch > 0.0)]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def square_object_points(square_size_m: float) -> np.ndarray:
    """正方形板在自身坐标系下的四角，顺序与 order_clockwise 一致。

    左上 (-h, +h)、右上 (+h, +h)、右下 (+h, -h)、左下 (-h, -h)，
    即板面 X 朝右、Y 朝上、Z 垂直板面向外指向相机。
    """
    half = square_size_m / 2.0
    return np.array([[-half, half, 0.0],
                     [half, half, 0.0],
                     [half, -half, 0.0],
                     [-half, -half, 0.0]], dtype=np.float64)


def reprojection_error(object_points: np.ndarray, image_points: np.ndarray,
                       rvec: np.ndarray, tvec: np.ndarray,
                       camera_matrix: np.ndarray, dist_coeffs: np.ndarray) -> float:
    projected, _ = cv2.projectPoints(object_points, rvec, tvec,
                                      camera_matrix, dist_coeffs)
    delta = projected.reshape(-1, 2) - image_points.reshape(-1, 2)
    return float(np.mean(np.linalg.norm(delta, axis=1)))


def solve_square_pose(points: np.ndarray, camera_matrix: np.ndarray,
                      dist_coeffs: np.ndarray,
                      square_size_m: float) -> SquarePose | None:
    """用四个角点解平面 PnP；IPPE_SQUARE 会给出两个解，取重投影误差小的那个。"""
    if camera_matrix is None:
        return None
    object_points = square_object_points(square_size_m)
    image_points = np.asarray(points, dtype=np.float64).reshape(4, 1, 2)
    try:
        count, rvecs, tvecs, _ = cv2.solvePnPGeneric(
            object_points, image_points, camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
    except cv2.error:
        return None
    if count <= 0:
        return None

    best = None
    for rvec, tvec in zip(rvecs, tvecs):
        if tvec[2, 0] <= 0.0:      # 解到相机背后的直接丢掉
            continue
        # LM 细化能吃掉镜头畸变带来的偏差，误差按细化后重算
        rvec, tvec = cv2.solvePnPRefineLM(object_points, image_points,
                                          camera_matrix, dist_coeffs, rvec, tvec)
        error = reprojection_error(object_points, image_points, rvec, tvec,
                                   camera_matrix, dist_coeffs)
        if best is None or error < best.error:
            best = SquarePose(rvec, tvec, error, float(np.linalg.norm(tvec)))
    return best


def usable_depth(color: Frame, depth: Frame | None,
                 max_age: float) -> tuple[Frame | None, str | None]:
    """筛掉不可用于测距的深度帧，返回 (可用帧, 原因)。"""
    if depth is None:
        return None, NO_DEPTH
    if depth.image.shape[:2] != color.image.shape[:2]:
        return None, DEPTH_SHAPE
    if abs(color.stamp - depth.stamp) > max_age:
        return None, DEPTH_STALE
    return depth, None


def intrinsics_issue(camera: CameraModel | None,
                     image_size: tuple[int, int]) -> str | None:
    """内参缺失或分辨率对不上时返回原因，可用于 PnP 时返回 None。"""
    if camera is None:
        return NO_INTRINSICS
    if tuple(camera.size) != tuple(image_size):
        return INTRINSICS_SIZE
    return None


def largest_component(mask: np.ndarray) -> np.ndarray:
    """只保留面积最大的连通域。

    分面只针对当前主要目标：否则画面里另一块远处红色（误检或背景）会多出一个
    平面，凑巧垂直时还会被配成一对，给出一个不存在的箱体。
    """
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if count <= 1:
        return mask
    index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    # 直接比较成 0/255，省掉 np.where 的两个整幅临时数组（3.9ms -> 1.4ms）。
    # 不用「最大轮廓 drawContours 填充」：那样会把 mask 内部的空洞也填上，
    # 空洞处的深度可能是背景，会往点云里混进假点。
    return cv2.compare(labels, index, cv2.CMP_EQ)


def depth_band(cloud: np.ndarray, low: float = 0.4,
               high: float = 1.8) -> np.ndarray:
    """只保留主目标附近的点。

    偏粉的墙面、远处木柜也会落进红色阈值，最深能到 3m，平面拟合会把它们
    凑成两米宽的假面。以点云深度中位数为准，切掉明显更近或更远的部分。
    """
    if len(cloud) == 0:
        return np.zeros(0, dtype=bool)
    median = float(np.median(cloud[:, 2]))
    return (cloud[:, 2] > low * median) & (cloud[:, 2] < high * median)


def split_faces(mask: np.ndarray, depth: np.ndarray, scale: float,
                camera: CameraModel, config: DetectorConfig,
                ) -> tuple[tuple[planes.Face, ...], planes.Box | None, str]:
    """在红色区域内按深度分面。

    只有一个有效平面或找不到垂直配对时返回空 faces 与 single 模式，
    调用方据此回退到单面 PnP。
    """
    cloud, pixels = planes.cloud_from_depth(depth, largest_component(mask),
                                            camera, scale)
    keep = depth_band(cloud)
    cloud, pixels = cloud[keep], pixels[keep]
    if len(cloud) < config.plane_min_points:
        return (), None, 'single'

    extracted = planes.extract_planes(
        cloud, pixels, config.plane_distance_mm / 1000.0,
        config.plane_iterations, config.plane_min_points, config.plane_max,
        config.plane_points)
    if len(extracted) < 2:
        return (), None, 'single'

    rough = [planes.build_face(plane) for plane in extracted]
    pair = planes.pair_faces(rough, config.face_pair_angle_deg)
    if pair is None:
        return (), None, 'single'

    first, second, edge, edge_point = pair
    aligned = (planes.build_face(first.plane, edge, edge_point),
               planes.build_face(second.plane, edge, edge_point))
    override = np.array([config.box_size_x_mm, config.box_size_y_mm,
                         config.box_size_z_mm]) / 1000.0
    box = planes.box_from_faces(aligned[0], aligned[1], edge, override)

    # 第三面是增量：找不到或尺寸对不上就保持两面结果
    third = planes.attach_third(extracted, aligned[0], aligned[1], box,
                                config.face_pair_angle_deg,
                                config.face_size_tolerance_pct / 100.0,
                                config.plane_min_points,
                                config.face_max_incidence_deg)
    if third is None:
        return aligned, box, 'dual'
    box = planes.box_from_faces(aligned[0], aligned[1], edge, override, third)
    return aligned + (third,), box, 'triple'


def detect(color: Frame, depth: Frame | None, camera: CameraModel | None,
           config: DetectorConfig,
           max_depth_age: float = DEFAULT_DEPTH_MAX_AGE) -> Detection:
    """一帧的完整流程：分割 -> 拟合 -> 测距 -> PnP，并汇总降级原因。"""
    image = color.image
    mask = clean_mask(
        hsv_mask(image, config.h_low, config.h_high, config.s_low, config.s_high,
                 config.v_low, config.v_high),
        config.morph_kernel)
    quads = detect_quads(mask, config.min_area)

    issues: list[str] = []
    paired, depth_issue = usable_depth(color, depth, max_depth_age)
    if depth_issue is not None:
        issues.append(depth_issue)
    distances = [sample_distance(paired.image if paired else None, quad.center,
                                 config.depth_window,
                                 paired.scale if paired else 1.0)
                 for quad in quads]

    pose_issue = intrinsics_issue(camera, (image.shape[1], image.shape[0]))
    if pose_issue is not None:
        issues.append(pose_issue)
    poses: list[SquarePose | None] = []
    for quad in quads:
        pose = None
        if pose_issue is None:
            pose = solve_square_pose(quad.points, camera.matrix, camera.dist_coeffs,
                                     config.square_size_mm / 1000.0)
            if pose is None:
                issues.append(PNP_FAILED)
        poses.append(pose)

    faces: tuple[planes.Face, ...] = ()
    box = None
    mode = 'single'
    if config.face_split and paired is not None and pose_issue is None:
        faces, box, mode = split_faces(mask, paired.image, paired.scale,
                                       camera, config)

    return Detection(mask, quads, distances, poses,
                     tuple(dict.fromkeys(issues)), faces, box, mode)
