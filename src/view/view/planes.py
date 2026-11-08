"""红色目标的深度分面与相邻面配准。

输入是对齐深度、红色 mask 与彩色内参，输出若干平面、每个可见面的位姿，
以及由两个相邻面合成的箱体位姿。只依赖 numpy/OpenCV，可被 pytest 直接驱动。

约定：
- 法线一律朝相机一侧（原点在光心，所以 normal·center < 0）。
- 面的 Z 轴取法线，X 轴取两面共用棱方向，Y = Z×X 指向面内部。
- 箱体的 X 轴同样沿共用棱，Y、Z 沿两个面的法线。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# 超过这个距离的深度点不参与分面（远处背景没有分面价值，还会拖慢 RANSAC）
MAX_RANGE_M = 3.0

# 生成平面法线时用的最小向量长度，低于它认为三点共线
MIN_CROSS = 1e-9

# 拟合平面时参与 SVD 的点数上限：更多点只增加耗时，不提升精度
FIT_POINTS_LIMIT = 20000

# 单帧点云的点数上限。下游的分面、配对全部与点数线性相关，红块占满画面时
# 二十多万点只是把单帧拖到几十毫秒，精度上无损（_fit_plane 本身也只取两万点）
CLOUD_MAX_POINTS = 20000


@dataclass(frozen=True)
class Plane:
    """一个拟合出的平面及其内点。"""

    normal: np.ndarray      # (3,) 单位法线，指向相机
    offset: float           # normal·p + offset = 0
    points: np.ndarray      # (N, 3) 内点三维坐标，单位米
    pixels: np.ndarray      # (N, 2) 内点像素坐标 u,v


@dataclass(frozen=True)
class Face:
    """一个可见面的位姿与外形。"""

    center: np.ndarray      # (3,) 面中心，相机系
    x_axis: np.ndarray      # (3,) 沿共用棱
    y_axis: np.ndarray      # (3,) 指向面内部
    z_axis: np.ndarray      # (3,) 法线，朝相机
    width: float            # 沿 x 的边长
    height: float           # 沿 y 的边长
    plane: Plane


@dataclass(frozen=True)
class Box:
    """由相邻两面合成的箱体。"""

    center: np.ndarray      # (3,) 箱体中心，相机系
    x_axis: np.ndarray      # 沿共用棱
    y_axis: np.ndarray      # 沿面 1 法线
    z_axis: np.ndarray      # 沿面 2 法线
    size: np.ndarray        # (3,) 三条边长，顺序同 x/y/z 轴


def cloud_from_depth(depth: np.ndarray, mask: np.ndarray, camera,
                     scale: float = 0.001,
                     max_range: float = MAX_RANGE_M,
                     max_points: int = CLOUD_MAX_POINTS):
    """把 mask 内且深度有效的像素反投影成点云，返回 (points(N,3), pixels(N,2))。

    深度已对齐到彩色，所以直接用彩色内参；RealSense 的彩色畸变系数为 0，
    这里不做去畸变，若换成有畸变的相机需要先 undistort。

    先按 mask 取像素再取深度：整幅 float64 转换加 isfinite 在 848x480 上要
    十几毫秒，而红块通常只占画面一部分。超过 max_points 时等间隔抽稀，让单帧
    成本与目标在画面里的大小解耦。
    """
    matrix = camera.matrix
    fx, fy = float(matrix[0, 0]), float(matrix[1, 1])
    cx, cy = float(matrix[0, 2]), float(matrix[1, 2])
    v, u = np.nonzero(mask > 0)
    if v.size == 0:
        return np.empty((0, 3)), np.empty((0, 2), dtype=np.int32)
    if v.size > max_points:
        stride = int(np.ceil(v.size / float(max_points)))
        v, u = v[::stride], u[::stride]
    zz = depth[v, u].astype(np.float64) * float(scale)
    valid = (zz > 0.0) & (zz < max_range)
    # 16UC1 不会出现 NaN/Inf，只有 32FC1 才需要这次全量扫描
    if np.issubdtype(depth.dtype, np.floating):
        valid &= np.isfinite(zz)
    u, v, zz = u[valid], v[valid], zz[valid]
    points = np.stack([(u - cx) * zz / fx, (v - cy) * zz / fy, zz], axis=1)
    pixels = np.stack([u, v], axis=1).astype(np.int32)
    return points, pixels


def _fit_plane(points: np.ndarray) -> tuple[np.ndarray, float]:
    """最小二乘拟合平面，返回朝相机的单位法线与 offset。"""
    # 十几万点做 SVD 没有额外精度收益，等间隔抽到上限再算
    if len(points) > FIT_POINTS_LIMIT:
        stride = int(np.ceil(len(points) / float(FIT_POINTS_LIMIT)))
        points = points[::stride]
    centroid = points.mean(axis=0)
    # 最小奇异向量就是法线；点数不足时 SVD 仍能给出一个退化解，交由调用方过滤
    _, _, vt = np.linalg.svd(points - centroid, full_matrices=False)
    normal = vt[2]
    # 原点在光心，朝相机的法线满足 normal·centroid < 0
    if float(normal @ centroid) > 0.0:
        normal = -normal
    return normal, -float(normal @ centroid)


def _ransac_plane(points: np.ndarray, distance: float, iterations: int,
                  rng: np.random.Generator) -> np.ndarray | None:
    """在点集里找最大内点平面，返回布尔内点掩码。

    一次性生成全部假设再矩阵化评估：逐次循环里每次都做一遍 Python 层的
    距离计算会把单帧拖到几十毫秒，批量后只剩一次 BLAS 矩阵乘。
    """
    count = len(points)
    if count < 3:
        return None

    triples = rng.integers(0, count, size=(iterations, 3))
    distinct = ((triples[:, 0] != triples[:, 1])
                & (triples[:, 1] != triples[:, 2])
                & (triples[:, 0] != triples[:, 2]))
    triples = triples[distinct]
    if len(triples) == 0:
        return None

    p0 = points[triples[:, 0]]
    p1 = points[triples[:, 1]]
    p2 = points[triples[:, 2]]
    normals = np.cross(p1 - p0, p2 - p0)
    lengths = np.linalg.norm(normals, axis=1)
    usable = lengths > MIN_CROSS
    if not usable.any():
        return None
    normals = normals[usable] / lengths[usable, None]
    offsets = -np.einsum('ij,ij->i', normals, p0[usable])

    residuals = np.abs(points @ normals.T + offsets)
    hits = (residuals < distance).sum(axis=0)
    best = int(np.argmax(hits))
    if int(hits[best]) < 3:
        return None
    return residuals[:, best] < distance


def _stride(count: int, max_points: int) -> int:
    """等间隔取样步长；同一帧内固定，保证结果可复现。"""
    if max_points <= 0 or count <= max_points:
        return 1
    return int(np.ceil(count / float(max_points)))


def extract_planes(points: np.ndarray, pixels: np.ndarray, distance: float,
                   iterations: int, min_points: int, max_planes: int = 3,
                   max_fit_points: int = 4000,
                   seed: int = 0) -> list[Plane]:
    """顺序提取最多 max_planes 个平面。

    RANSAC 只跑在降采样的点上，内点判定与最终拟合用全分辨率点，
    这样法线和面外形都不会被降采样带到失真。
    """
    if len(points) < min_points:
        return []

    used = np.zeros(len(points), dtype=bool)
    rng = np.random.default_rng(seed)
    found: list[Plane] = []
    for _ in range(max_planes):
        rest = np.flatnonzero(~used)
        if len(rest) < min_points:
            break
        sample = rest[::_stride(len(rest), max_fit_points)]
        # min_points 是全分辨率口径，样本只是它的一个子集，按比例折算后再比，
        # 否则 plane_min_points 调大一点就会在第一帧就判定失败
        sample_min = max(3, int(min_points * len(sample) / float(len(rest))))
        inliers = _ransac_plane(points[sample], distance, iterations, rng)
        if inliers is None or int(inliers.sum()) < sample_min:
            break

        normal, offset = _fit_plane(points[sample[inliers]])
        # 全分辨率复核内点时直接在整幅点云上算距离，避免把候选点再拷一份
        full = (np.abs(points @ normal + offset) < distance) & ~used
        if int(full.sum()) < min_points:
            break
        index = np.flatnonzero(full)
        # 只把上限数量的内点拷出来做重拟合，十几万点的拷贝纯属浪费
        normal, offset = _fit_plane(
            points[index[::_stride(len(index), FIT_POINTS_LIMIT)]])
        full = (np.abs(points @ normal + offset) < distance) & ~used
        if int(full.sum()) < min_points:
            break

        used |= full
        found.append(Plane(normal, offset, points[full], pixels[full]))
    return found


def _any_in_plane(normal: np.ndarray) -> np.ndarray:
    """在平面内取一个与 normal 垂直的单位向量。"""
    helper = np.array([1.0, 0.0, 0.0])
    if abs(float(normal[0])) > 0.9:
        helper = np.array([0.0, 1.0, 0.0])
    axis = np.cross(normal, helper)
    return axis / float(np.linalg.norm(axis))


def build_face(plane: Plane, edge_dir: np.ndarray | None = None,
               edge_point: np.ndarray | None = None) -> Face:
    """由平面内点求面的中心、两条面内轴与边长。

    edge_dir/edge_point 给出共用棱时，X 沿棱、Y 指向面内部；否则用任意面内轴，
    这种面没有确定朝向，只适合展示。
    """
    z = plane.normal
    origin = -plane.offset * z
    if edge_dir is None:
        x = _any_in_plane(z)
    else:
        x = np.asarray(edge_dir, dtype=np.float64)
        x = x - z * float(z @ x)
        norm = float(np.linalg.norm(x))
        x = _any_in_plane(z) if norm < MIN_CROSS else x / norm
    y = np.cross(z, x)

    along_x = (plane.points - origin) @ x
    along_y = (plane.points - origin) @ y
    center = (origin + 0.5 * float(along_x.min() + along_x.max()) * x
              + 0.5 * float(along_y.min() + along_y.max()) * y)

    if edge_point is not None:
        inward = center - np.asarray(edge_point, dtype=np.float64)
        inward = inward - z * float(z @ inward)
        if float(y @ inward) < 0.0:
            # 同时翻 X、Y 保持右手系，边长不受影响
            x, y = -x, -y

    return Face(center, x, y, z,
                float(along_x.max() - along_x.min()),
                float(along_y.max() - along_y.min()), plane)


def _edge_point(plane_a: Plane, plane_b: Plane,
                edge_dir: np.ndarray) -> np.ndarray:
    """两面交线上离面 A 中心最近的点。"""
    matrix = np.vstack([plane_a.normal, plane_b.normal, edge_dir])
    rhs = np.array([-plane_a.offset, -plane_b.offset, 0.0])
    point = np.linalg.lstsq(matrix, rhs, rcond=None)[0]
    return point


def pair_faces(faces: list[Face], angle_tol_deg: float = 12.0):
    """找出最像同一箱子相邻两面的组合。

    判据：法线夹角接近 90°，且两面中心距离不超过较大面边长的两倍。
    返回 (face_a, face_b, edge_dir, edge_point) 或 None。
    """
    best = None
    for i in range(len(faces)):
        for j in range(i + 1, len(faces)):
            first, second = faces[i], faces[j]
            cosine = abs(float(first.z_axis @ second.z_axis))
            angle = float(np.degrees(np.arccos(min(1.0, cosine))))
            if abs(angle - 90.0) > angle_tol_deg:
                continue
            edge = np.cross(first.z_axis, second.z_axis)
            norm = float(np.linalg.norm(edge))
            if norm < 1e-6:      # 两面平行，没有唯一交线
                continue
            edge = edge / norm
            gap = float(np.linalg.norm(first.center - second.center))
            scale = max(first.width, first.height, second.width, second.height)
            if scale <= 0.0 or gap > 2.0 * scale:
                continue
            # 先比内点数再比夹角/距离：反光造成的稀疏平面会被少数点撑出很大的
            # 假边长，光看几何贴近度容易被它抢走配对。
            count = len(first.plane.points) + len(second.plane.points)
            penalty = abs(angle - 90.0) + gap / scale
            if best is None or (count, -penalty) > best[0]:
                best = ((count, -penalty), first, second, edge,
                        _edge_point(first.plane, second.plane, edge))
    return None if best is None else best[1:]


def build_face_on_axes(plane: Plane, axis_u: np.ndarray,
                       axis_v: np.ndarray) -> Face:
    """按给定的两条面内轴构建面，用于第三面（它的轴由箱体决定）。"""
    z = plane.normal
    u = np.asarray(axis_u, dtype=np.float64)
    u = u - z * float(z @ u)
    norm = float(np.linalg.norm(u))
    if norm < MIN_CROSS:
        u = _any_in_plane(z)
    else:
        u = u / norm
    v = np.cross(z, u)
    origin = -plane.offset * z
    along_u = (plane.points - origin) @ u
    along_v = (plane.points - origin) @ v
    center = (origin + 0.5 * float(along_u.min() + along_u.max()) * u
              + 0.5 * float(along_v.min() + along_v.max()) * v)
    return Face(center, u, v, z,
                float(along_u.max() - along_u.min()),
                float(along_v.max() - along_v.min()), plane)


def incidence_degrees(center: np.ndarray, normal: np.ndarray) -> float:
    """法线与视线的夹角：0 表示正对相机，90 表示完全侧对。"""
    ray = -np.asarray(center, dtype=np.float64)
    norm = float(np.linalg.norm(ray))
    if norm < 1e-9:
        return 90.0
    cosine = abs(float(np.asarray(normal) @ (ray / norm)))
    return float(np.degrees(np.arccos(min(1.0, cosine))))


def attach_third(candidates: list[Plane], face_a: Face, face_b: Face,
                 box: Box, angle_tol_deg: float = 12.0,
                 size_tolerance: float = 0.15,
                 min_points: int = 600,
                 max_incidence_deg: float = 65.0) -> Face | None:
    """在已配好的两面之外找第三个面。

    两主面已经占掉了条件最好的两个面，剩下的第三面通常最掠射，而掠射面的
    边长会被深度边缘啃掉几十毫米，经 d/2 直接带偏中心。因此除了「法线与
    箱体 X 轴近似平行、内点数足够、边长与已知尺寸相差不超过 size_tolerance」
    之外，还要求入射角不超过 max_incidence_deg，否则宁可退回两面。
    """
    tolerance = float(np.cos(np.radians(angle_tol_deg)))
    best = None
    for plane in candidates:
        if plane is face_a.plane or plane is face_b.plane:
            continue
        if len(plane.points) < min_points:
            continue
        if abs(float(plane.normal @ box.x_axis)) < tolerance:
            continue
        face = build_face_on_axes(plane, box.y_axis, box.z_axis)
        if incidence_degrees(face.center, face.z_axis) > max_incidence_deg:
            continue
        if abs(face.width - box.size[1]) > size_tolerance * box.size[1]:
            continue
        if abs(face.height - box.size[2]) > size_tolerance * box.size[2]:
            continue
        score = 1.0 - abs(float(plane.normal @ box.x_axis))
        if best is None or score < best[0]:
            best = (score, face)
    return None if best is None else best[1]


def box_from_faces(face_a: Face, face_b: Face, edge_dir: np.ndarray,
                   size_override: np.ndarray | None = None,
                   third: Face | None = None) -> Box:
    """由相邻两面（可选第三面）合成箱体中心与尺寸。

    面 A 的法线 n1、面 B 的法线 n2：面 A 跨「棱 × n2」，面 B 跨「棱 × n1」，
    所以 B 向尺寸取自面 B 的高、C 向尺寸取自面 A 的高；给了第三面（法线沿 X）
    时，B、C 各有两次测量取均值。中心始终由两个主面各推一次再平均，
    第三面只贡献尺寸冗余。
    """
    x = np.asarray(edge_dir, dtype=np.float64)
    y = face_a.z_axis
    if float(np.cross(x, y) @ face_b.z_axis) < 0.0:
        x = -x                      # 只翻棱向，保证 X×Y 与面 B 法线同向
    z = np.cross(x, y)

    if third is None:
        size = np.array([0.5 * (face_a.width + face_b.width),
                         face_b.height,
                         face_a.height])
    else:
        size = np.array([0.5 * (face_a.width + face_b.width),
                         0.5 * (face_b.height + third.width),
                         0.5 * (face_a.height + third.height)])
    if size_override is not None:
        override = np.asarray(size_override, dtype=np.float64)
        given = override > 0.0
        size[given] = override[given]

    # 中心只用两个主面：把第三面也塞进最小二乘会更差——它的法线差 1°，在 0.7m
    # 距离上就对应约 12mm 的平面偏移，反而把中心带偏；第三面的价值在尺寸冗余。
    from_a = face_a.center - 0.5 * size[1] * face_a.z_axis
    from_b = face_b.center - 0.5 * size[2] * face_b.z_axis
    center = 0.5 * (from_a + from_b)
    return Box(center, x, y, z, size)


def face_corners(face: Face) -> np.ndarray:
    """面的四个角，顺序为 -x-y、+x-y、+x+y、-x+y。"""
    hx, hy = face.width / 2.0, face.height / 2.0
    return np.array([face.center - hx * face.x_axis - hy * face.y_axis,
                     face.center + hx * face.x_axis - hy * face.y_axis,
                     face.center + hx * face.x_axis + hy * face.y_axis,
                     face.center - hx * face.x_axis + hy * face.y_axis])
