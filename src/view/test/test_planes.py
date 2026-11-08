import numpy as np

from view import planes


class _Camera:
    """只提供 cloud_from_depth 需要的矩阵与尺寸。"""

    def __init__(self, fx=500.0, fy=500.0, cx=50.0, cy=50.0):
        self.matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.zeros(5)
        self.size = (100, 100)


def _two_face_cloud(size=(0.35, 0.35, 0.35), center=(0.4, 0.0, 0.7),
                    grid=60, noise=0.001, seed=1):
    """构造正对镜头加上左侧可见的箱体点云。

    箱子偏在 x=+0.4，相机在 x=0，于是能看到前面 z=c2-hz 与左侧面 x=c0-hx，
    两个面的外法线（-z 与 -x）都朝相机，符合本模块的法线约定。
    """
    rng = np.random.default_rng(seed)
    center = np.asarray(center, dtype=np.float64)
    half = np.asarray(size, dtype=np.float64) / 2.0

    u = np.linspace(-half[0], half[0], grid)
    v = np.linspace(-half[1], half[1], grid)
    gx, gy = np.meshgrid(u, v)
    front = np.stack([center[0] + gx.ravel(),
                      center[1] + gy.ravel(),
                      np.full(gx.size, center[2] - half[2])], axis=1)

    wy = np.linspace(-half[1], half[1], grid)
    wz = np.linspace(center[2] - half[2], center[2] + half[2], grid)
    gy2, gz2 = np.meshgrid(wy, wz)
    left = np.stack([np.full(gy2.size, center[0] - half[0]),
                     center[1] + gy2.ravel(),
                     gz2.ravel()], axis=1)

    points = np.vstack([front, left])
    points = points + rng.normal(scale=noise, size=points.shape)
    pixels = np.zeros((len(points), 2), dtype=np.int32)
    return points, pixels


def _three_face_cloud(size=(0.35, 0.35, 0.35), center=(0.4, 0.425, 0.7),
                      noise=0.001, seed=2):
    """前面 + 左侧面 + 顶面，模拟相机在箱子左上前方看到三个面。

    三个面刻意给不同的采样密度（70/55/45），让 pair_faces 按内点数稳定地
    选中前面与左侧面，顶面固定作为第三面，测试才有确定的分工。
    顶面取 y = cy - h（相机 y 朝下，所以这是朝上的一面），法线朝相机。
    """
    rng = np.random.default_rng(seed)
    center = np.asarray(center, dtype=np.float64)
    half = np.asarray(size, dtype=np.float64) / 2.0

    def grid_face(count, map_uv):
        u, v = np.meshgrid(np.linspace(-1.0, 1.0, count),
                           np.linspace(-1.0, 1.0, count))
        return map_uv(u.ravel(), v.ravel())

    faces = [
        grid_face(70, lambda u, v: np.stack([center[0] + u * half[0],
                                             center[1] + v * half[1],
                                             np.full(u.size, center[2] - half[2])], 1)),
        grid_face(55, lambda u, v: np.stack([np.full(u.size, center[0] - half[0]),
                                             center[1] + u * half[1],
                                             center[2] + v * half[2]], 1)),
        grid_face(45, lambda u, v: np.stack([center[0] + u * half[0],
                                             np.full(u.size, center[1] - half[1]),
                                             center[2] + v * half[2]], 1)),
    ]
    points = np.vstack(faces)
    points = points + rng.normal(scale=noise, size=points.shape)
    pixels = np.zeros((len(points), 2), dtype=np.int32)
    return points, pixels


def test_cloud_from_depth_deprojects_pixel_rays():
    depth = np.full((100, 100), 1000, dtype=np.uint16)
    mask = np.full((100, 100), 255, dtype=np.uint8)
    points, pixels = planes.cloud_from_depth(depth, mask, _Camera(), 0.001)
    assert len(points) == len(pixels) == 100 * 100
    assert np.allclose(points[:, 2], 1.0, atol=1e-9)
    # 像素 50,50 是主点，射线沿光轴
    index = int(np.flatnonzero((pixels[:, 0] == 50) & (pixels[:, 1] == 50))[0])
    assert np.allclose(points[index], [0.0, 0.0, 1.0], atol=1e-9)


def test_cloud_from_depth_skips_invalid_pixels():
    depth = np.full((10, 10), 1000, dtype=np.uint16)
    depth[0, :] = 0
    depth[5, 5] = 0
    mask = np.zeros((10, 10), dtype=np.uint8)
    mask[1:3, 1:3] = 255
    points, _ = planes.cloud_from_depth(depth, mask, _Camera(), 0.001)
    assert len(points) == 4


def test_extract_planes_finds_both_faces():
    points, pixels = _two_face_cloud()
    found = planes.extract_planes(points, pixels, 0.008, 200, 500, 3, 4000)
    assert len(found) == 2
    assert abs(float(found[0].normal @ found[1].normal)) < 0.05
    # 法线朝相机
    for plane in found:
        assert float(plane.normal @ plane.points.mean(axis=0)) < 0.0


def test_extract_planes_returns_single_for_flat_surface():
    rng = np.random.default_rng(3)
    grid = np.linspace(-0.2, 0.2, 50)
    gx, gy = np.meshgrid(grid, grid)
    points = np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, 0.6)], axis=1)
    points = points + rng.normal(scale=0.001, size=points.shape)
    pixels = np.zeros((len(points), 2), dtype=np.int32)
    found = planes.extract_planes(points, pixels, 0.008, 200, 500, 3, 4000)
    assert len(found) == 1


def test_pair_and_box_recover_pose():
    points, pixels = _two_face_cloud()
    extracted = planes.extract_planes(points, pixels, 0.008, 200, 500, 3, 4000)
    rough = [planes.build_face(plane) for plane in extracted]
    pair = planes.pair_faces(rough, 12.0)
    assert pair is not None

    first, second, edge, edge_point = pair
    face_a = planes.build_face(first.plane, edge, edge_point)
    face_b = planes.build_face(second.plane, edge, edge_point)
    box = planes.box_from_faces(face_a, face_b, edge)

    assert np.allclose(box.center, [0.4, 0.0, 0.7], atol=0.005)
    assert np.allclose(box.size, [0.35, 0.35, 0.35], atol=0.01)
    # 右手系
    assert np.allclose(np.cross(box.x_axis, box.y_axis), box.z_axis, atol=1e-6)


def test_third_face_improves_size_and_keeps_center():
    points, pixels = _three_face_cloud()
    extracted = planes.extract_planes(points, pixels, 0.008, 200, 500, 4, 4000)
    assert len(extracted) == 3

    rough = [planes.build_face(plane) for plane in extracted]
    first, second, edge, edge_point = planes.pair_faces(rough, 12.0)
    face_a = planes.build_face(first.plane, edge, edge_point)
    face_b = planes.build_face(second.plane, edge, edge_point)
    box_two = planes.box_from_faces(face_a, face_b, edge)

    third = planes.attach_third(extracted, face_a, face_b, box_two,
                                size_tolerance=0.15, min_points=500,
                                max_incidence_deg=89.0)
    assert third is not None
    box_three = planes.box_from_faces(face_a, face_b, edge, None, third)

    assert np.allclose(box_three.size, 0.35, atol=0.01)
    assert np.allclose(box_three.center, [0.4, 0.425, 0.7], atol=0.005)
    # 中心仍由两个主面给出；尺寸被第三面修正后中心只跟着动一点点
    assert np.allclose(box_three.center, box_two.center, atol=0.002)


def test_third_face_rejected_at_grazing_incidence():
    points, pixels = _three_face_cloud()
    extracted = planes.extract_planes(points, pixels, 0.008, 200, 500, 4, 4000)
    rough = [planes.build_face(plane) for plane in extracted]
    first, second, edge, edge_point = planes.pair_faces(rough, 12.0)
    face_a = planes.build_face(first.plane, edge, edge_point)
    face_b = planes.build_face(second.plane, edge, edge_point)
    box = planes.box_from_faces(face_a, face_b, edge)

    # 顶面在这个布局里入射角约 73°，65° 门槛应当拒掉它
    assert planes.attach_third(extracted, face_a, face_b, box,
                               size_tolerance=0.15, min_points=500,
                               max_incidence_deg=65.0) is None


def test_face_axes_follow_edge_and_point_inward():
    points, pixels = _two_face_cloud()
    extracted = planes.extract_planes(points, pixels, 0.008, 200, 500, 3, 4000)
    rough = [planes.build_face(plane) for plane in extracted]
    first, second, edge, edge_point = planes.pair_faces(rough, 12.0)
    face = planes.build_face(first.plane, edge, edge_point)

    assert abs(float(face.x_axis @ edge)) > 0.99
    assert abs(float(face.x_axis @ face.z_axis)) < 1e-6
    assert abs(float(face.y_axis @ face.z_axis)) < 1e-6
    assert np.isclose(np.linalg.norm(face.y_axis), 1.0)
    # Y 指向面内部：中心到棱的向量与 Y 同向
    inward = face.center - edge_point
    inward = inward - face.z_axis * float(face.z_axis @ inward)
    assert float(face.y_axis @ inward) > 0.0
