"""view 视觉的独立 MuJoCo 场景：0.35m 红箱 + 可参数化相机 + 真值。

不包含机械臂，也不依赖 ROS；渲染出的 BGR + 米制深度直接喂给 `view.vision.detect()`。
相机内参按相机插件同一规则由 fovy 反推：f = (height/2) / tan(fovy/2)。

用法（在 src/view 下）:
    python3 tools/sim_scene.py --case dual --seed 0
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import cv2  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

WIDTH = 640
HEIGHT = 480
FOVY_DEG = 60.0
FX = (HEIGHT / 2.0) / math.tan(math.radians(FOVY_DEG) / 2.0)
INTRINSICS = np.array([
    [FX, 0.0, WIDTH / 2.0],
    [0.0, FX, HEIGHT / 2.0],
    [0.0, 0.0, 1.0],
], dtype=np.float64)

BOX_SIZE = 0.35
BOX_CENTER = np.array([0.0, 0.0, BOX_SIZE / 2.0])
FLOOR_Z = -0.001  # 比箱底低 1mm，避免底面与地面共面 z-fighting
DEPTH_NOISE_A = 8.0e-4
DEPTH_NOISE_B = 2.0e-3

CASE_CHOICES = ["single", "dual", "triple", "occluded", "truncated"]
CASE_SALT = {"single": 11, "dual": 23, "triple": 37, "occluded": 53, "truncated": 67}


@dataclass
class SceneFace:
    center: np.ndarray
    normal: np.ndarray


@dataclass
class Scene:
    case: str
    mode: str                 # single | dual | triple（期望的可见面模式）
    seed: int
    box_center: np.ndarray
    box_size: float
    faces: list[SceneFace]
    expect_reject: bool
    xml: str
    camera_azimuth_deg: float
    camera_distance_m: float
    camera_height_m: float
    occluder: dict | None = None


def _normalize(vector: np.ndarray) -> np.ndarray:
    return vector / (np.linalg.norm(vector) + 1e-12)


def _camera_xyaxes(pos: np.ndarray, target: np.ndarray) -> np.ndarray:
    forward = _normalize(target - pos)
    right = _normalize(np.cross(forward, np.array([0.0, 0.0, 1.0])))
    up = np.cross(right, forward)
    return np.concatenate([right, up])


def _camera_position(azimuth_deg: float, distance: float, height: float) -> np.ndarray:
    a = math.radians(azimuth_deg)
    return np.array([
        distance * math.sin(a),
        -distance * math.cos(a),
        height,
    ], dtype=np.float64)


def _box_geom() -> str:
    s = BOX_SIZE / 2.0
    return (f'<geom name="red_box" type="box" group="0" '
            f'size="{s:.6f} {s:.6f} {s:.6f}" '
            f'pos="{BOX_CENTER[0]:.6f} {BOX_CENTER[1]:.6f} {BOX_CENTER[2]:.6f}" '
            f'rgba="1 0 0 1"/>')


def _scene_xml(camera_pos: np.ndarray, target: np.ndarray,
               occluder: dict | None) -> str:
    xy = _camera_xyaxes(camera_pos, target)
    extra = ""
    if occluder is not None:
        half = occluder["half"]
        center = occluder["center"]
        extra = (f'\n    <geom name="occluder" type="box" group="0" '
                 f'size="{half[0]:.6f} {half[1]:.6f} {half[2]:.6f}" '
                 f'pos="{center[0]:.6f} {center[1]:.6f} {center[2]:.6f}" '
                 f'rgba="0.5 0.5 0.5 1"/>')
        extra += (f'\n    <geom name="occluder2" type="box" group="0" '
                  f'size="{half[0]:.6f} {half[1]:.6f} {half[2] * 0.2:.6f}" '
                  f'pos="{center[0]:.6f} {center[1]:.6f} {center[2] + half[2] * 0.8:.6f}" '
                  f'rgba="0.5 0.5 0.5 1"/>')
    return f"""<mujoco model="view_vision_bench">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <visual>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.2 0.2 0.2" specular="0.1 0.1 0.1"/>
    <global offwidth="{WIDTH}" offheight="{HEIGHT}"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.6 0.65 0.7" rgb2="0.3 0.35 0.4" width="64" height="64"/>
  </asset>
  <worldbody>
    <geom name="floor" type="plane" size="3 3 0.1" pos="0 0 {FLOOR_Z}" rgba="0.55 0.55 0.55 1"/>
    <light name="main" pos="0 0 3" dir="0 0 -1" directional="true"/>
    <camera name="vision_camera" pos="{camera_pos[0]:.6f} {camera_pos[1]:.6f} {camera_pos[2]:.6f}"
            xyaxes="{xy[0]:.8f} {xy[1]:.8f} {xy[2]:.8f} {xy[3]:.8f} {xy[4]:.8f} {xy[5]:.8f}"
            fovy="{FOVY_DEG}" resolution="{WIDTH} {HEIGHT}"/>
    {_box_geom()}{extra}
  </worldbody>
</mujoco>
"""


def _visible_faces(mode: str) -> list[SceneFace]:
    """按约定模式给出真值可见面；相机方位总在 +y 侧、箱体 yaw=0。"""
    half = BOX_SIZE / 2.0
    front = SceneFace(BOX_CENTER + half * np.array([0.0, -1.0, 0.0]),
                      np.array([0.0, -1.0, 0.0]))
    side = SceneFace(BOX_CENTER + half * np.array([1.0, 0.0, 0.0]),
                     np.array([1.0, 0.0, 0.0]))
    top = SceneFace(BOX_CENTER + half * np.array([0.0, 0.0, 1.0]),
                    np.array([0.0, 0.0, 1.0]))
    if mode == "single":
        return [front]
    if mode == "dual":
        return [front, side]
    return [front, side, top]


def build_scene(case: str, seed: int) -> Scene:
    rng = np.random.default_rng([int(seed), CASE_SALT[case]])
    mode = {"single": "single", "dual": "dual", "triple": "triple"}.get(case, "dual")
    expect_reject = case in ("occluded", "truncated")

    if case == "single":
        azimuth = float(rng.uniform(0.0, 10.0))
        distance = float(rng.uniform(0.62, 0.80))
        height = BOX_CENTER[2]
    elif case == "dual":
        # seed 0 复刻现有 scene_vision.xml 的 0.79m / 主面约 28.9° 入射；
        # seed 5 取同距离的对称 45°，其余种子在 20–60° 扫。
        fixed = {0: 22.8, 5: 45.0}
        azimuth = fixed.get(seed, float(rng.uniform(20.0, 60.0)))
        distance = 0.79 if seed in fixed else float(rng.uniform(0.70, 0.90))
        height = BOX_CENTER[2]
    elif case == "triple":
        # 三个面同时进 65° 入射需要远相机：箱心约 1.6–1.8m，入射 53–58°。
        azimuth = 45.0 + float(rng.uniform(-4.0, 4.0))
        distance = float(rng.uniform(0.90, 1.10)) * math.sqrt(2.0)
        height = float(rng.uniform(1.20, 1.40))
    elif case == "occluded":
        azimuth = 35.0 + float(rng.uniform(-8.0, 8.0))
        distance = 0.79
        height = BOX_CENTER[2]
    elif case == "truncated":
        azimuth = 35.0 + float(rng.uniform(-8.0, 8.0))
        distance = 0.79
        height = BOX_CENTER[2]
    else:
        raise ValueError(f"未知场景: {case}")

    camera_pos = _camera_position(azimuth, distance, height)
    target = BOX_CENTER.copy()

    occluder = None
    if case == "truncated":
        forward = _normalize(BOX_CENTER - camera_pos)
        right = _normalize(np.cross(forward, np.array([0.0, 0.0, 1.0])))
        target = BOX_CENTER + 0.36 * right
    elif case == "occluded":
        forward = _normalize(BOX_CENTER - camera_pos)
        right = _normalize(np.cross(forward, np.array([0.0, 0.0, 1.0])))
        center = camera_pos + 0.55 * float(np.linalg.norm(BOX_CENTER - camera_pos)) * forward
        center = center + 0.09 * right
        center[2] = 0.25
        occluder = {"center": center, "half": np.array([0.05, 0.03, 0.25])}

    xml = _scene_xml(camera_pos, target, occluder)
    return Scene(case=case, mode=mode, seed=seed, box_center=BOX_CENTER.copy(),
                 box_size=BOX_SIZE, faces=_visible_faces(mode),
                 expect_reject=expect_reject, xml=xml,
                 camera_azimuth_deg=azimuth, camera_distance_m=distance,
                 camera_height_m=height, occluder=occluder)


class SimRenderer:
    """编译 MJCF，渲染 BGR + 米制深度，并提供世界->光学相机的真值换算。"""

    def __init__(self, scene: Scene):
        self.scene = scene
        self.model = mujoco.MjModel.from_xml_string(scene.xml)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "vision_camera")
        self.cam_pos = np.array(self.data.cam_xpos[cam_id], dtype=np.float64).copy()
        self.R_cw = np.array(self.data.cam_xmat[cam_id], dtype=np.float64).reshape(3, 3).copy()
        self.K = INTRINSICS.copy()
        self.renderer = mujoco.Renderer(self.model, HEIGHT, WIDTH)

    def render(self) -> tuple[np.ndarray, np.ndarray]:
        self.renderer.disable_depth_rendering()
        self.renderer.update_scene(self.data, camera="vision_camera")
        rgb = np.asarray(self.renderer.render())

        self.renderer.enable_depth_rendering()
        self.renderer.update_scene(self.data, camera="vision_camera")
        depth = np.asarray(self.renderer.render(), dtype=np.float32)

        bgr = rgb[..., ::-1].copy()
        depth = np.where((depth > 0.05) & (depth < 10.0), depth, 0.0).astype(np.float32)
        return bgr, depth

    def to_camera(self, point_world) -> np.ndarray:
        p = self.R_cw.T @ (np.asarray(point_world, dtype=np.float64) - self.cam_pos)
        return p * np.array([1.0, -1.0, -1.0])

    def normal_to_camera(self, normal_world) -> np.ndarray:
        n = self.R_cw.T @ np.asarray(normal_world, dtype=np.float64)
        return n * np.array([1.0, -1.0, -1.0])

    def close(self) -> None:
        try:
            self.renderer.close()
        except Exception:  # pragma: no cover
            pass


def add_sensor_noise(bgr: np.ndarray, depth: np.ndarray, rng: np.random.Generator,
                     level: str = "typical") -> tuple[np.ndarray, np.ndarray]:
    """RealSense 级噪声：彩色噪声/轻模糊 + 深度高斯 + 边缘飞点。"""
    if level == "clean":
        return bgr.copy(), depth.copy()

    color = cv2.GaussianBlur(bgr.astype(np.float32), (3, 3), 0.6)
    color += rng.normal(0.0, 2.0, color.shape)
    color = np.clip(color, 0, 255).astype(np.uint8)

    d = depth.astype(np.float32).copy()
    valid = d > 0
    gx = np.abs(np.diff(d, axis=1, prepend=d[:, :1]))
    gy = np.abs(np.diff(d, axis=0, prepend=d[:1, :]))
    edge = np.maximum(gx, gy) > 0.01
    edge = cv2.dilate(edge.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
    blurred = cv2.GaussianBlur(d, (5, 5), 1.2)
    d = np.where(edge & valid, blurred, d)

    sigma = DEPTH_NOISE_A * d * d + DEPTH_NOISE_B * d
    d = np.where(valid, d + rng.normal(0.0, 1.0, d.shape) * sigma, 0.0)
    d = np.round(np.clip(d, 0.0, None), 3)
    d[~valid] = 0.0
    return color, d.astype(np.float32)


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description="view 仿真场景自检")
    parser.add_argument("--case", default="dual", choices=CASE_CHOICES)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", default=str(Path(__file__).resolve().parent.parent / "out" / "scene"))
    args = parser.parse_args()

    scene = build_scene(args.case, args.seed)
    renderer = SimRenderer(scene)
    bgr, depth = renderer.render()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out / "rgb.png"), bgr)
    cv2.imwrite(str(out / "depth.png"), (depth * 1000.0).astype(np.uint16))
    print(f"场景 {args.case} seed={args.seed} mode={scene.mode} "
          f"az={scene.camera_azimuth_deg:.1f}° d={scene.camera_distance_m:.2f}m "
          f"h={scene.camera_height_m:.2f}m -> {out}")
    renderer.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
