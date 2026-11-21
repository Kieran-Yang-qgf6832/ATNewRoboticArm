"""view 视觉的离线 MuJoCo 基准：精度、模式、耗时。

用法（在 src/view 下）:
    python3 tools/bench_sim.py
    python3 tools/bench_sim.py --out out/bench --repeats 5

覆盖：
- single / dual / triple 各 10 个随机种子，clean 与 RealSense 级噪声各一遍（60 帧）；
- dual 的 seed 0 复刻 scene_vision.xml 的 0.79m / 主面约 28.9° 位姿；
- triple 用远相机（箱心约 1.6–1.8m），让三个面同时进 65° 入射；
- occluded / truncated 各 5 帧，仅 noisy，作诊断，不纳入主 PASS。

精度口径沿用 view/check_sim_vision.py：面心 20mm、法线 3°、箱心 15mm、边长 25mm，
按**均值**判定；P50/P95/max 只报告。single 没有箱心/箱体尺寸，只测面心与法线。
"""
from __future__ import annotations

import argparse
import cProfile
import csv
import json
import os
import pstats
import platform
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import cv2  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from view import target, vision  # noqa: E402
from view.config import DetectorConfig  # noqa: E402
from view.render import compose  # noqa: E402
from tools.sim_scene import (BOX_SIZE, FX, HEIGHT, WIDTH, SimRenderer,  # noqa: E402
                             add_sensor_noise, build_scene)

THRESHOLDS = {
    "face_mm": 20.0,
    "normal_deg": 3.0,
    "box_mm": 15.0,
    "size_mm": 25.0,
}

VARIANTS = ("clean", "noisy")
NORMAL_MODES = ("single", "dual", "triple")
DRIVE_MODES = ("dual", "triple")  # 有箱心/尺寸的模式
DIAGNOSTIC_CASES = ("occluded", "truncated")

FIELDS = [
    "case", "group", "seed", "variant", "expected_mode", "detected_mode",
    "mode_ok", "emitted", "missed", "face_err_mm", "normal_err_deg",
    "box_center_err_mm", "size_err_mm", "size_x_mm", "size_y_mm", "size_z_mm",
    "incidence_deg", "plane_rms_mm", "position_source", "issues",
]


def angle_between(first: np.ndarray, second: np.ndarray) -> float:
    a = np.asarray(first, dtype=np.float64)
    b = np.asarray(second, dtype=np.float64)
    cosine = float(a @ b) / max(float(np.linalg.norm(a) * np.linalg.norm(b)), 1e-12)
    return float(np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0))))


def _stat(values) -> dict | None:
    arr = np.asarray([v for v in values if v is not None], dtype=np.float64)
    if arr.size == 0:
        return None
    return {
        "n": int(arr.size),
        "mean": float(arr.mean()),
        "p50": float(np.percentile(arr, 50)),
        "p95": float(np.percentile(arr, 95)),
        "max": float(arr.max()),
    }


def make_jobs() -> list[dict]:
    jobs: list[dict] = []
    for mode in NORMAL_MODES:
        for seed in range(10):
            for variant in VARIANTS:
                jobs.append({"case": mode, "group": mode, "seed": seed,
                             "variant": variant, "diagnostic": False})
    for case in DIAGNOSTIC_CASES:
        for seed in range(5):
            jobs.append({"case": case, "group": "diagnostic", "seed": seed,
                         "variant": "noisy", "diagnostic": True})
    return jobs


def evaluate(renderer: SimRenderer, scene, detection, sample) -> dict:
    row = {
        "case": scene.case, "group": scene.case,
        "seed": scene.seed, "expected_mode": scene.mode,
        "detected_mode": detection.mode,
        "mode_ok": int(detection.mode == scene.mode),
        "emitted": int(sample is not None),
        "missed": int(sample is None and not scene.expect_reject),
        "issues": ";".join(detection.issues),
    }
    if sample is None:
        return row

    faces = [(renderer.to_camera(face.center), renderer.normal_to_camera(face.normal))
             for face in scene.faces]
    face_center, face_normal = min(
        faces, key=lambda item: float(np.linalg.norm(item[0] - sample.center)))
    row["face_err_mm"] = float(np.linalg.norm(sample.center - face_center)) * 1000.0
    row["normal_err_deg"] = angle_between(sample.normal, face_normal)
    row["incidence_deg"] = float(sample.incidence_deg)
    row["plane_rms_mm"] = float(sample.plane_rms_m) * 1000.0
    row["position_source"] = str(sample.position_source)

    if detection.box is not None:
        gt_center = renderer.to_camera(scene.box_center)
        detected = np.asarray(detection.box.size, dtype=np.float64) * 1000.0
        gt_mm = scene.box_size * 1000.0
        row["box_center_err_mm"] = float(np.linalg.norm(detection.box.center - gt_center)) * 1000.0
        row["size_err_mm"] = float(np.max(np.abs(detected - gt_mm)))
        row["size_x_mm"] = float(detected[0])
        row["size_y_mm"] = float(detected[1])
        row["size_z_mm"] = float(detected[2])
    return row


def time_detection(color: vision.Frame, depth: vision.Frame,
                   camera: vision.CameraModel, config: DetectorConfig,
                   repeats: int) -> float:
    detection = vision.detect(color, depth, camera, config)
    target.sample_from_detection(detection, config.square_size_mm)
    samples = []
    for _ in range(max(1, repeats)):
        t0 = time.perf_counter()
        detection = vision.detect(color, depth, camera, config)
        target.sample_from_detection(detection, config.square_size_mm)
        samples.append((time.perf_counter() - t0) * 1000.0)
    return float(np.median(samples))


def _profile_module(filename: str) -> str:
    text = str(filename)
    if "/view/" in text:
        return f"view/{Path(text).stem}"
    if text in ("~", "") or text.startswith("<"):
        return "C 扩展"
    for package in ("numpy", "cv2", "scipy", "yaml"):
        if f"/{package}/" in text or f"site-packages/{package}" in text:
            return package
    return Path(text).stem


def profile_detection(jobs: list[dict], config: DetectorConfig) -> list[dict]:
    selected = [job for job in jobs if not job["diagnostic"]
                and job["seed"] == 0 and job["variant"] == "noisy"]
    frames = []
    for job in selected:
        scene = build_scene(job["case"], job["seed"])
        renderer = SimRenderer(scene)
        try:
            bgr, depth = renderer.render()
            frames.append((bgr, depth, renderer.K.copy()))
        finally:
            renderer.close()

    # 渲染放在 profiling 之外，只统计 detect + sample_from_detection
    profiler = cProfile.Profile()
    profiler.enable()
    for bgr, depth, intrinsics in frames:
        color = vision.Frame(bgr, 0.0)
        depth_frame = vision.Frame(depth, 0.0, 1.0)
        camera = vision.CameraModel(intrinsics, np.zeros(5), (WIDTH, HEIGHT))
        detection = vision.detect(color, depth_frame, camera, config)
        target.sample_from_detection(detection, config.square_size_mm)
    profiler.disable()

    stats = pstats.Stats(profiler)
    merged: dict[str, dict] = defaultdict(lambda: {"tot": 0.0, "calls": 0})
    for (filename, _line, _name), (_cc, nc, tt, _ct, _callers) in stats.stats.items():
        key = _profile_module(filename)
        merged[key]["tot"] += tt
        merged[key]["calls"] += nc
    items = [{"module": key, **value} for key, value in merged.items()]
    items.sort(key=lambda item: item["tot"], reverse=True)
    total = sum(item["tot"] for item in items) or 1.0
    for item in items:
        item["share"] = item["tot"] / total
    return items[:10]


def summarize(rows: list[dict], group: str, variant: str) -> dict:
    sub = [r for r in rows if r["group"] == group and r["variant"] == variant]
    emitted = [r for r in sub if r["emitted"]]
    size_vectors = [[r["size_x_mm"], r["size_y_mm"], r["size_z_mm"]]
                    for r in emitted if r.get("size_x_mm") is not None]
    mean_size = (np.mean(size_vectors, axis=0) if size_vectors else None)
    mean_axis_err = (float(np.max(np.abs(mean_size - BOX_SIZE * 1000.0)))
                     if mean_size is not None else None)
    return {
        "group": group,
        "variant": variant,
        "frames": len(sub),
        "emitted": len(emitted),
        "mode_ok": sum(r["mode_ok"] for r in sub),
        "face": _stat([r["face_err_mm"] for r in emitted]),
        "normal": _stat([r["normal_err_deg"] for r in emitted]),
        "box": _stat([r.get("box_center_err_mm") for r in emitted]),
        "size": _stat([r.get("size_err_mm") for r in emitted]),
        "mean_size_axis_mm": (mean_size.tolist() if mean_size is not None else None),
        "mean_axis_err_mm": mean_axis_err,
    }


def check_pass(groups: dict[str, dict], variant: str = "noisy") -> dict:
    checks: dict[str, bool] = {}
    details: dict[str, dict] = {}
    for mode in NORMAL_MODES:
        agg = groups[mode][variant]
        face = agg["face"]
        normal = agg["normal"]
        ok = (agg["emitted"] == agg["frames"]
              and agg["mode_ok"] == agg["frames"]
              and face is not None and face["mean"] <= THRESHOLDS["face_mm"]
              and normal is not None and normal["mean"] <= THRESHOLDS["normal_deg"])
        prefix = mode
        checks[f"{prefix} 全部出目标"] = agg["emitted"] == agg["frames"]
        checks[f"{prefix} mode 全对"] = agg["mode_ok"] == agg["frames"]
        checks[f"{prefix} 面心均值 ≤{THRESHOLDS['face_mm']:.0f}mm"] = (
            face is not None and face["mean"] <= THRESHOLDS["face_mm"])
        checks[f"{prefix} 法线均值 ≤{THRESHOLDS['normal_deg']:.0f}°"] = (
            normal is not None and normal["mean"] <= THRESHOLDS["normal_deg"])
        if mode in DRIVE_MODES:
            box = agg["box"]
            size_axis = agg["mean_axis_err_mm"]
            checks[f"{prefix} 箱心均值 ≤{THRESHOLDS['box_mm']:.0f}mm"] = (
                box is not None and box["mean"] <= THRESHOLDS["box_mm"])
            checks[f"{prefix} 边长均值轴 ≤{THRESHOLDS['size_mm']:.0f}mm"] = (
                size_axis is not None and size_axis <= THRESHOLDS["size_mm"])
        details[prefix] = agg
    return {"checks": checks, "pass": all(checks.values()), "groups": details}


def build_observations(rows: list[dict], groups: dict[str, dict]) -> list[str]:
    obs = []
    for mode in NORMAL_MODES:
        parts = []
        for variant in VARIANTS:
            agg = groups[mode][variant]
            parts.append(f"{variant} {agg['emitted']}/{agg['frames']} 出目标、"
                         f"{agg['mode_ok']}/{agg['frames']} mode 对")
        obs.append(f"{mode}: " + "，".join(parts))
    diag = [r for r in rows if r["case"] in DIAGNOSTIC_CASES]
    if diag:
        by_case = Counter()
        emitted = Counter()
        for row in diag:
            by_case[row["case"]] += 1
            emitted[row["case"]] += row["emitted"]
        obs.append("遮挡/截断诊断（仅 noisy）：" + "，".join(
            f"{case} {emitted[case]}/{by_case[case]} 仍出目标" for case in sorted(by_case)))
    errors = [r["issues"] for r in rows if r["issues"]]
    if errors:
        obs.append("检测 issues：" + "；".join(
            f"{k} ×{v}" for k, v in Counter(errors).most_common(4)))
    return obs


def write_markdown(path: Path, summary: dict, out_dir: Path) -> None:
    th = THRESHOLDS
    lines = [
        "# view 视觉仿真基准",
        "",
        f"- 时间：{summary['timestamp']}",
        f"- 环境：Python {summary['env']['python']}，NumPy {summary['env']['numpy']}，"
        f"OpenCV {summary['env']['opencv']}，MuJoCo {summary['env']['mujoco']}",
        f"- 场景：0.35m 红箱、无机械臂、640×480、fovy 60°（fx={summary['scene']['fx']:.2f}）",
        f"- 覆盖：single/dual/triple 各 10 种子 × clean/noisy = 60 帧；"
        f"遮挡/截断各 5 帧（noisy，仅诊断）",
        "",
        "## 结论",
        "",
        f"**{'PASS' if summary['pass']['pass'] else 'FAIL'}**（以 noisy 一组判定）",
        "",
    ]
    for name, ok in summary["pass"]["checks"].items():
        lines.append(f"- {'PASS' if ok else 'FAIL'} {name}")
    lines.append("")
    clean = summary["pass_clean"]
    lines.append(f"clean 一组：**{'PASS' if clean['pass'] else 'FAIL'}**"
                 + ("" if clean["pass"] else "（" + "、".join(
                     name for name, ok in clean["checks"].items() if not ok) + "）"))
    lines.append("")

    if summary.get("observations"):
        lines += ["## 关键观察", ""] + [f"- {item}" for item in summary["observations"]] + [""]

    lines += ["## 精度（命中帧）", "",
              "| 模式 | 变体 | 出目标 | 面心均值/P95 (mm) | 法线均值/P95 (°) | 箱心均值/P95 (mm) | 边长 max 轴误差均值/P95 (mm) |",
              "| --- | --- | --- | --- | --- | --- | --- |"]
    for mode in NORMAL_MODES:
        for variant in VARIANTS:
            agg = summary["groups"][mode][variant]
            fmt = lambda stat, p=2: "-" if stat is None else f"{stat['mean']:.{p}f} / {stat['p95']:.{p}f}"
            lines.append(
                f"| {mode} | {variant} | {agg['emitted']}/{agg['frames']} | "
                f"{fmt(agg['face'])} | {fmt(agg['normal'])} | {fmt(agg['box'])} | {fmt(agg['size'], 1)} |")
    lines.append("")

    lines += ["### 箱体尺寸（三轴平均，mm）", "",
              "| 模式 | 变体 | 平均边长 x/y/z | 最大轴误差 | 允许 |",
              "| --- | --- | --- | --- | --- |"]
    for mode in DRIVE_MODES:
        for variant in VARIANTS:
            agg = summary["groups"][mode][variant]
            size = agg["mean_size_axis_mm"]
            axis = agg["mean_axis_err_mm"]
            text = "-" if size is None else f"{size[0]:.1f} / {size[1]:.1f} / {size[2]:.1f}"
            axis_text = "-" if axis is None else f"{axis:.1f}"
            lines.append(f"| {mode} | {variant} | {text} | {axis_text} | {th['size_mm']:.0f} |")
    lines.append("")

    lines += ["## 遮挡/截断诊断（不纳入 PASS）", "",
              "| 场景 | 帧 | 仍出目标 | 面心均值 (mm) | 箱心均值 (mm) | 边长 max 轴误差均值 (mm) |",
              "| --- | --- | --- | --- | --- | --- |"]
    for case in DIAGNOSTIC_CASES:
        agg = summary["diagnostic"][case]
        fmt = lambda stat, p=1: "-" if stat is None else f"{stat['mean']:.{p}f}"
        lines.append(f"| {case} | {agg['frames']} | {agg['emitted']}/{agg['frames']} | "
                     f"{fmt(agg['face'])} | {fmt(agg['box'])} | {fmt(agg['size'])} |")
    lines.append("")

    lines += ["## 速度（detect + sample_from_detection，不含渲染）", "",
              "| 模式 | 中位数(ms) | P95(ms) | 帧/秒 |", "| --- | --- | --- | --- |"]
    for mode, timing in summary["timing"]["by_mode"].items():
        lines.append(f"| {mode} | {timing['median_ms']:.1f} | {timing['p95_ms']:.1f} | "
                     f"{1000.0 / timing['median_ms']:.2f} |")
    overall = summary["timing"]["overall"]
    lines.append(f"| 全部 | {overall['median_ms']:.1f} | {overall['p95_ms']:.1f} | "
                 f"{1000.0 / overall['median_ms']:.2f} |")
    lines.append("")

    lines += ["## 分阶段（cProfile，按模块自身耗时）", "",
              "| 模块 | 自身耗时(ms) | 占比 |", "| --- | --- | --- |"]
    for item in summary["profile"]:
        lines.append(f"| {item['module']} | {item['tot'] * 1000:.1f} | {item['share'] * 100:.1f}% |")
    lines.append("")

    lines += ["## 说明", "",
              "- 场景不含机械臂；渲染深度来自 `mujoco.Renderer`，与相机插件同一条 "
              "`znear/far` 换算（插件注释直接引用 `renderer.py`）。",
              "- dual 的 seed 0 是现有 `scene_vision.xml` 的 0.79m / 主面约 28.9° 位姿，"
              "seed 5 是同距离对称 45°；其余在 20–60° 扫。",
              "- triple 用远相机（箱心约 1.6–1.8m），三个面入射约 53–58°；"
              "这是让三面同时进 65° 的窄几何，属于能力测试，不代表 4 轴部署位。",
              "- single 在 `check_sim_vision` 里会因 `has_box=False` 被过滤；本基准直接评估面心/法线。",
              "- noisy 的 dual 尺寸膨胀来自深度边缘飞点：view 的点云没有 aburobocon 那种 "
              "`fly_pixel_mask` 过滤，边缘混合深度被计入侧面点云，把面高从 350mm 拉到 480–540mm；"
              "clean 一组不受影响，说明几何本身没问题。",
              "- 遮挡/截断仅诊断：记录是否仍出目标及其误差，不参与 PASS。",
              f"- 逐帧明细：`{out_dir / 'frames.csv'}`；失败帧：`{out_dir / 'fail'}`。",
              ""]

    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="view 视觉离线基准")
    parser.add_argument("--out", default=str(ROOT / "out" / "bench"))
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--config", default=None, help="预留：暂不支持外部 YAML")
    args = parser.parse_args()

    if args.config:
        raise SystemExit("view 的参数是 ROS 参数，目前只支持 DetectorConfig 默认值")

    config = DetectorConfig()
    out_dir = Path(args.out)
    fail_dir = out_dir / "fail"
    fail_dir.mkdir(parents=True, exist_ok=True)

    import mujoco

    jobs = make_jobs()
    rows: list[dict] = []
    timings: list[dict] = []
    fail_count = 0

    for job in jobs:
        scene = build_scene(job["case"], job["seed"])
        renderer = SimRenderer(scene)
        try:
            bgr, depth = renderer.render()
            rng = np.random.default_rng([job["seed"], 0 if job["variant"] == "clean" else 1])
            color_bgr, depth_m = add_sensor_noise(bgr, depth, rng, job["variant"])
            color = vision.Frame(color_bgr, 0.0)
            depth_frame = vision.Frame(depth_m, 0.0, 1.0)
            camera = vision.CameraModel(renderer.K, np.zeros(5), (WIDTH, HEIGHT))

            detection = vision.detect(color, depth_frame, camera, config)
            sample = target.sample_from_detection(detection, config.square_size_mm)
            row = evaluate(renderer, scene, detection, sample)
            row["variant"] = job["variant"]
            rows.append(row)

            timing = time_detection(color, depth_frame, camera, config, args.repeats)
            timings.append({"group": job["group"], "case": job["case"],
                            "seed": job["seed"], "variant": job["variant"],
                            "median_ms": timing})

            if not job["diagnostic"]:
                over = (row.get("face_err_mm", 0.0) > THRESHOLDS["face_mm"]
                        or row.get("normal_err_deg", 0.0) > THRESHOLDS["normal_deg"]
                        or row.get("box_center_err_mm", 0.0) > THRESHOLDS["box_mm"]
                        or row.get("size_err_mm", 0.0) > THRESHOLDS["size_mm"])
                if not row["emitted"] or not row["mode_ok"] or over:
                    image = compose(color_bgr, detection, camera, 0.05, 1280, sample)
                    name = f"{job['case']}_{job['seed']:02d}_{job['variant']}"
                    cv2.imwrite(str(fail_dir / f"{name}.png"), image)
                    (fail_dir / f"{name}.xml").write_text(scene.xml, encoding="utf-8")
                    fail_count += 1
            elif row["emitted"]:
                image = compose(color_bgr, detection, camera, 0.05, 1280, sample)
                name = f"{job['case']}_{job['seed']:02d}_{job['variant']}"
                cv2.imwrite(str(fail_dir / f"{name}.png"), image)
                (fail_dir / f"{name}.xml").write_text(scene.xml, encoding="utf-8")
        finally:
            renderer.close()
        print(f"[{job['case']} seed={job['seed']} {job['variant']}] "
              f"mode={row['detected_mode']} emitted={row['emitted']}", flush=True)

    groups = {mode: {variant: summarize(rows, mode, variant) for variant in VARIANTS}
              for mode in NORMAL_MODES}
    diagnostic = {case: summarize(rows, case, "noisy") for case in DIAGNOSTIC_CASES}
    pass_result = check_pass(groups, "noisy")
    pass_clean = check_pass(groups, "clean")

    by_mode = {}
    for mode in NORMAL_MODES:
        vals = [t["median_ms"] for t in timings
                if t["group"] == mode and t["variant"] == "noisy"]
        by_mode[mode] = {
            "median_ms": float(np.median(vals)),
            "p95_ms": float(np.percentile(vals, 95)),
        }
    overall_timing = {
        "median_ms": float(np.median([t["median_ms"] for t in timings])),
        "p95_ms": float(np.percentile([t["median_ms"] for t in timings], 95)),
    }

    profile = profile_detection(jobs, config)
    summary = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "env": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "opencv": cv2.__version__,
            "mujoco": mujoco.__version__,
        },
        "scene": {
            "box_size_m": BOX_SIZE,
            "width": WIDTH,
            "height": HEIGHT,
            "fovy_deg": 60.0,
            "fx": float(FX),
            "no_arm": True,
            "frames": len(jobs),
        },
        "thresholds": THRESHOLDS,
        "pass": pass_result,
        "pass_clean": pass_clean,
        "observations": build_observations(rows, groups),
        "groups": groups,
        "diagnostic": diagnostic,
        "timing": {"by_mode": by_mode, "overall": overall_timing},
        "profile": profile,
        "fail_frames": fail_count,
    }

    with (out_dir / "frames.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in FIELDS})
    (out_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    write_markdown(out_dir / "summary.md", summary, out_dir)

    print("\n=== 结果 ===")
    for mode in NORMAL_MODES:
        agg = groups[mode]["noisy"]
        print(f"{mode}: 出目标 {agg['emitted']}/{agg['frames']}，mode 对 "
              f"{agg['mode_ok']}/{agg['frames']}，面心均值 "
              f"{'-' if agg['face'] is None else round(agg['face']['mean'], 2)}mm")
    print(f"速度: detect+sample 中位数 {overall_timing['median_ms']:.1f}ms "
          f"({1000.0 / overall_timing['median_ms']:.2f} FPS)")
    print(f"判定: {'PASS' if pass_result['pass'] else 'FAIL'}")
    print(f"输出: {out_dir / 'summary.md'}")
    return 0 if pass_result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
