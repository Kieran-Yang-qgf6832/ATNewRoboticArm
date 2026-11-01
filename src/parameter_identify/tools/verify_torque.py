#!/usr/bin/env python3
"""验证辨识出的 URDF 的力矩预测精度（docs/bugs.md O8 / C7）。

为什么需要单独一个工具：identify_arm 报告里的 rmse 量的是「最小二乘基参数」的拟合，
而写进 URDF 的是「物理投影之后的完整参数」，两者可以差几十倍（实测 iter30：报告 0.57%，
URDF 实际 41.8%）。判断一个 URDF 能不能用，必须拿它自己正演一遍。

两条路径，按 CSV 的列数自动选择：

1. **有 acc_* 列**（仿真 CSV，1+4*dof 列）：用 pinocchio 的 RNEA 以记录的 q̈ 逐样本正演。
2. **没有 acc_* 列**（实机 CSV，1+3*dof 列）：复用辨识链路的分部积分滤波回归量
   （`filtered_identification.build_acceleration_free_regressor`），把该 URDF 的参数
   当作固定 φ、算残差 `τ_f − W_f φ` 的 RMS。q̈ 在这一步被解析消掉，所以不需要加速度。

用法：
    python3 verify_torque.py --urdf out.urdf --csv train.csv [--csv holdout.csv ...]
                             [--reference src/arm_model/model/robotic_arm.urdf]
                             [--config src/parameter_identify/config/identify.yaml]

**第一份 `--csv` 视为训练集，其余视为留出集**（只有需要打印判据时才这么分）。

实机判据（docs/parameter_identification_guide.md 第 5 节）：
  - 留出集残差 RMS ≤ 5% 额定力矩；
  - 留出集不比训练集差太多（默认 1.5 倍以内）；
  - 被测 URDF 的残差应小于参照（先验）URDF——否则辨识没有带来收益。

输出（对每份 CSV）：
  - RNEA 路径：逐关节 + 总体的 平均/最大 相对误差  ‖tau_pred - tau_meas‖ / ‖tau_meas‖
  - 滤波路径：残差 RMS（N·m）与占额定力矩的百分比（逐关节 + 最大）
  - 与 --reference 的对比：辨识没引入额外误差时两者应基本一致
  - 残差结构：|残差| 与 |dq|（RNEA 路径还有 |acc|）的相关系数
    （明显正相关 = 有摩擦/阻尼等未建模项）
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin

try:  # 源码树直接运行（python3 src/parameter_identify/tools/verify_torque.py）
    from parameter_identify.identify_arm import (
        _default_config_path,
        _load_config,
        build_filtered_regression,
    )
except ImportError:  # 兜底：把包根插进 sys.path 再试一次
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from parameter_identify.identify_arm import (  # type: ignore[no-redef]
        _default_config_path,
        _load_config,
        build_filtered_regression,
    )

# 判据阈值（可按需在此调整，命令行可覆盖）
MAX_RATED_FRACTION = 0.05  # 留出集残差 ≤ 5% 额定力矩
MAX_HOLDOUT_RATIO = 1.5  # 留出集 ≤ 1.5 × 训练集


def load_csv(path: str, dof: int) -> dict:
    """读 CSV，按列数判断有无 acc_* 列。

    仿真：time + pos + vel + acc + torque = 1 + 4*dof
    实机：time + pos + vel + torque     = 1 + 3*dof
    """
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    columns = data.shape[1]
    if columns >= 1 + 4 * dof:
        has_acc = True
    elif columns >= 1 + 3 * dof:
        has_acc = False
    else:
        raise ValueError(
            f"{path}: 需要 1+4*{dof} 列（time,pos,vel,acc,torque）或 1+3*{dof} 列"
            f"（time,pos,vel,torque），实际 {columns} 列"
        )

    off = 1
    out = {
        "time": data[:, 0],
        "pos": data[:, off : off + dof],
        "vel": data[:, off + dof : off + 2 * dof],
        "acc": data[:, off + 2 * dof : off + 3 * dof] if has_acc else None,
        "torque": data[:, off + (3 if has_acc else 2) * dof : off + (4 if has_acc else 3) * dof],
        "has_acc": has_acc,
        "rows": int(data.shape[0]),
    }
    if out["torque"].shape[1] != dof:
        raise ValueError(f"{path}: 解析出的力矩列数为 {out['torque'].shape[1]}，期望 {dof}")
    return out


def predict(urdf: str, d: dict) -> np.ndarray:
    model = pin.buildModelFromUrdf(urdf)
    data = model.createData()
    if model.nv != d["pos"].shape[1]:
        raise ValueError(f"{urdf}: 模型 nv={model.nv} 与 CSV 关节数 {d['pos'].shape[1]} 不一致")
    n = d["pos"].shape[0]
    pred = np.empty((n, model.nv))
    for i in range(n):
        pred[i] = pin.rnea(model, data, d["pos"][i], d["vel"][i], d["acc"][i])
    return pred


def report_rnea(tag: str, urdf: str, csv: str, d: dict) -> dict:
    """仿真路径：用 q̈ 做 RNEA 正演，比较预测与实测力矩。"""
    pred = predict(urdf, d)
    tau = d["torque"]
    res = pred - tau
    scale = np.linalg.norm(tau, axis=1).mean()
    per_joint = np.abs(res).mean(axis=0) / np.maximum(np.abs(tau).mean(axis=0), 1e-12)
    overall_mean = np.linalg.norm(res, axis=1).mean() / scale
    overall_max = np.linalg.norm(res, axis=1).max() / scale

    # 残差结构：和速度/加速度的相关性（正相关说明有未建模的摩擦/阻尼）
    r = np.abs(res).mean(axis=1)
    v = np.abs(d["vel"]).mean(axis=1)
    a = np.abs(d["acc"]).mean(axis=1)
    corr_v = float(np.corrcoef(r, v)[0, 1]) if r.std() > 0 and v.std() > 0 else float("nan")
    corr_a = float(np.corrcoef(r, a)[0, 1]) if r.std() > 0 and a.std() > 0 else float("nan")

    print(f"  [{tag}] {Path(urdf).name}  vs  {Path(csv).name}   ({d['rows']} 样本，RNEA 正演)")
    print(f"      总体相对误差  平均 {100*overall_mean:8.4f}%   最大 {100*overall_max:8.4f}%")
    print(f"      逐关节平均误差 [%]: " + " ".join(f"{100*x:7.3f}" for x in per_joint))
    print(f"      残差 vs |dq| 相关 {corr_v:+.3f}   残差 vs |acc| 相关 {corr_a:+.3f}")
    return {"mean": overall_mean, "max": overall_max, "per_joint": per_joint, "rms": float("nan")}


def report_filtered(tag: str, urdf: str, csv: str, config, d: dict) -> dict:
    """实机路径：无 q̈，用分部积分滤波回归量算残差 RMS（按额定力矩归一）。"""
    base = build_filtered_regression(urdf, csv, config)
    residual = base["tau"] - base["W"] @ base["phi"]
    n_active = base["n_active"]

    rms_all = float(np.sqrt(np.mean(residual**2)))
    res_mat = residual.reshape(n_active, -1)
    rms_per_joint = np.sqrt(np.mean(res_mat**2, axis=1))
    effort = np.maximum(base["effort"], 1e-12)
    pct_per_joint = 100.0 * rms_per_joint / effort
    pct_max = float(np.max(pct_per_joint))

    # 残差结构：滤波回归量丢掉了开头 drop 段，这里对齐到保留下来的样本
    kept = base["kept_samples"]
    vel = np.abs(d["vel"])[-kept:] if kept <= d["vel"].shape[0] else d["vel"]
    r = np.abs(res_mat).mean(axis=0)
    v = np.abs(vel).mean(axis=1)
    if r.size == v.size and r.std() > 0 and v.std() > 0:
        corr_v = float(np.corrcoef(r, v)[0, 1])
    else:
        corr_v = float("nan")

    print(
        f"  [{tag}] {Path(urdf).name}  vs  {Path(csv).name}   "
        f"({kept} 样本 @{1000*base['sample_time']:.2f} ms，无 acc_* → 滤波回归量)"
    )
    print(
        f"      残差 RMS {rms_all:8.5f} N·m   占额定力矩 最大 {pct_max:6.3f}%"
        f"   （判据 ≤ {100*MAX_RATED_FRACTION:.0f}%）"
    )
    print(f"      逐关节 [% 额定]: " + " ".join(f"{x:7.3f}" for x in pct_per_joint))
    print(f"      残差 vs |dq| 相关 {corr_v:+.3f}")
    return {
        "mean": float(np.mean(pct_per_joint)) / 100.0,
        "max": pct_max / 100.0,
        "per_joint": pct_per_joint / 100.0,
        "rms": rms_all,
        "pct_max": pct_max,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="验证 URDF 的力矩预测精度")
    ap.add_argument("--urdf", required=True, help="要验证的 URDF（通常是辨识输出）")
    ap.add_argument(
        "--csv",
        action="append",
        required=True,
        help="测量 CSV，可重复传多份；第一份视为训练集，其余视为留出集",
    )
    ap.add_argument("--reference", default=None, help="参考模型（真值 / 先验 URDF），用于给出误差地板")
    ap.add_argument(
        "--config",
        default=None,
        help="辨识 YAML 配置（滤波路径需要；默认用 parameter_identify 自带的 identify.yaml）",
    )
    ap.add_argument("--dof", type=int, default=6)
    args = ap.parse_args()

    config_path = Path(args.config) if args.config else _default_config_path()
    try:
        config = _load_config(config_path)
    except Exception as exc:  # 配置读不到时只有滤波路径不可用，RNEA 路径仍可跑
        print(f"警告: 读取配置 {config_path} 失败（{exc}）；无 acc_* 的 CSV 将无法验证")
        config = None

    rc = 0
    records = []
    for index, csv in enumerate(args.csv):
        split = "train" if index == 0 else "holdout"
        print(f"=== {csv} ===")
        try:
            d = load_csv(csv, args.dof)
        except Exception as exc:
            print(f"  失败: {exc}")
            rc = 1
            continue

        try:
            if d["has_acc"]:
                measured = report_rnea("被测", args.urdf, csv, d)
                reference = report_rnea("参照", args.reference, csv, d) if args.reference else None
            else:
                if config is None:
                    raise RuntimeError(
                        "这份 CSV 没有 acc_* 列，需要 --config（或包内 identify.yaml）走滤波回归量"
                    )
                measured = report_filtered("被测", args.urdf, csv, config, d)
                reference = (
                    report_filtered("参照", args.reference, csv, config, d) if args.reference else None
                )
        except Exception as exc:
            print(f"  失败: {exc}")
            rc = 1
            continue

        if reference is not None:
            if d["has_acc"]:
                extra = (measured["mean"] - reference["mean"]) / max(reference["mean"], 1e-12)
                print(f"      相对参照模型多出的误差: {100*extra:+.2f}%"
                      f"   （≈0 说明辨识没引入额外误差）")
            else:
                ratio = measured["pct_max"] / max(reference["pct_max"], 1e-12)
                print(
                    f"      相对参照模型: {ratio:.3f}×（<1 说明辨识后比先验更贴合这份数据）"
                )
                measured["ref_pct_max"] = reference["pct_max"]
                measured["ref_ratio"] = ratio
        records.append({"csv": csv, "split": split, **measured})
        print()

    # 判据只在滤波路径（实机）且至少两份 CSV 时给出：RNEA 路径沿用原有输出习惯。
    filtered_records = [r for r in records if "pct_max" in r]
    train = next((r for r in filtered_records if r["split"] == "train"), None)
    holdouts = [r for r in filtered_records if r["split"] == "holdout"]
    if filtered_records and train is not None:
        print("=== 判据（残差按额定力矩归一）===")
        print(f"  训练集 {Path(train['csv']).name:28} 被测 {train['pct_max']:6.3f}%")
        for holdout in holdouts:
            print(f"  留出集 {Path(holdout['csv']).name:28} 被测 {holdout['pct_max']:6.3f}%")
        if not holdouts:
            print("  [INFO] 只给了训练集：只能说明拟合能力，无法判断泛化；建议再录一条换 seed 的 CSV")
        for holdout in holdouts:
            ok = holdout["pct_max"] <= 100.0 * MAX_RATED_FRACTION
            print(
                f"  [{'PASS' if ok else 'FAIL'}] 留出集 {holdout['pct_max']:.3f}% "
                f"{'≤' if ok else '>'} {100*MAX_RATED_FRACTION:.0f}% 额定力矩"
            )
            if not ok:
                rc = 1
            ratio = holdout["pct_max"] / max(train["pct_max"], 1e-12)
            ok = ratio <= MAX_HOLDOUT_RATIO
            print(
                f"  [{'PASS' if ok else 'FAIL'}] 留出/训练 = {ratio:.2f} "
                f"{'≤' if ok else '>'} {MAX_HOLDOUT_RATIO:.2f}"
            )
            if not ok:
                rc = 1
        ratios = [r["ref_ratio"] for r in filtered_records if "ref_ratio" in r]
        if ratios:
            worst = max(ratios)
            if worst < 1.0:
                print(f"  [PASS] 被测/参照 最差 {worst:.3f}× <1，辨识后比先验更贴合数据")
            elif worst <= 1.05:
                print(f"  [INFO] 被测/参照 最差 {worst:.3f}×，与先验基本持平（无法证明收益）")
            else:
                print(f"  [FAIL] 被测/参照 最差 {worst:.3f}× >1.05，辨识后反而更差")
                rc = 1
        print()

    return rc


if __name__ == "__main__":
    sys.exit(main())
