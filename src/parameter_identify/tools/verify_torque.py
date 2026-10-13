#!/usr/bin/env python3
"""验证辨识出的 URDF 的**力矩预测**精度（docs/bugs.md O8 / C7）。

为什么需要单独一个工具：identify_arm 报告里的 rmse 量的是「最小二乘基参数」的拟合，
而写进 URDF 的是「物理投影之后的完整参数」，两者可以差几十倍（实测 iter30：报告 0.57%，
URDF 实际 41.8%）。判断一个 URDF 能不能用，必须拿它自己正演一遍。

用法：
    python3 verify_torque.py --urdf out.urdf --csv train.csv [--csv test.csv ...]
                             [--reference src/arm_model/model/robotic_arm.urdf]

输出（对每份 CSV）：
  - 逐关节 + 总体的 平均/最大 相对误差   ‖tau_pred - tau_meas‖ / ‖tau_meas‖
  - 与 --reference（真值模型）的对比：辨识没引入额外误差时两者应基本一致
  - 残差结构：|残差| 与 |dq| / |acc| 的相关系数（明显正相关 = 有摩擦/阻尼等未建模项）
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin


def load_csv(path: str, dof: int) -> dict[str, np.ndarray]:
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    if data.ndim != 2 or data.shape[1] < 1 + 4 * dof:
        raise ValueError(f"{path}: 需要 1+4*{dof} 列（time,pos,vel,acc,torque），实际 {data.shape[1]} 列")
    off = 1
    out = {
        "time": data[:, 0],
        "pos": data[:, off : off + dof],
        "vel": data[:, off + dof : off + 2 * dof],
        "acc": data[:, off + 2 * dof : off + 3 * dof],
        "torque": data[:, off + 3 * dof : off + 4 * dof],
    }
    return out


def predict(urdf: str, d: dict[str, np.ndarray]) -> np.ndarray:
    model = pin.buildModelFromUrdf(urdf)
    data = model.createData()
    if model.nv != d["pos"].shape[1]:
        raise ValueError(f"{urdf}: 模型 nv={model.nv} 与 CSV 关节数 {d['pos'].shape[1]} 不一致")
    n = d["pos"].shape[0]
    pred = np.empty((n, model.nv))
    for i in range(n):
        pred[i] = pin.rnea(model, data, d["pos"][i], d["vel"][i], d["acc"][i])
    return pred


def report(tag: str, urdf: str, csv: str, dof: int) -> dict:
    d = load_csv(csv, dof)
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

    print(f"  [{tag}] {Path(urdf).name}  vs  {Path(csv).name}   ({d['pos'].shape[0]} 样本)")
    print(f"      总体相对误差  平均 {100*overall_mean:8.4f}%   最大 {100*overall_max:8.4f}%")
    print(f"      逐关节平均误差 [%]: " + " ".join(f"{100*x:7.3f}" for x in per_joint))
    print(f"      残差 vs |dq| 相关 {corr_v:+.3f}   残差 vs |acc| 相关 {corr_a:+.3f}")
    return {"mean": overall_mean, "max": overall_max, "per_joint": per_joint}


def main() -> int:
    ap = argparse.ArgumentParser(description="验证 URDF 的力矩预测精度")
    ap.add_argument("--urdf", required=True, help="要验证的 URDF（通常是辨识输出）")
    ap.add_argument("--csv", action="append", required=True, help="测量 CSV，可重复传多份")
    ap.add_argument("--reference", default=None, help="参考模型（真值 URDF），用于给出误差地板")
    ap.add_argument("--dof", type=int, default=6)
    args = ap.parse_args()

    rc = 0
    for csv in args.csv:
        print(f"=== {csv} ===")
        try:
            m = report("被测", args.urdf, csv, args.dof)
        except Exception as exc:
            print(f"  失败: {exc}")
            rc = 1
            continue
        if args.reference:
            ref = report("参照", args.reference, csv, args.dof)
            extra = (m["mean"] - ref["mean"]) / max(ref["mean"], 1e-12)
            print(f"      相对参照模型多出的误差: {100*extra:+.2f}%"
                  f"   （≈0 说明辨识没引入额外误差）")
        print()
    return rc


if __name__ == "__main__":
    sys.exit(main())
