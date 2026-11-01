"""不依赖关节加速度的参数辨识：分部积分滤波回归量。

适用场景
--------
实机的 MCU 只上报关节位置/速度/力矩，**没有加速度**；对速度做数值微分得到的 q̈
相对误差 ~30%，会把辨识结果推出物理可行集（docs/bugs.md O8）。这里改用
「滤波 + 分部积分」把 q̈ 解析消掉，只吃 q、q̇、τ。

推导
----
刚体动力学对参数是线性的：

    τ = A(q) q̈ + h(q, q̇)

其中 h 已经包含科氏/重力/摩擦/偏置（这些都不含 q̈），A(q) 是 q̈ 的线性系数
（∂Y/∂q̈）。对整条方程施加同一个一阶低通 L(s) = a/(s+a)（a = 2π f_c，相位滞后
对两边相同、相互抵消）：

    L τ = L(A q̈) + L h

而 A q̈ = d/dt(A q̇) − Ȧ q̇，于是（`s·L(s) = a − a·L(s)`，把 Ḃ 的导数转嫁给滤波核）：

    L(A q̈) = a·B − a·L(B) − L(V) − a·e^{−at}·B(0)

末项是零初值滤波的初值瞬态：t→∞ 指数衰减，靠丢开头 drop_initial_seconds 吸收。
2 Hz 时 a≈12.57，e^{−a·0.5}≈0.002，残余约为 B 量级的 0.2%。合起来：

    W_f = a·B − a·L(B) − L(V) + L(H),    τ_f = L τ

    B = A(q) q̇                （由 Y(q, 0, q̇) − Y(q, 0, 0) 得到：速度列传 0，
                                摩擦列恒为 0；q̇=0 的重力与偏置相减抵消，
                                所以 B 只含 A(q)q̇，不含摩擦）
    V = Ȧ q̇ = ∂B/∂q · q̇      （方向导数，用中心差分算，见下）
    H = Y(q, q̇, 0)            （不含 q̈ 的全部项；摩擦与偏置只在 H 里出现）

这几项都只由 q、q̇ 得到，**q̈ 从未出现**，而且惯性参数没有被消掉（基参数集不变）。

方向导数：
    Ȧ q̇ = d/dε B(q + ε q̇, q̇) |_{ε=0}
取 ε = h / ‖q̇‖（使扰动幅度恒为 h ≈ 1e-6 rad），中心差分即为 V。

数值细节
--------
- 低通写成因果一阶递归 y_k = α y_{k-1} + (1−α) x_k，α = e^{−a Δt}。
- 零初值起步，开头 drop_initial_seconds 秒是滤波器瞬态，整段丢掉（不进回归）。
- Δt 从 CSV 的 time 列取中位数（实机 50/100 Hz、仿真 500 Hz 自动适配）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np


@dataclass
class FilteredRegressor:
    """滤波回归量的结果（行按「样本 × 关节」展开，与 FIGAROH 的布局一致）。"""

    regressor: np.ndarray  # (n_kept * nv, n_columns)
    torques: np.ndarray  # (n_kept, nv)
    kept_indices: np.ndarray  # 保留下来的样本下标（原 CSV 行号）
    effective_sample_time: float


def estimate_sample_time(time: np.ndarray, fallback: float) -> float:
    """从 time 列取采样步长（中位数），单样本或时间列不单调时退回配置值。"""
    if time.size >= 2:
        deltas = np.diff(time)
        deltas = deltas[deltas > 0.0]
        if deltas.size > 0:
            return float(np.median(deltas))
    if fallback <= 0.0:
        raise ValueError("sample_time must be positive when the CSV has fewer than two samples")
    return float(fallback)


def _effective_cutoff(cutoff_hz: float, sample_time: float) -> float:
    """返回与离散 IIR 精确匹配的等效 a（连续域是 2πf_c）。

    一阶 IIR 是 `H(z) = (1−α)/(1−αz⁻¹)`，速度用后向差分 `D(z) = (1−z⁻¹)/dt`。
    离散恒等式 `D·H = a_eff·(1 − H)` 当且仅当

        a_eff = (1 − α) / (α · dt)

    成立；直接拿连续域的 `a = 2πf_c` 代入会留下 O(a·dt) 的失配
    （2 Hz 在 100 Hz 采样下约 7%，50 Hz 采样下约 14%）。这份失配会让
    「用真值参数拟合滤波回归量」都残留 ~1% 的误差，再被最小二乘吸收进参数。
    """
    a = 2.0 * math.pi * float(cutoff_hz)
    alpha = math.exp(-a * sample_time)
    if alpha >= 1.0 or alpha <= 0.0:
        return a
    return (1.0 - alpha) / (alpha * sample_time)


def lowpass(
    values: np.ndarray, sample_time: float, cutoff_hz: float, axis: int = 0
) -> np.ndarray:
    """因果一阶低通（沿 `axis` 方向的时间），零初值。"""
    if cutoff_hz <= 0.0:
        return np.array(values, dtype=float, copy=True)
    alpha = math.exp(-2.0 * math.pi * cutoff_hz * sample_time)
    values = np.asarray(values, dtype=float)
    try:
        from scipy.signal import lfilter

        return lfilter([1.0 - alpha], [1.0, -alpha], values, axis=axis)
    except Exception:
        out = np.empty_like(values)
        state = np.zeros(values.shape[1:], dtype=float)
        for index in range(values.shape[0]):
            state = alpha * state + (1.0 - alpha) * values[index]
            out[index] = state
        return out


def build_acceleration_free_regressor(
    robot: Any,
    positions: np.ndarray,
    velocities: np.ndarray,
    torques: np.ndarray,
    identification_config: Mapping[str, Any],
    build_regressor_basic: Any,
    *,
    cutoff_hz: float,
    sample_time: float,
    drop_initial_seconds: float,
    finite_difference_step: float = 1.0e-6,
    derivative_scheme: str = "identity",
) -> FilteredRegressor:
    """按上面的推导，算出不含 q̈ 的回归矩阵与同样低通过的力矩。

    Args:
        robot: FIGAROH 的 RobotModel 包装（含 pinocchio model）。
        positions/velocities/torques: (S, nv) 测量数据。
        identification_config: FIGAROH 的 identification 配置（决定摩擦/偏置列的开关）。
        build_regressor_basic: FIGAROH 的 build_regressor_basic。
        cutoff_hz: 低通截止频率（默认 2 Hz）。
        sample_time: 采样步长（秒），建议用 estimate_sample_time 从 CSV 求。
        drop_initial_seconds: 丢掉开头多少秒（滤波器瞬态）。
        finite_difference_step: 方向导数的扰动幅度（rad）。

    Returns:
        FilteredRegressor，行数已去掉开头瞬态。
    """
    nv = positions.shape[1]
    sample_count = positions.shape[0]
    zeros = np.zeros_like(velocities)

    # H = Y(q, q̇, 0)：不含 q̈ 的全部项（科氏/重力/摩擦/偏置都在里面）。
    H = build_regressor_basic(robot, positions, velocities, zeros, identification_config)

    # B = A(q) q̇：用 q̇=0、q̈=q̇ 求值，再减掉 q̈=0 的重力项。
    # 速度列传的是 0，两次求值的摩擦列本来就为 0；q̇=0 时的偏置列在相减中抵消。
    # 因此相减结果里只剩 A(q)q̇，不含摩擦/偏置（它们在 H 里）。
    speed = np.linalg.norm(velocities, axis=1)
    eps = finite_difference_step / np.maximum(speed, finite_difference_step)

    q_plus = positions + eps[:, None] * velocities
    q_minus = positions - eps[:, None] * velocities
    b_plus = _inertia_part(robot, build_regressor_basic, identification_config, q_plus, velocities)
    b_minus = _inertia_part(robot, build_regressor_basic, identification_config, q_minus, velocities)

    # 中心差分的平均给出 B(q)，差值给出 Ȧ q̇（= ∂B/∂q·q̇ 的方向导数）。
    # FIGAROH 的回归矩阵是「关节优先」：行号 = 关节 * 样本数 + 样本序号。先摊平成
    # (关节, 样本, 列)，时间就在 axis=1 上；再除以逐关节的 ε（一个 (1, S, 1) 广播，
    # 不会像 (S,1,1) 那样把数组撑爆）。
    columns = b_plus.shape[1]
    b_plus = b_plus.reshape(nv, sample_count, columns)
    b_minus = b_minus.reshape(nv, sample_count, columns)
    b = 0.5 * (b_plus + b_minus)
    v_term = (b_plus - b_minus) / (2.0 * eps[None, :, None])

    h = H.reshape(nv, sample_count, columns)

    a_eff = _effective_cutoff(cutoff_hz, sample_time)
    if derivative_scheme == "central":
        # 离线中心差分：DB 的误差是 O(dt²)，比 a_eff 相消法（隐含后向差分，O(dt)）
        # 更准，代价是对测量噪声更敏感（低通会把它压回去）。
        d_b = np.gradient(b, sample_time, axis=1)
        inertia_filtered = lowpass(d_b, sample_time, cutoff_hz, axis=1)
    else:
        inertia_filtered = a_eff * b - a_eff * lowpass(b, sample_time, cutoff_hz, axis=1)
    filtered = (
        inertia_filtered
        - lowpass(v_term, sample_time, cutoff_hz, axis=1)
        + lowpass(h, sample_time, cutoff_hz, axis=1)
    )
    tau_filtered = lowpass(torques, sample_time, cutoff_hz, axis=0)

    # 零初值起步的瞬态在开头 drop_initial_seconds 秒内，整段丢掉（不进回归）。
    drop = int(np.ceil(max(drop_initial_seconds, 0.0) / max(sample_time, 1e-12)))
    drop = min(drop, sample_count)
    kept = np.arange(drop, sample_count)

    return FilteredRegressor(
        regressor=filtered[:, kept].reshape(nv * kept.size, columns),
        torques=tau_filtered[kept],
        kept_indices=kept,
        effective_sample_time=float(sample_time),
    )


def _inertia_part(robot: Any, build_regressor_basic: Any, identification_config: Mapping[str, Any], q: np.ndarray, qd: np.ndarray) -> np.ndarray:
    """Y(q, 0, q̇) − Y(q, 0, 0)：只剩 A(q)q̇ 的列。

    速度列传的是 0，所以摩擦列两次求值都是 0；q̇=0 时的重力与偏置在相减中抵消。
    """
    zeros = np.zeros_like(q)
    with_acceleration = build_regressor_basic(robot, q, zeros, qd, identification_config)
    gravity_only = build_regressor_basic(robot, q, zeros, zeros, identification_config)
    return with_acceleration - gravity_only
