#!/usr/bin/env python3
"""多路点轨迹（waypoint）仿真验收工具。

控制器在 `joint_traj` / `cart_traj` 状态里接收多路点轨迹，规划成功后会在
`arm_waypoints_duration`（std_msgs/msg/Float64）发布实际规划总时长，失败时发布 0.0。
本工具在收到正的规划时长后开始记录 `/joint_states`，只分析 `[t0, t0 + planned + margin]`
这一段窗口（因此前面的 reset / joint_move 不会被算进平滑度指标），然后检查：

  1. 关节位置始终在 URDF 限位内；
  2. joint 模式下，记录轨迹对每个期望用户路点的最小 max-abs 距离 <= tolerance；
     cart 模式下脚本内没有 FK，路点验收交给任务执行器，只打印提示；
  3. 对位置序列估计出的 velocity / acceleration / jerk 峰值不超过
     limit * 1.10 + 数值噪声下限。估计器默认是对位置做 Savitzky-Golay 低通后
     再求解析导数（`--smooth-window` 默认 0.20 s / `--polyorder` 3，先按时间戳
     把丢拍造成的非等间隔样本线性重采样）：500 Hz 伺服跟踪纹波会被裸二阶/三阶
     差分放大几个数量级，直接差分「测到的是噪声」而不是规划量。

用法（先启动仿真，再在轨迹执行前运行本工具）：

  ros2 run tasks waypoint_check --mode joint \
      --expected '[[0,0,0,0,0,0],[0.3,0.2,-0.2,0,0,0]]'

  ros2 run tasks waypoint_check --mode cart --tolerance 0.005 \
      --expected '[[0.35,0.0,0.25,0,0,0,1]]'

退出码：0 = 全部通过；1 = 任一检查失败（含超时、控制器拒绝规划）。
"""

import argparse
import json
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

try:
    from scipy.signal import savgol_filter

    HAVE_SCIPY = True
except ImportError:
    HAVE_SCIPY = False

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# URDF 限位（rad）：joint1 ±1.8、joint2 ±1.570796327、joint3 ±2.9、
# joint4 -2.0..1.8、joint5 ±0.610865238、joint6 ±3.14159
JOINT_LIMITS = [
    (-1.8, 1.8),
    (-1.570796327, 1.570796327),
    (-2.9, 2.9),
    (-2.0, 1.8),
    (-0.610865238, 0.610865238),
    (-3.14159, 3.14159),
]

# 数值噪声下限：差分放大的采样噪声，阈值 = limit * 1.10 + noise。
NOISE_FLOOR = {"vel": 2e-3, "acc": 0.05, "jerk": 0.5}

# Savitzky-Golay 窗口最少样本数与默认值（见 --smooth-window / --polyorder）。
MIN_SMOOTH_SAMPLES = 11

# 时间戳不可用时假定 /joint_states 的发布频率。
FALLBACK_RATE_HZ = 500.0


def parse_limit(text):
    """接受单值或逗号分隔的 6 值，返回 (6,) 数组。"""
    parts = [p.strip() for p in text.split(",") if p.strip()]
    try:
        values = [float(p) for p in parts]
    except ValueError:
        raise argparse.ArgumentTypeError(f"无法解析数值：{text!r}")
    if len(values) == 1:
        return np.full(6, values[0])
    if len(values) == 6:
        return np.array(values)
    raise argparse.ArgumentTypeError(f"需要 1 个或 6 个值，收到 {len(values)} 个：{text!r}")


def parse_args():
    parser = argparse.ArgumentParser(
        prog="waypoint_check",
        description="多路点轨迹仿真验收：限位、路点到达、平滑度（velocity/acceleration/jerk 峰值）。",
        epilog=(
            "示例：ros2 run tasks waypoint_check --mode joint "
            "--expected '[[0,0,0,0,0,0],[0.3,0.2,-0.2,0,0,0]]'"
        ),
    )
    parser.add_argument(
        "--mode",
        choices=["joint", "cart"],
        default="joint",
        help="joint：期望路点为 [[q1..q6], ...]；cart：[[x,y,z,qx,qy,qz,qw], ...]（默认 joint）",
    )
    parser.add_argument(
        "--expected",
        default=None,
        help="期望的用户路点列表（JSON）。joint 模式必需；cart 模式仅用于打印提示。",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=None,
        help="路点到达容差：joint 默认 0.01 rad，cart 默认 0.005 m（cart 模式当前不检查）",
    )
    parser.add_argument(
        "--vel-limit",
        type=parse_limit,
        default=parse_limit("1.0"),
        metavar="V",
        help="速度上限 rad/s，单值或逗号分隔 6 值（默认 1.0）",
    )
    parser.add_argument(
        "--acc-limit",
        type=parse_limit,
        default=parse_limit("2.0"),
        metavar="A",
        help="加速度上限 rad/s²，单值或逗号分隔 6 值（默认 2.0）",
    )
    parser.add_argument(
        "--jerk-limit",
        type=parse_limit,
        default=parse_limit("10.0"),
        metavar="J",
        help="jerk 上限 rad/s³，单值或逗号分隔 6 值（默认 10.0）",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=180.0,
        help="等待 arm_waypoints_duration 正值的超时（秒，默认 180）",
    )
    parser.add_argument(
        "--smooth-window",
        type=float,
        default=0.20,
        help=(
            "Savitzky-Golay 平滑窗口时长（秒，默认 0.20）；按采样间隔换算成奇数样本数。"
            "0.10 s 时实测仍有 ~25 rad/s³ 的跟踪谐波被算成 jerk，0.20 s 才稳定低于规划限值"
        ),
    )
    parser.add_argument(
        "--polyorder",
        type=int,
        default=3,
        help="Savitzky-Golay 多项式阶数（默认 3，必须小于窗口样本数）",
    )
    parser.add_argument(
        "--no-smoothing",
        action="store_true",
        help="关闭平滑，退回对位置的裸差分（会放大 500 Hz 跟踪纹波，仅供对照）",
    )
    parser.add_argument(
        "--margin",
        type=float,
        default=1.0,
        help="分析窗口在规划时长之后的余量（秒，默认 1.0）",
    )
    return parser.parse_known_args()


class WaypointChecker(Node):
    def __init__(self):
        super().__init__("waypoint_check")
        # 每项：(header_stamp, 到达时刻, position[6], velocity[6])
        self.samples = []
        self.collecting = False
        self.t0 = 0.0
        self.planned = 0.0
        self.rejected_value = None
        self.joint_state_sub = self.create_subscription(
            JointState, "joint_states", self.on_joint_state, qos_profile_sensor_data
        )
        self.duration_sub = self.create_subscription(
            Float64, "arm_waypoints_duration", self.on_duration, 10
        )

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def align(msg):
        """按名字对齐到 joint1..joint6；名字不全时退回消息顺序。"""
        indices = None
        if msg.name:
            try:
                indices = [msg.name.index(name) for name in JOINT_NAMES]
            except ValueError:
                indices = None
        if indices is not None:
            pos = [float(msg.position[i]) for i in indices]
            vel = [
                float(msg.velocity[i]) if i < len(msg.velocity) else 0.0
                for i in indices
            ]
            return pos, vel
        if len(msg.position) < 6:
            return None
        pos = [float(v) for v in msg.position[:6]]
        vel = [float(v) for v in msg.velocity[:6]] if len(msg.velocity) >= 6 else [0.0] * 6
        return pos, vel

    def on_joint_state(self, msg):
        aligned = self.align(msg)
        if aligned is None:
            return
        pos, vel = aligned
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.samples.append((stamp, self.now_sec(), pos, vel))
        if not self.collecting and len(self.samples) > 600000:
            del self.samples[:200000]

    def on_duration(self, msg):
        if self.planned > 0.0 or self.rejected_value is not None:
            return  # 已经有结论，忽略后续消息
        if msg.data > 0.0:
            self.planned = float(msg.data)
            self.t0 = self.now_sec()
            self.samples.clear()
            self.collecting = True
        else:
            self.rejected_value = float(msg.data)


def derivative(x, t):
    """按时间做一阶差分，返回 (d x/dt, 中点时刻)。"""
    dt = np.diff(t)
    return np.diff(x, axis=0) / dt[:, None], 0.5 * (t[1:] + t[:-1])


def build_time_base(samples):
    """优先用 header.stamp；时间戳重复/非递增时退回消息数 / 500 Hz。"""
    stamps = np.array([s[0] for s in samples], dtype=float)
    if len(stamps) >= 3 and np.all(np.diff(stamps) > 0.0):
        return stamps - stamps[0], False
    return np.arange(len(samples), dtype=float) / FALLBACK_RATE_HZ, True


def load_expected(raw, mode):
    if raw is None:
        return None
    try:
        data = json.loads(raw)
    except json.JSONDecodeError as exc:
        print(f"错误：--expected 不是合法 JSON：{exc}", file=sys.stderr)
        return None
    if not isinstance(data, list) or not data:
        print("错误：--expected 必须是非空列表", file=sys.stderr)
        return None
    expected = []
    for i, wp in enumerate(data):
        if not isinstance(wp, list) or len(wp) < (6 if mode == "joint" else 3):
            need = "6 个关节角" if mode == "joint" else "至少 3 个值（x,y,z）"
            print(f"错误：第 {i} 个路点格式不对，{mode} 模式需要 {need}", file=sys.stderr)
            return None
        expected.append([float(v) for v in wp])
    return expected


def check_joint_limits(q):
    """返回 [(joint_index, min, max, lo, hi), ...] 越界明细。"""
    violations = []
    for j, (lo, hi) in enumerate(JOINT_LIMITS):
        qmin = float(np.min(q[:, j]))
        qmax = float(np.max(q[:, j]))
        if qmin < lo - 1e-9 or qmax > hi + 1e-9:
            violations.append((j, qmin, qmax, lo, hi))
    return violations


def resample_uniform(q, t, dt):
    """把非等间隔的位置序列线性重采样到 dt 网格。

    Savitzky-Golay 要求等间隔输入；`/joint_states` 偶发丢拍（实测有 4~6 ms 的间隔）
    会让固定步长的三阶导数炸出几十倍虚假尖峰，所以先按时间戳插值对齐。
    返回 (q_uniform, 网格点数)；若已是等间隔则原样返回。
    """
    span = float(t[-1] - t[0])
    expected = 1 + int(round(span / dt))
    if len(t) == expected and np.allclose(np.diff(t), dt, atol=1e-6):
        return q, len(t)
    grid = t[0] + dt * np.arange(expected)
    uniform = np.column_stack([np.interp(grid, t, q[:, j]) for j in range(q.shape[1])])
    return uniform, expected


def smoothness_peaks(q, t, smooth_window, polyorder, enabled):
    """估计 velocity/acceleration/jerk 的逐关节峰值。

    默认用 Savitzky-Golay 对位置序列拟合局部多项式并取解析导数（等价于低通后再求导），
    这样 500 Hz 伺服跟踪纹波不会被二阶/三阶差分放大成数量级虚假的 acc/jerk。
    `enabled=False` 或 scipy 不可用时退回裸差分。

    返回 (peaks, 描述信息, 样本间隔 dt)。
    """
    dt = float(np.median(np.diff(t)))
    if dt <= 0.0:
        dt = 1.0 / FALLBACK_RATE_HZ

    use_smoothing = enabled and HAVE_SCIPY and len(t) >= 3
    if not use_smoothing:
        vel, t_v = derivative(q, t)
        acc, t_a = derivative(vel, t_v)
        jerk, _ = derivative(acc, t_a)
        peaks = {
            "vel": np.max(np.abs(vel), axis=0) if vel.size else np.zeros(6),
            "acc": np.max(np.abs(acc), axis=0) if acc.size else np.zeros(6),
            "jerk": np.max(np.abs(jerk), axis=0) if jerk.size else np.zeros(6),
        }
        reason = "关闭（裸差分）" if enabled and not HAVE_SCIPY else "关闭（裸差分，--no-smoothing）"
        return peaks, reason, dt

    q_uniform, n_samples = resample_uniform(q, t, dt)
    renote = "" if n_samples == len(t) else f"，非等间隔样本 {len(t)} 个已重采样到 {n_samples}"

    # 窗口时长 -> 样本数：奇数、至少 MIN_SMOOTH_SAMPLES、不超过样本数。
    win = int(round(smooth_window / dt))
    if win % 2 == 0:
        win += 1
    win = max(win, MIN_SMOOTH_SAMPLES)
    note = ""
    if win > n_samples:
        win = n_samples if n_samples % 2 == 1 else n_samples - 1
        note = f"（样本仅 {n_samples} 个，窗口已缩小到 {win}）"
        print(
            f"提示：Savitzky-Golay 窗口 {smooth_window:.3f}s 折合样本数超过 {n_samples}，"
            f"已缩小到 {win} 个样本",
            file=sys.stderr,
        )
    order = polyorder
    if order >= win:
        order = max(1, win - 1)
        print(
            f"提示：--polyorder={polyorder} 不小于窗口样本数 {win}，已降为 {order}",
            file=sys.stderr,
        )

    vel = savgol_filter(q_uniform, win, order, deriv=1, delta=dt, axis=0, mode="interp")
    acc = savgol_filter(q_uniform, win, order, deriv=2, delta=dt, axis=0, mode="interp")
    jerk = savgol_filter(q_uniform, win, order, deriv=3, delta=dt, axis=0, mode="interp")
    peaks = {
        "vel": np.max(np.abs(vel), axis=0),
        "acc": np.max(np.abs(acc), axis=0),
        "jerk": np.max(np.abs(jerk), axis=0),
    }
    reason = (
        f"Savitzky-Golay window={smooth_window:.3f}s polyorder={order}"
        f"（{win} 样本 @ dt={dt * 1e3:.3f} ms，deriv=1/2/3，mode=interp）{renote}{note}"
    )
    return peaks, reason, dt


def print_joint_vector(label, values, fmt):
    text = "  ".join(f"j{j + 1}={fmt.format(v)}" for j, v in enumerate(values))
    print(f"  {label}: {text}")


def main():
    args, ros_args = parse_args()
    expected = load_expected(args.expected, args.mode)
    if args.expected is not None and expected is None:
        return 1
    if args.mode == "joint" and expected is None:
        print("错误：joint 模式必须用 --expected 提供期望路点", file=sys.stderr)
        return 1
    tolerance = args.tolerance
    if tolerance is None:
        tolerance = 0.01 if args.mode == "joint" else 0.005
    if not args.no_smoothing and not HAVE_SCIPY:
        print(
            "警告：未能导入 scipy.signal.savgol_filter，--smooth-window/--polyorder 无效，"
            "已退回裸差分（acc/jerk 会被 500 Hz 跟踪纹波放大）",
            file=sys.stderr,
        )

    rclpy.init(args=ros_args)
    node = WaypointChecker()
    failure = None
    try:
        if args.mode == "cart" and expected:
            endpoint = expected[-1][:3]
            print(
                "cart 模式：脚本内不做 FK，路点到达由任务执行器的终点验收负责；"
                f"期望终点 xyz = [{endpoint[0]:.4f}, {endpoint[1]:.4f}, {endpoint[2]:.4f}]"
            )
        print(
            f"等待 arm_waypoints_duration 正值（mode={args.mode}, timeout={args.timeout:.1f}s）...",
            flush=True,
        )
        start = node.now_sec()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.rejected_value is not None:
                failure = (
                    "控制器拒绝了规划（arm_waypoints_duration = "
                    f"{node.rejected_value:.3f} <= 0），轨迹未执行"
                )
                break
            now = node.now_sec()
            if node.planned > 0.0:
                if now >= node.t0 + node.planned + args.margin:
                    break
            elif now - start >= args.timeout:
                failure = (
                    f"超时未收到规划时长（等待 {args.timeout:.1f}s 仍无正的 "
                    "arm_waypoints_duration）；确认控制器已收到多路点轨迹"
                )
                break
    except KeyboardInterrupt:
        failure = "用户中断"
    finally:
        if failure is not None:
            print(f"FAIL：{failure}", file=sys.stderr)
            node.destroy_node()
            rclpy.shutdown()
            return 1

        samples = node.samples
        planned = node.planned
        node.destroy_node()
        rclpy.shutdown()

    if len(samples) < 10:
        print(f"FAIL：记录样本不足（{len(samples)} 个），无法分析", file=sys.stderr)
        return 1

    q = np.array([s[2] for s in samples], dtype=float)
    vel_ref = np.array([s[3] for s in samples], dtype=float)
    t, fallback = build_time_base(samples)
    window = float(t[-1] - t[0])

    print(
        f"\n样本数：{len(samples)}，规划时长：{planned:.3f}s，"
        f"分析窗口：{window:.3f}s（{t[0]:.3f} ~ {t[-1]:.3f}s）"
    )
    if fallback:
        print(
            f"警告：/joint_states 的 header.stamp 重复或非递增，已退回消息数 / "
            f"{FALLBACK_RATE_HZ:.0f} Hz 估算时间"
        )

    ok = True

    # 1) 关节限位
    violations = check_joint_limits(q)
    print("\n[限位]")
    if violations:
        ok = False
        for j, qmin, qmax, lo, hi in violations:
            print(
                f"  FAIL joint{j + 1}: 记录范围 [{qmin:.5f}, {qmax:.5f}] "
                f"超出 URDF 限位 [{lo:.5f}, {hi:.5f}]"
            )
    else:
        print("  PASS 所有关节位置均在 URDF 限位内")

    # 2) 路点到达
    if args.mode == "joint":
        print("\n[路点到达]")
        for i, wp in enumerate(expected):
            target = np.array(wp[:6], dtype=float)
            dist = float(np.min(np.max(np.abs(q - target), axis=1)))
            passed = dist <= tolerance
            ok = ok and passed
            print(
                f"  路点 {i}: 最小 max-abs 距离 {dist:.6f} rad "
                f"(tolerance {tolerance:.6f}) -> {'PASS' if passed else 'FAIL'}"
            )
    else:
        print("\n[路点到达]")
        print("  SKIP cart 模式无法在脚本内做 FK，请以任务执行器的终点验收为准")

    # 3) 平滑度
    peaks, smooth_desc, _ = smoothness_peaks(
        q, t, args.smooth_window, args.polyorder, not args.no_smoothing
    )
    limits = {"vel": args.vel_limit, "acc": args.acc_limit, "jerk": args.jerk_limit}
    names = {"vel": "速度 (rad/s)", "acc": "加速度 (rad/s²)", "jerk": "jerk (rad/s³)"}
    print(f"\n平滑：{smooth_desc}")
    print("[平滑度]（阈值 = limit * 1.10 + 噪声下限）")
    for key in ("vel", "acc", "jerk"):
        thresholds = limits[key] * 1.10 + NOISE_FLOOR[key]
        over = peaks[key] > thresholds
        order_ok = not bool(np.any(over))
        ok = ok and order_ok
        print(f"  {names[key]}: {'PASS' if order_ok else 'FAIL'}")
        print_joint_vector("峰值", peaks[key], "{:.4f}")
        print_joint_vector("阈值", thresholds, "{:.4f}")
        if not order_ok:
            over_names = ", ".join(f"joint{j + 1}" for j in range(6) if over[j])
            print(f"  超限关节: {over_names}")

    ref_peak = np.max(np.abs(vel_ref), axis=0) if vel_ref.size else np.zeros(6)
    print("\n[参考] /joint_states 自带 velocity 的峰值（不参与判定）")
    print_joint_vector("峰值", ref_peak, "{:.4f}")

    print(f"\n结果：{'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
