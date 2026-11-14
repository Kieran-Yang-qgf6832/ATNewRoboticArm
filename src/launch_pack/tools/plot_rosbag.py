#!/usr/bin/env python3
"""把 rosbag 里录到的关节命令与关节状态画成逐关节对比图。

数据来源（由 launch_pack/launch/arm_bag_record.launch.py 录制）：

  /joint_states                 反馈：下位机（实机 MCU / 仿真 MuJoCo）回报的
                                position / velocity / effort
  /arm_controller/joint_command 命令：控制器每周期真正写进命令接口的
                                position / velocity / effort（前馈力矩）
  /arm_controller/state         控制器实际状态（std_msgs/String，**没有 header**）

输出：每个关节一张 PNG，4 个子图 —— 位置 / 速度 / 力矩（命令 vs 反馈叠加）+ 位置误差；
默认写到 <项目根>/docs/rosbags/<bag 目录名>/joint1.png …（重跑直接覆盖）。

用法：
    # 画 .ros/ros2bags 下最新的 bag
    ros2 run launch_pack plot_rosbag
    # 指定 bag
    ros2 run launch_pack plot_rosbag .ros/ros2bags/arm_sim_20261114-143345
    # 换输出目录 / 提高分辨率
    ros2 run launch_pack plot_rosbag <bag> --out-dir /tmp/plots --dpi 180
    # 只看命令、不画反馈（位置误差子图留空；同名文件会被覆盖）
    ros2 run launch_pack plot_rosbag <bag> --no-feedback

时间轴与对齐：
- x 轴用消息的 header.stamp（命令与状态同一时钟；仿真里是仿真时间），首帧反馈为 0；
- bag 的接收时间戳是墙钟（录制器刻意不开仿真时间，见设计约束 C9），只在把
  /arm_controller/state（无 header）对到数据时钟时用到：借 /joint_states 的
  「接收时间 → header 时间」线性映射换算，实机下两者近似相等；
- 命令按 header.stamp 线性插值到反馈的时间网格上，误差 = 命令 − 反馈；命令时间范围之外
  不做外推（误差留空）。

口径说明（不要误读）：
- **不画加速度**：JointState 没有该字段，实机 MCU 也不上报，差分出来的加速度只能看趋势，
  所以直接不画（需要时回 docs/execute/02 看 bag 里到底有什么）。
- 命令 effort 是前馈力矩，反馈 effort 是执行器实际出力（仿真 qfrc_actuator / 实机 MCU 上报），
  两者不是「目标 vs 实测」的关系，不能当误差看。
- 缺 /arm_controller/joint_command 的旧 bag 自动降级为只画反馈，并在标题与终端标注。
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml

import matplotlib

matplotlib.use("Agg")  # 无 X 的机器上也能出图；必须在 import pyplot 之前设置
import matplotlib.pyplot as plt  # noqa: E402

import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

TOPIC_FEEDBACK = "/joint_states"
TOPIC_COMMAND = "/arm_controller/joint_command"
TOPIC_STATE = "/arm_controller/state"

# 中文字体候选：按优先级探测，找不到就退回英文标签（绝不画成方块）。
CJK_FONT_CANDIDATES = (
    "Noto Sans CJK SC",
    "Noto Serif CJK SC",
    "Source Han Sans SC",
    "WenQuanYi Zen Hei",
    "AR PL UMing CN",
)

# 状态带的配色循环（浅色填充 + alpha 在 _shade_states 里控制）。
STATE_COLORS = ("tab:blue", "tab:green", "tab:purple", "tab:brown", "tab:cyan", "tab:gray")


# --------------------------------------------------------------------------------------
# 路径与字体
# --------------------------------------------------------------------------------------
def _project_root() -> Path:
    """向上找项目根：同时存在 src/launch_pack 与 install/ 的目录。

    判据与 launch/arm_bag_record.launch.py 一致；从 install/launch_pack/lib/launch_pack
    出发也能落回项目根，找不到时退回当前工作目录。
    """
    current = Path(__file__).resolve().parent
    for _ in range(8):
        if (current / "src" / "launch_pack").is_dir() and (current / "install").is_dir():
            return current
        if current.parent == current:
            break
        current = current.parent
    return Path.cwd()


def _latest_bag(bags_root: Path) -> Path:
    """取 bags_root 下 metadata.yaml 最新的子目录（bag 收尾时该文件最后写，mtime 最可靠）。"""
    if not bags_root.is_dir():
        raise SystemExit(f"找不到 bag 目录 {bags_root}；先启动一次控制器录一段 bag，或显式给出 bag 路径。")
    candidates = [p for p in bags_root.iterdir() if p.is_dir() and (p / "metadata.yaml").is_file()]
    if not candidates:
        raise SystemExit(f"{bags_root} 下没有 bag（缺 metadata.yaml）；请显式给出 bag 路径。")
    return max(candidates, key=lambda p: (p / "metadata.yaml").stat().st_mtime)


def _setup_chinese_font() -> bool:
    """尝试启用中文字体；返回是否成功（失败时调用方改用英文标签）。"""
    from matplotlib import font_manager

    available = {font.name for font in font_manager.fontManager.ttflist}
    for name in CJK_FONT_CANDIDATES:
        if name in available:
            plt.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            plt.rcParams["axes.unicode_minus"] = False  # 中文字体常缺 U+2212
            return True
    return False


def _texts(chinese: bool) -> dict[str, str]:
    if chinese:
        return {
            "cmd": "命令",
            "fb": "反馈",
            "pos": "位置 (rad)",
            "vel": "速度 (rad/s)",
            "eff": "力矩 (N·m)",
            "err": "位置误差 (rad)",
            "time": "时间 (s)（首帧反馈 = 0）",
            "title": "{bag} · {joint}：命令 vs 反馈",
            "missing": "（缺命令数据，仅反馈）",
            "only_cmd": "（仅命令）",
            "no_fb": "（已关闭反馈绘制）",
            "note": "命令 effort = 前馈力矩；反馈 effort = 执行器实际出力（仿真 qfrc_actuator / 实机 MCU 上报）。"
            "状态带取自 /arm_controller/state（控制器 500 ms 采样，边界 ±0.5 s，首条之前留空）。",
        }
    return {
        "cmd": "command",
        "fb": "feedback",
        "pos": "position (rad)",
        "vel": "velocity (rad/s)",
        "eff": "effort (N·m)",
        "err": "position error (rad)",
        "time": "time (s), first feedback sample = 0",
        "title": "{bag} · {joint}: command vs feedback",
        "missing": " (command topic missing, feedback only)",
        "only_cmd": " (command only)",
        "no_fb": " (feedback plotting disabled)",
        "note": "command effort = feedforward torque; feedback effort = actuator output (sim qfrc_actuator / MCU report). "
        "State bands come from /arm_controller/state (500 ms sampling, edge accuracy +-0.5 s; blank before the first).",
    }


# --------------------------------------------------------------------------------------
# 读 bag
# --------------------------------------------------------------------------------------
def _storage_id(bag: Path) -> str:
    metadata = bag / "metadata.yaml"
    if not metadata.is_file():
        raise SystemExit(f"{bag} 不是 rosbag2 目录（缺 metadata.yaml）")
    info = (yaml.safe_load(metadata.read_text(encoding="utf-8")) or {}).get("rosbag2_bagfile_information", {})
    return info.get("storage_identifier") or "mcap"


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _aligned(values, names, order) -> list[float]:
    """按主关节顺序重排一帧数据；关节顺序不一致时按名字映射（缺的填 NaN）。"""
    if list(names) == order:
        return list(values)
    table = {name: index for index, name in enumerate(names)}
    return [values[table[name]] if name in table else math.nan for name in order]


def _sorted_unique(t: np.ndarray, *arrays: np.ndarray):
    """按时间排序并去掉重复时刻（保留最后一条），保证 np.interp 的输入严格递增。"""
    order = np.argsort(t, kind="stable")
    t = t[order]
    arrays = tuple(array[order] for array in arrays)
    keep = np.concatenate(([True], np.diff(t) > 0))
    return (t,) + tuple(array[keep] for array in arrays)


def _read_bag(bag: Path) -> dict:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=_storage_id(bag)),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}

    order: list[str] = []
    fb_time_bag, fb_time_hdr, fb_pos, fb_vel, fb_eff = [], [], [], [], []
    cmd_time_hdr, cmd_pos, cmd_vel, cmd_eff = [], [], [], []
    state_time_bag, state_name = [], []

    while reader.has_next():
        topic, raw, recv_ns = reader.read_next()

        if topic == TOPIC_FEEDBACK:
            msg = deserialize_message(raw, get_message(types[topic]))
            if not order:
                order = list(msg.name)
            fb_time_bag.append(recv_ns)
            fb_time_hdr.append(_stamp_ns(msg.header.stamp))
            fb_pos.append(_aligned(msg.position, msg.name, order))
            fb_vel.append(_aligned(msg.velocity, msg.name, order))
            fb_eff.append(_aligned(msg.effort, msg.name, order))

        elif topic == TOPIC_COMMAND:
            msg = deserialize_message(raw, get_message(types[topic]))
            if not order:
                order = list(msg.name)
            cmd_time_hdr.append(_stamp_ns(msg.header.stamp))
            cmd_pos.append(_aligned(msg.position, msg.name, order))
            cmd_vel.append(_aligned(msg.velocity, msg.name, order))
            cmd_eff.append(_aligned(msg.effort, msg.name, order))

        elif topic == TOPIC_STATE:
            msg = deserialize_message(raw, get_message(types[topic]))
            state_time_bag.append(recv_ns)
            state_name.append(msg.data)

    try:
        reader.close()
    except AttributeError:  # 老版本绑定没有 close()
        pass

    if not order or not fb_time_bag:
        raise SystemExit(f"{bag} 里没有 {TOPIC_FEEDBACK}；本工具至少要反馈数据才能画图。")

    # 反馈按 bag 接收时间排序（= 录制顺序，header 时间同步递增），并去掉重复时刻。
    fb_time_bag, fb_time_hdr, fb_pos, fb_vel, fb_eff = _sorted_unique(
        np.asarray(fb_time_bag, dtype=np.int64),
        np.asarray(fb_time_hdr, dtype=np.int64),
        np.asarray(fb_pos, dtype=float),
        np.asarray(fb_vel, dtype=float),
        np.asarray(fb_eff, dtype=float),
    )
    feedback = {
        "time_bag": fb_time_bag,
        "time": fb_time_hdr,  # ns，header.stamp
        "pos": fb_pos,
        "vel": fb_vel,
        "eff": fb_eff,
    }

    command = None
    if cmd_time_hdr:
        # 命令按 header 时间排序：后面要按它做 np.interp（要求严格递增）。
        cmd_time_hdr, cmd_pos, cmd_vel, cmd_eff = _sorted_unique(
            np.asarray(cmd_time_hdr, dtype=np.int64),
            np.asarray(cmd_pos, dtype=float),
            np.asarray(cmd_vel, dtype=float),
            np.asarray(cmd_eff, dtype=float),
        )
        command = {"time": cmd_time_hdr, "pos": cmd_pos, "vel": cmd_vel, "eff": cmd_eff}

    states = {"time_bag": np.asarray(state_time_bag, dtype=np.int64), "name": state_name}
    return {"order": order, "feedback": feedback, "command": command, "states": states}


# --------------------------------------------------------------------------------------
# 时间对齐
# --------------------------------------------------------------------------------------
def _interp_to(t_src: np.ndarray, values: np.ndarray, t_dst: np.ndarray) -> np.ndarray:
    """按列线性插值到 t_dst；超出源时间范围的位置填 NaN（不外推）。"""
    out = np.full((t_dst.size, values.shape[1]), np.nan)
    inside = (t_dst >= t_src[0]) & (t_dst <= t_src[-1])
    for column in range(values.shape[1]):
        out[inside, column] = np.interp(t_dst[inside], t_src, values[:, column])
    return out


def _state_segments(data: dict, t0: float, t1: float) -> list[tuple[float, float, str, str]]:
    """/arm_controller/state（无 header）→ [(start, end, 状态名, 配色)]。

    状态消息只有 bag 接收时间（墙钟），用 /joint_states 的「接收时间 → header 时间」
    线性映射换算到数据时钟。末段一直延伸到最后一帧反馈；首条状态之前不补（那段的状态
    无法从 bag 得知）。
    """
    feedback = data["feedback"]
    raw_time = data["states"]["time_bag"]
    names = data["states"]["name"]
    if raw_time.size == 0 or names == []:
        return []

    # 反馈的接收时间可能重复，先排序去重再当映射的 x。
    bag_t, hdr_t = _sorted_unique(feedback["time_bag"], feedback["time"].astype(float))
    mapped = np.interp(raw_time.astype(float), bag_t.astype(float), hdr_t)
    mapped = mapped / 1e9  # ns → s

    # 同一状态连续出现多条时只保留第一条，避免图上出现零宽段。
    starts, labels = [], []
    for time_s, name in zip(mapped, names):
        if labels and labels[-1] == name:
            continue
        starts.append(float(time_s))
        labels.append(name)

    # 同一状态名固定用同一种颜色，避免 reset 在不同段换色造成误读。
    color_of: dict[str, str] = {}
    segments = []
    for index, (start, name) in enumerate(zip(starts, labels)):
        end = starts[index + 1] if index + 1 < len(starts) else t1
        if name not in color_of:
            color_of[name] = STATE_COLORS[len(color_of) % len(STATE_COLORS)]
        segments.append((max(start, t0), min(end, t1), name, color_of[name]))
    # 刻意不把首段向前补到 t0：首条状态早于 t0 时它在 t0 就开始，但我们无法从 bag 得知
    # 首条状态之前是什么状态（录制器启动早于控制器激活时，那段是「未知」），留空最诚实。
    return [segment for segment in segments if segment[1] > segment[0]]


# --------------------------------------------------------------------------------------
# 绘图
# --------------------------------------------------------------------------------------
def _shade_states(axes, segments) -> None:
    """在四个子图上画状态背景带，并在最上面的子图里写状态名。"""
    if not segments:
        return
    span = segments[-1][1] - segments[0][0]
    for start, end, name, color in segments:
        for axis in axes:
            axis.axvspan(start, end, color=color, alpha=0.08, linewidth=0, zorder=0)
        if span > 0 and (end - start) / span >= 0.02:  # 太窄就不写字，免得糊成一团
            axes[0].text(
                (start + end) / 2.0,
                0.97,
                name,
                transform=axes[0].get_xaxis_transform(),
                ha="center",
                va="top",
                fontsize=8,
                color="0.25",
            )


def _plot_joint(
    path: Path,
    bag_name: str,
    joint: str,
    curves: dict,
    segments,
    texts,
    dpi: int,
    show_feedback: bool = True,
) -> None:
    fig, axes = plt.subplots(4, 1, figsize=(12.0, 11.0), sharex=True)
    t = curves["t"]

    def overlay(axis, command, feedback, ylabel):
        if command is not None:
            axis.plot(curves["t_cmd"], command, color="tab:blue", linewidth=1.0, label=texts["cmd"])
        if show_feedback:
            axis.plot(t, feedback, color="tab:orange", linewidth=1.0, label=texts["fb"])
        axis.set_ylabel(ylabel)
        axis.grid(alpha=0.3)
        if command is not None or show_feedback:
            axis.legend(loc="best", fontsize=9)

    overlay(axes[0], curves["cmd_pos"], curves["fb_pos"], texts["pos"])
    overlay(axes[1], curves["cmd_vel"], curves["fb_vel"], texts["vel"])
    overlay(axes[2], curves["cmd_eff"], curves["fb_eff"], texts["eff"])

    if not show_feedback:
        # 误差 = 命令 − 反馈，反馈被关掉时这一格没有意义，用文字交代而不是留空白。
        placeholder = texts["no_fb"]
    elif curves["error"] is None:
        placeholder = texts["missing"]
    else:
        placeholder = None

    if placeholder is None:
        axes[3].plot(t, curves["error"], color="tab:red", linewidth=0.9)
        axes[3].axhline(0.0, color="0.35", linewidth=0.8, linestyle="--")
    else:
        axes[3].text(
            0.5, 0.5, placeholder, transform=axes[3].transAxes, ha="center", va="center", color="0.5"
        )
        axes[3].set_yticks([])  # 没有误差数据时不要留一排 0.0~1.0 的空刻度
    axes[3].set_ylabel(texts["err"])
    axes[3].grid(alpha=0.3)
    axes[3].set_xlabel(texts["time"])

    _shade_states(axes, segments)

    title = texts["title"].format(bag=bag_name, joint=joint)
    if curves["cmd_pos"] is None:
        title += texts["missing"]
    elif not show_feedback:
        title += texts["only_cmd"]
    fig.suptitle(title, fontsize=13)
    fig.text(0.01, 0.006, texts["note"], fontsize=7.5, color="0.4")
    fig.tight_layout(rect=(0.0, 0.022, 1.0, 0.97))
    fig.savefig(path, dpi=dpi)
    plt.close(fig)


def _build_curves(data: dict, index: int, compute_error: bool = True) -> dict:
    """取第 index 个关节的命令/反馈序列；compute_error 时在反馈时间网格上算位置误差。"""
    feedback = data["feedback"]
    t0_ns = float(feedback["time"][0])
    t = feedback["time"].astype(float) / 1e9 - t0_ns / 1e9

    curves = {
        "t": t,
        "t_cmd": None,
        "cmd_pos": None,
        "cmd_vel": None,
        "cmd_eff": None,
        "fb_pos": feedback["pos"][:, index],
        "fb_vel": feedback["vel"][:, index],
        "fb_eff": feedback["eff"][:, index],
        "error": None,
    }

    command = data["command"]
    if command is not None:
        curves["t_cmd"] = command["time"].astype(float) / 1e9 - t0_ns / 1e9
        curves["cmd_pos"] = command["pos"][:, index]
        curves["cmd_vel"] = command["vel"][:, index]
        curves["cmd_eff"] = command["eff"][:, index]
        if compute_error:
            # 误差在反馈的时间网格上算：命令按 header.stamp 插值过来，范围外留空（不外推）。
            cmd_pos_on_fb = _interp_to(command["time"].astype(float), command["pos"], feedback["time"].astype(float))
            curves["error"] = cmd_pos_on_fb[:, index] - curves["fb_pos"]

    return curves


# --------------------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------------------
def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="把 rosbag 里的关节命令与关节状态画成逐关节对比图（位置/速度/力矩 + 位置误差）。",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("bag", nargs="?", help="rosbag2 目录；省略时取 <项目根>/.ros/ros2bags 下最新的一个")
    parser.add_argument("--out-dir", default=None, help="输出根目录；默认 <项目根>/docs/rosbags")
    parser.add_argument("--dpi", type=int, default=140, help="PNG 分辨率")
    parser.add_argument(
        "--no-feedback",
        dest="show_feedback",
        action="store_false",
        # 不写 default：让 ArgumentDefaultsHelpFormatter 不要打出容易误解的 "default: True"，
        # 缺省值在下面用 getattr 取（默认画反馈）。
        default=argparse.SUPPRESS,
        help="不画反馈曲线（只画命令；默认画反馈）；位置误差子图相应留空，时间轴与关节集合仍取 /joint_states",
    )
    args = parser.parse_args(argv)
    show_feedback = getattr(args, "show_feedback", True)

    project_root = _project_root()
    bag = Path(args.bag).expanduser().resolve() if args.bag else _latest_bag(project_root / ".ros" / "ros2bags")
    if not bag.is_dir():
        raise SystemExit(f"bag 目录不存在：{bag}")
    out_root = Path(args.out_dir).expanduser().resolve() if args.out_dir else project_root / "docs" / "rosbags"
    out_dir = out_root / bag.name
    out_dir.mkdir(parents=True, exist_ok=True)

    data = _read_bag(bag)
    has_command = data["command"] is not None
    chinese = _setup_chinese_font()
    texts = _texts(chinese)
    if not chinese:
        print("[plot_rosbag] 未找到中文字体（Noto Sans/Serif CJK SC 等），图内改用英文标签。", file=sys.stderr)

    feedback_count = data["feedback"]["pos"].shape[0]
    command_count = data["command"]["pos"].shape[0] if has_command else 0
    state_count = data["states"]["time_bag"].size
    print(f"[plot_rosbag] bag: {bag}")
    print(f"[plot_rosbag] 消息数：{TOPIC_FEEDBACK} {feedback_count}，{TOPIC_COMMAND} {command_count}，{TOPIC_STATE} {state_count}")
    if not has_command:
        print(
            f"[plot_rosbag] 警告：bag 里没有 {TOPIC_COMMAND}（旧版录制），"
            "只画反馈曲线，位置误差与命令线留空。",
            file=sys.stderr,
        )

    if not show_feedback:
        print("[plot_rosbag] --no-feedback：只画命令曲线，位置误差子图留空（同名文件会被覆盖）。")

    # 状态段只算一次（所有关节共用），t0/t1 取反馈时间轴。
    t_fb = data["feedback"]["time"].astype(float) / 1e9
    segments = _state_segments(data, 0.0, float(t_fb[-1] - t_fb[0]))

    written = []
    for index, joint in enumerate(data["order"]):
        curves = _build_curves(data, index, compute_error=show_feedback)
        filename = "".join(ch if (ch.isalnum() or ch in "-_") else "_" for ch in joint) + ".png"
        path = out_dir / filename
        _plot_joint(path, bag.name, joint, curves, segments, texts, args.dpi, show_feedback=show_feedback)
        written.append(path)

    print(f"[plot_rosbag] {len(written)} 张图 → {out_dir}")
    for path in written:
        print(f"  {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
