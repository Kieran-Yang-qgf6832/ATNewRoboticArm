#!/usr/bin/env python3
"""统计「末端惯性空间保持」的误差，出 CSV/PNG，并按暂定门槛给出 PASS/FAIL。

误差由控制器在 world 系里直接算好后发在 ~/inertial_hold_status 上（见
robot_msgs/msg/InertialHoldStatus），这里只做记录与统计，不重新算坐标——
坐标链一旦在外部重算错了，会得到一份看起来很正常但其实无效的结论。

评估窗口：从**基座开始运动**的那一帧算起（订阅 base_motion_state 的 motion_active），
到 --duration 秒为止。基座静止阶段不计入统计，否则 RMS 会被静态段稀释。

用法：
  ros2 run arm_base_motion evaluate_inertial_hold.py --duration 30 --output-dir /tmp/hold_eval
  # 门槛（暂定，首轮实测后可修订）：
  #   位置峰值 <= 5 mm、位置 RMS <= 2 mm、姿态峰值 <= 0.5 deg
"""

import argparse
import csv
import math
import os
import sys

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from robot_msgs.msg import BaseMotionState, InertialHoldStatus


def stamp_to_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class Evaluator(Node):
    def __init__(self, args):
        super().__init__("inertial_hold_evaluator")
        self.args = args
        self.samples = []
        self.window_start_ns = None
        self.done = False
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self.status_sub = self.create_subscription(InertialHoldStatus, args.topic, self.on_status, 10)
        self.base_sub = self.create_subscription(BaseMotionState, args.base_topic, self.on_base_state, 5)

    def on_base_state(self, message):
        if self.window_start_ns is None and message.motion_active:
            self.window_start_ns = stamp_to_ns(message.header.stamp)
            self.get_logger().info("evaluation window started (base motion active)")

    def on_status(self, message):
        stamp_ns = stamp_to_ns(message.header.stamp)
        if self.window_start_ns is not None and stamp_ns < self.window_start_ns:
            return
        self.samples.append(
            (
                stamp_ns,
                message.position_error,
                message.orientation_error,
                bool(message.base_state_valid),
                bool(message.ik_converged),
            )
        )
        if (
            self.window_start_ns is not None
            and self.args.duration > 0.0
            and (stamp_ns - self.window_start_ns) * 1e-9 >= self.args.duration
        ):
            self.done = True

    def summarize(self):
        windowed = [s for s in self.samples if self.window_start_ns is not None and s[0] >= self.window_start_ns]
        if not windowed:
            print("没有可评估的样本：基座是否真的开始运动？检查 base_motion_state 的 motion_active。")
            return 1

        position = [s[1] for s in windowed]
        orientation = [s[2] for s in windowed]
        valid_ratio = sum(1 for s in windowed if s[3]) / len(windowed)
        ik_ratio = sum(1 for s in windowed if s[4]) / len(windowed)

        def rms(values):
            return math.sqrt(sum(v * v for v in values) / len(values))

        pos_rms = rms(position)
        pos_peak = max(position)
        ori_rms_deg = math.degrees(rms(orientation))
        ori_peak_deg = math.degrees(max(orientation))

        print("samples            : %d" % len(windowed))
        print("position   RMS     : %.4f mm" % (pos_rms * 1e3))
        print("position   peak    : %.4f mm" % (pos_peak * 1e3))
        print("orientation RMS    : %.4f deg" % ori_rms_deg)
        print("orientation peak   : %.4f deg" % ori_peak_deg)
        print("base_state valid   : %.1f %%" % (valid_ratio * 100.0))
        print("ik converged       : %.1f %%" % (ik_ratio * 100.0))

        position_ok = pos_peak <= self.args.position_peak and pos_rms <= self.args.position_rms
        orientation_ok = ori_peak_deg <= self.args.orientation_peak_deg
        print(
            "thresholds         : position peak <= %.1f mm, position RMS <= %.1f mm, orientation peak <= %.2f deg"
            % (
                self.args.position_peak * 1e3,
                self.args.position_rms * 1e3,
                self.args.orientation_peak_deg,
            )
        )
        verdict = position_ok and orientation_ok and valid_ratio > 0.99 and ik_ratio > 0.99
        print("verdict            : %s" % ("PASS" if verdict else "FAIL"))

        os.makedirs(self.args.output_dir, exist_ok=True)
        csv_path = os.path.join(self.args.output_dir, "inertial_hold_error.csv")
        with open(csv_path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["stamp_ns", "position_error_m", "orientation_error_rad", "base_state_valid", "ik_converged"])
            writer.writerows(windowed)
        print("csv                : " + csv_path)

        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            seconds = [(s[0] - windowed[0][0]) * 1e-9 for s in windowed]
            figure, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
            axes[0].plot(seconds, [v * 1e3 for v in position])
            axes[0].axhline(self.args.position_peak * 1e3, color="r", linestyle="--", linewidth=0.8)
            axes[0].set_ylabel("position error [mm]")
            axes[0].grid(True)
            axes[1].plot(seconds, [math.degrees(v) for v in orientation], color="tab:orange")
            axes[1].axhline(self.args.orientation_peak_deg, color="r", linestyle="--", linewidth=0.8)
            axes[1].set_ylabel("orientation error [deg]")
            axes[1].set_xlabel("time since base motion start [s]")
            axes[1].grid(True)
            figure.suptitle("inertial hold: end-effector error in world frame")
            png_path = os.path.join(self.args.output_dir, "inertial_hold_error.png")
            figure.tight_layout()
            figure.savefig(png_path, dpi=140)
            print("png                : " + png_path)
        except ImportError:
            print("png                : skipped (matplotlib not installed)")

        return 0 if verdict else 1


def main():
    parser = argparse.ArgumentParser(description="Evaluate the inertial-hold error topic")
    parser.add_argument("--topic", default="/arm_controller/inertial_hold_status")
    parser.add_argument("--base-topic", default="/base_motion_state")
    parser.add_argument("--duration", type=float, default=30.0, help="seconds of base motion to evaluate")
    parser.add_argument("--output-dir", default="inertial_hold_eval")
    parser.add_argument("--position-peak", type=float, default=0.005, help="m")
    parser.add_argument("--position-rms", type=float, default=0.002, help="m")
    parser.add_argument("--orientation-peak-deg", type=float, default=0.5, help="deg")
    args = parser.parse_args()

    rclpy.init()
    evaluator = Evaluator(args)
    executor = SingleThreadedExecutor()
    executor.add_node(evaluator)
    try:
        while rclpy.ok() and not evaluator.done:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    exit_code = evaluator.summarize()
    executor.remove_node(evaluator)
    evaluator.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
