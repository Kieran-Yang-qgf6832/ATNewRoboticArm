"""
@file arm_bag_record.launch.py
@brief 用 rosbag2 录制「上位机下发的关节命令」与「下位机回报的关节状态」（实机/仿真通用）

录制话题：
  /joint_states                 joint_state_broadcaster 输出：下位机（MCU / MuJoCo）回报的
                                关节 position / velocity / effort
  /arm_controller/joint_command ArmController 每周期**真正写进命令接口**的关节命令
                                （sensor_msgs/JointState，position/velocity/effort = 命令值）
  /arm_controller/state         控制器实际所处状态（idel / reset / joint_traj …），解释数据用
  /arm_joint_traj               任务层下发的关节轨迹指令（意图，非每周期命令）
  /arm_joint_waypoints          任务层下发的关节多路点指令（意图）
  /arm_servo                    任务层 / 控制台下发的伺服指令（意图）

输出目录：`<bag_dir>/<bag_name>_<YYYYmmdd-HHMMSS>/`（mcap）。
`bag_dir` 默认取 **项目根的 `.ros/ros2bags`**（`.ros` 已在 .gitignore 里），
即 `~/MyProjects/2027ATRobotArm_LowerAndUpper/ATNewRoboticArm_Upper/.ros/ros2bags/arm_sim_<时间戳>`。

用法（一般不需要直接启动，由 arm_real.launch.py 与 arm_controller_test_sim.launch.py 拉起）：
  ros2 launch launch_pack arm_bag_record.launch.py
  ros2 launch launch_pack arm_bag_record.launch.py record_bag:=false
  ros2 launch launch_pack arm_bag_record.launch.py bag_dir:=/data/bags bag_name:=mytest

参数：
  record_bag  := true   是否录制（false = 本文件不启动任何进程）
  bag_dir     := ""     输出根目录；空 = <项目根>/.ros/ros2bags
  bag_name    := arm    bag 名前缀，实机入口传 arm_real、仿真入口传 arm_sim
  bag_storage := mcap   存储插件（mcap / sqlite3）

@note 刻意不传 --use-sim-time：仿真里 /clock 是 500 Hz，给 Python 节点开仿真时间会吃掉一个核
      （设计约束 C9）。所以 bag 的接收时间戳是墙钟，**对齐请用消息的 header.stamp**——
      /joint_states 与 /arm_controller/joint_command 都取自控制周期，是同一个时钟。
@note 录制器在 launch 起始就启动。rosbag2 对指定话题是「先订阅、等数据」，话题暂时不存在
      不会报错，因此控制器激活后的自动复位运动也能录到。
@note 用 Ctrl-C 结束 launch 时 rosbag2 会收到 SIGINT 正常收尾（写 metadata.yaml）；
      不要 kill -9，否则 bag 不可读。
"""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# 前三个是「命令 ↔ 状态」对比的必需项，后三个是任务层的关节空间意图。
BAG_TOPICS = [
    "/joint_states",
    "/arm_controller/joint_command",
    "/arm_controller/state",
    "/arm_joint_traj",
    "/arm_joint_waypoints",
    "/arm_servo",
]

# record_bag 的真值写法（launch 参数是字符串，不能用 bool()）。
_TRUTHY = {"1", "true", "yes", "on"}


def _project_root():
    """向上找项目根：同时存在 src/launch_pack 与 install/ 的目录。

    判据与 arm_measure_sim.launch.py 里的 _project_root() 一致，这样无论 launch 是从
    源码树还是 install/ 里解析，都能落回项目根；找不到时退回当前工作目录。
    """
    starts = [
        os.path.dirname(os.path.realpath(__file__)),
        os.path.dirname(os.path.realpath(get_package_share_directory("launch_pack"))),
    ]
    for start in starts:
        current = start
        for _ in range(8):
            if os.path.isdir(os.path.join(current, "src", "launch_pack")) and os.path.isdir(
                os.path.join(current, "install")
            ):
                return current
            parent = os.path.dirname(current)
            if parent == current:
                break
            current = parent
    return os.getcwd()


def _launch_setup(context):
    if context.perform_substitution(LaunchConfiguration("record_bag")).strip().lower() not in _TRUTHY:
        return []

    # bag_dir 为空 = 默认目录（项目根的 .ros/ros2bags）；用户给了就按用户给的（支持 ~ 展开）。
    configured_dir = context.perform_substitution(LaunchConfiguration("bag_dir")).strip()
    bag_root = (
        os.path.expanduser(configured_dir)
        if configured_dir
        else os.path.join(_project_root(), ".ros", "ros2bags")
    )
    bag_name = context.perform_substitution(LaunchConfiguration("bag_name")).strip() or "arm"
    storage = context.perform_substitution(LaunchConfiguration("bag_storage")).strip() or "mcap"
    # 时间戳取一次，保证同一次启动的目录名唯一（rosbag2 不会覆盖已有目录）。
    bag_output = os.path.join(bag_root, f"{bag_name}_{time.strftime('%Y%m%d-%H%M%S')}")

    record_hint = LogInfo(msg=f"[bag] 录制关节命令/状态到 {bag_output}（Ctrl-C 结束 launch 时自动收尾）")
    record = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--storage",
            storage,
            "--output",
            bag_output,
            # launch 里的 stdin 不是终端：关掉键盘交互，避免 rosbag2 等待按键。
            "--disable-keyboard-controls",
            "--topics",
            *BAG_TOPICS,
        ],
        output="screen",
    )
    return [record_hint, record]


def generate_launch_description():
    record_bag_arg = DeclareLaunchArgument(
        "record_bag",
        default_value="true",
        description="是否用 rosbag2 录制关节命令/关节状态（false 时不启动录制进程）",
    )
    bag_dir_arg = DeclareLaunchArgument(
        "bag_dir",
        default_value="",
        description="rosbag 输出根目录；空 = <项目根>/.ros/ros2bags",
    )
    bag_name_arg = DeclareLaunchArgument(
        "bag_name",
        default_value="arm",
        description="bag 名前缀；实际目录为 <bag_dir>/<bag_name>_<时间戳>",
    )
    bag_storage_arg = DeclareLaunchArgument(
        "bag_storage",
        default_value="mcap",
        description="rosbag2 存储插件：mcap（默认）或 sqlite3",
    )

    return LaunchDescription(
        [
            record_bag_arg,
            bag_dir_arg,
            bag_name_arg,
            bag_storage_arg,
            OpaqueFunction(function=_launch_setup),
        ]
    )
