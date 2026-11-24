"""基座运动补偿 / 末端惯性空间保持 的一键仿真。

流程（全部由本文件编排）：
  1. 用 generate_float_mjcf.py 从 arm_model 的固定基座模型生成浮动基座 MJCF（/tmp）；
  2. 复用 launch_pack 的 arm_controller_test_sim.launch.py 起 MuJoCo + 控制器链，
     传入生成的场景与 config/inertial_hold_overlay.yaml，并打开 floating_base 分支
     （URDF 去掉 world 固定关节，base_link 成为 TF 根）；
  3. 起 base_motion_tf_node，订阅基座状态话题补上 world→base_link 动态 TF；
  4. TimerAction 在 switch_hold_after 秒把控制器从 reset 切到 inertial_hold；
  5. 基座插件按 start_delay（overlay YAML 里的 12 s）开始跑轨迹。

用法：
    ros2 launch arm_base_motion inertial_hold_sim.launch.py
    ros2 launch arm_base_motion inertial_hold_sim.launch.py headless:=true show_rviz:=false
    # 评估（另开一个终端，等基座开始动之后）：
    ros2 run arm_base_motion evaluate_inertial_hold.py --duration 30

注意：本 launch 生成的 MJCF 在 $TMPDIR/arm_base_motion 下，每次启动都会重新生成；
生成物不要手改（改源模型 arm_model/model/robotic_arm.xml）。
"""

import os
import subprocess
import sys
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arm_base_motion_share = get_package_share_directory("arm_base_motion")
    arm_base_motion_prefix = get_package_prefix("arm_base_motion")
    launch_pack_share = get_package_share_directory("launch_pack")
    arm_model_share = get_package_share_directory("arm_model")

    # 生成浮动基座场景。放在 launch 解析阶段同步执行：ros2_control 启动时文件必须已就绪。
    generator = os.path.join(arm_base_motion_prefix, "lib", "arm_base_motion", "generate_float_mjcf.py")
    if not os.path.isfile(generator):
        raise RuntimeError("generate_float_mjcf.py not found at " + generator + "; rebuild arm_base_motion")

    output_dir = os.path.join(tempfile.gettempdir(), "arm_base_motion")
    model = subprocess.run(
        [
            sys.executable,
            generator,
            "--arm-model-dir",
            os.path.join(arm_model_share, "model"),
            "--out-dir",
            output_dir,
        ],
        capture_output=True,
        text=True,
    )
    if model.returncode != 0:
        raise RuntimeError("floating MJCF generation failed:\n" + model.stderr)
    scene_path = model.stdout.strip().splitlines()[-1]

    overlay_path = os.path.join(arm_base_motion_share, "config", "inertial_hold_overlay.yaml")

    switch_hold_after_arg = DeclareLaunchArgument(
        "switch_hold_after",
        default_value="8.0",
        description="控制器启动后多少秒把 exp_state 从 reset 切到 inertial_hold（要早于基座 start_delay）",
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_pack_share, "launch", "arm_controller_test_sim.launch.py")
        ),
        launch_arguments={
            "mujoco_model": scene_path,
            "controller_overlay": overlay_path,
            "floating_base": "true",
            "headless": LaunchConfiguration("headless"),
            "show_rviz": LaunchConfiguration("show_rviz"),
            "record_bag": LaunchConfiguration("record_bag"),
        }.items(),
    )

    tf_node = Node(
        package="arm_base_motion",
        executable="base_motion_tf_node",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"topic": "/base_motion_state"},
            {"world_frame": "world"},
            {"base_frame": "base_link"},
            {"tf_rate": 100.0},
        ],
    )

    switch_to_hold = TimerAction(
        period=LaunchConfiguration("switch_hold_after"),
        actions=[
            LogInfo(msg="[inertial_hold] switching exp_state: reset -> inertial_hold"),
            ExecuteProcess(
                cmd=["ros2", "param", "set", "/arm_controller", "exp_state", "inertial_hold"],
                output="screen",
            ),
        ],
    )

    headless_arg = DeclareLaunchArgument("headless", default_value="false")
    show_rviz_arg = DeclareLaunchArgument("show_rviz", default_value="true")
    record_bag_arg = DeclareLaunchArgument("record_bag", default_value="false")

    return LaunchDescription(
        [
            headless_arg,
            show_rviz_arg,
            record_bag_arg,
            switch_hold_after_arg,
            LogInfo(msg="[inertial_hold] floating MJCF: " + scene_path),
            sim,
            tf_node,
            switch_to_hold,
        ]
    )
