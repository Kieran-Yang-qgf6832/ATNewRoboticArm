"""
@file task_idel.launch.py
@brief 一键启动：MuJoCo 仿真链路 + tasks 的 idel 任务（回空闲 / 锁定当前位置）

行为：
  - start_sim 为 true（默认）时先拉起 arm_controller_test_sim.launch.py，再启动
    tasks/task_runner，任务链只有一项：idel
  - 控制器切到 idel 后锁定当前位置（位置保持），任务节点随即结束并退出
  - 用途：把控制器从任何状态“安全拉回空闲”，相当于一个「停止 / 回空闲」按钮，
    也常用于上一次任务链失败后的复位

用法：
    ros2 launch launch_pack task_idel.launch.py
    ros2 launch launch_pack task_idel.launch.py start_sim:=false       # 仿真已在跑

@note 本任务的参数 idel_hold_duration 在这里不会生效：任务链只有 idel 一项，
      节点在进入 idel 后立即结束（这是刻意设计，便于当“回空闲”按钮用）。
      想让节点继续跑后续任务，请用 ros2 run tasks task_runner 指定更长的 task_sequence。
@warning 仿真已在运行时务必加 start_sim:=false，否则会起第二个 controller_manager。
@see src/tasks/task/idel_task.hpp
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    launch_pack_share = get_package_share_directory("launch_pack")
    sim_launch = os.path.join(launch_pack_share, "launch", "arm_controller_test_sim.launch.py")

    start_sim_arg = DeclareLaunchArgument(
        "start_sim", default_value="true", description="是否同时启动 MuJoCo 仿真（仿真已在运行时设为 false）"
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="使用仿真时间（仿真下必须为 true）"
    )
    show_rviz_arg = DeclareLaunchArgument(
        "show_rviz", default_value="true", description="start_sim 为 true 时是否同时启动 RViz2"
    )
    startup_timeout_arg = DeclareLaunchArgument(
        "startup_timeout", default_value="60.0", description="等待控制器与 /joint_states 就绪的超时（秒）"
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        condition=IfCondition(LaunchConfiguration("start_sim")),
        launch_arguments={
            "show_rviz": LaunchConfiguration("show_rviz"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    task = Node(
        package="tasks",
        executable="task_runner",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                "startup_timeout": ParameterValue(LaunchConfiguration("startup_timeout"), value_type=float),
                "task_sequence": ["idel"],
            }
        ],
    )

    return LaunchDescription(
        [
            start_sim_arg,
            use_sim_time_arg,
            show_rviz_arg,
            startup_timeout_arg,
            sim,
            task,
        ]
    )
