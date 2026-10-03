"""
@file task_teach_pendant.launch.py
@brief 一键启动：MuJoCo 仿真链路 + tasks 的 teach_pendant 任务（示教 / 零力拖动）

行为：
  - start_sim 为 true（默认）时先拉起 arm_controller_test_sim.launch.py，再启动
    tasks/task_runner，任务链为 idel -> teach_pendant -> idel
    （控制器只允许 teach_pendant 从 idel 进出，runner 会自动补两侧的 idel）
  - 进入示教后控制器把位置增益置零、只保留速度增益并输出重力补偿力矩，机械臂可被外力拖动；
    保持 teach_pendant_duration 秒后自动切回 idel 并打印各关节角变化量，然后退出

怎么“拖”：
  - 真机：直接用手拖动
  - 仿真：在 MuJoCo 自带的交互界面里对连杆施加外力

用法：
    ros2 launch launch_pack task_teach_pendant.launch.py
    ros2 launch launch_pack task_teach_pendant.launch.py start_sim:=false
    ros2 launch launch_pack task_teach_pendant.launch.py teach_pendant_duration:=30.0

@note teach_pendant_duration 设为 0 或负数表示不限时（保持到 Ctrl-C）；
      此时请同时把 task_timeout 调大，否则整条任务链会先超时中止。
@warning 仿真已在运行时务必加 start_sim:=false，否则会起第二个 controller_manager。
@see src/tasks/task/teach_pendant_task.hpp
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
    duration_arg = DeclareLaunchArgument(
        "teach_pendant_duration", default_value="15.0", description="示教时长（秒）；<= 0 表示不限时"
    )
    task_timeout_arg = DeclareLaunchArgument(
        "task_timeout", default_value="120.0", description="整条任务链超时（秒），需大于示教时长"
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
                "task_sequence": ["teach_pendant"],
                "teach_pendant_duration": ParameterValue(
                    LaunchConfiguration("teach_pendant_duration"), value_type=float
                ),
                "task_timeout": ParameterValue(LaunchConfiguration("task_timeout"), value_type=float),
            }
        ],
    )

    return LaunchDescription(
        [
            start_sim_arg,
            use_sim_time_arg,
            show_rviz_arg,
            startup_timeout_arg,
            duration_arg,
            task_timeout_arg,
            sim,
            task,
        ]
    )
