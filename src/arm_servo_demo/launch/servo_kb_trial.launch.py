"""键盘伺服试验：MuJoCo 仿真 + 控制器 + 试验追踪节点。

键盘节点需要独占一个终端，所以不在这里启动。用法：

    终端 1：
        ros2 launch arm_servo_demo servo_kb_trial.launch.py
    终端 2（等终端 1 里出现 tracking: ... 之后）：
        ros2 run arm_servo_demo ball_keyboard

按键：W/S ±x、A/D ±y、E/Q ±z、X 结束（最长 120 s）。
试验数据写到 .ros/servo_trials/servo_kb_<时间戳>.csv。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("launch_pack"), "launch",
                         "arm_controller_test_sim.launch.py")),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "show_rviz": LaunchConfiguration("show_rviz"),
            "record_bag": LaunchConfiguration("record_bag"),
        }.items(),
    )

    trial = Node(
        package="arm_servo_demo",
        executable="servo_track_test",
        name="servo_track_test",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "gain": ParameterValue(LaunchConfiguration("gain"), value_type=float),
            "max_speed": ParameterValue(LaunchConfiguration("max_speed"), value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false",
                              description="关掉 MuJoCo 自带 UI（true = 纯命令行）"),
        DeclareLaunchArgument("show_rviz", default_value="true",
                              description="是否启动 RViz（球体 Marker 在 RViz 里显示）"),
        DeclareLaunchArgument("record_bag", default_value="false",
                              description="是否录 rosbag（试验默认只写 CSV）"),
        DeclareLaunchArgument("gain", default_value="2.0",
                              description="位置误差反馈增益（1/s）"),
        DeclareLaunchArgument("max_speed", default_value="0.08",
                              description="速度指令限幅（m/s）"),
        LogInfo(msg="[servo_kb_trial] 仿真与追踪节点启动中；看到 tracking: 之后，"
                    "另开一个终端运行 ros2 run arm_servo_demo ball_keyboard。"
                    "按键 W/S ±x、A/D ±y、E/Q ±z、X 结束。"),
        sim_launch,
        trial,
    ])
