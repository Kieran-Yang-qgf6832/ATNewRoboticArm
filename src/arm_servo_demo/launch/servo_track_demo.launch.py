"""伺服跟踪演示的一键入口：MuJoCo 仿真 + 控制器 + 演示节点。

用法：
    ros2 launch arm_servo_demo servo_track_demo.launch.py
    # 只跑 20 s（一个周期）、不录 bag、不显示仿真 UI
    ros2 launch arm_servo_demo servo_track_demo.launch.py duration:=20.0 record_bag:=false headless:=true

演示节点会自己完成「摆到 ready -> 切 servo -> 跟踪 40 s -> 停住回 idel」，
结束时在终端打印平均/最大误差，并以退出码给出通过（0）或失败（1）。
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

    demo = Node(
        package="arm_servo_demo",
        executable="servo_track_demo",
        name="servo_track_demo",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "radius": ParameterValue(LaunchConfiguration("radius"), value_type=float),
            "period": ParameterValue(LaunchConfiguration("period"), value_type=float),
            "gain": ParameterValue(LaunchConfiguration("gain"), value_type=float),
            "max_speed": ParameterValue(LaunchConfiguration("max_speed"), value_type=float),
            "duration": ParameterValue(LaunchConfiguration("duration"), value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false",
                              description="关掉 MuJoCo 自带 UI（true = 纯命令行）"),
        DeclareLaunchArgument("show_rviz", default_value="true",
                              description="是否启动 RViz（球体 Marker 在 RViz 里显示）"),
        DeclareLaunchArgument("record_bag", default_value="false",
                              description="是否录 rosbag（演示默认不录）"),
        DeclareLaunchArgument("radius", default_value="0.05",
                              description="球轨迹半径（米）；圆心偏向 base_link 的 +x"),
        DeclareLaunchArgument("period", default_value="20.0",
                              description="圆周周期（秒）"),
        DeclareLaunchArgument("gain", default_value="2.0",
                              description="位置误差反馈增益（1/s）"),
        DeclareLaunchArgument("max_speed", default_value="0.05",
                              description="速度指令限幅（m/s）"),
        DeclareLaunchArgument("duration", default_value="40.0",
                              description="跟踪时长（秒），默认两个周期"),
        LogInfo(msg="[servo_track_demo] 仿真启动后约 10 s 开始：先摆到 ready，再切入 servo 跟踪虚拟球。"
                    "RViz 里橙色小球即目标，终端每 2 s 打印一次误差。"),
        sim_launch,
        demo,
    ])
