"""
@file task_cart_traj.launch.py
@brief 一键启动：MuJoCo 仿真链路 + tasks 的 cart_traj 任务（笛卡尔两点轨迹）

行为：
  - 任务节点延迟 task_delay 秒（默认 6.0）后再启动，用来等控制器的上电自动复位
    （ros2_controller.yaml 的 exp_state: reset）跑完，避免任务与复位抢状态 / 抢轨迹起点；
    仿真已经在跑时用 task_delay:=0.0 跳过等待
  - start_sim 为 true（默认）时先拉起 arm_controller_test_sim.launch.py，再启动
    tasks/task_runner，任务链为 cart_traj -> idel
  - 任务节点等控制器与 /joint_states 就绪后切 exp_state=cart_traj，用 TF 取当前末端
    位姿作起点，目标 = 起点 + cart_traj_target_offset，向 /arm_cart_traj 发布两点轨迹；
    等轨迹时长走完并校验到位误差后退出

用法：
    ros2 launch launch_pack task_cart_traj.launch.py
    ros2 launch launch_pack task_cart_traj.launch.py start_sim:=false        # 仿真已在跑
    ros2 launch launch_pack task_cart_traj.launch.py cart_traj_motion_duration:=5.0

@note 默认目标只改位置（z 抬升 5 cm）。姿态偏移是旋转向量相加，只在小偏转角下近似成立，
      要改目标请直接改下面的 cart_target_offset 列表（数组不便作为 launch 参数传递）。
@warning 仿真已在运行时务必加 start_sim:=false，否则会起第二个 controller_manager。
@see src/tasks/task/cart_traj_task.hpp
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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

    # 默认目标：相对当前末端位姿，仅 z 抬升 5 cm（改这里即可换目标）
    cart_target_offset = [0.0, 0.0, 0.2, 0.0, 0.0, 0.0]

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
    settle_arg = DeclareLaunchArgument(
        "cart_traj_settle_duration", default_value="0.3", description="切入 cart_traj 后等待控制器稳定的时间（秒）"
    )
    motion_arg = DeclareLaunchArgument(
        "cart_traj_motion_duration", default_value="3.0", description="轨迹运动时长（秒）"
    )
    tolerance_arg = DeclareLaunchArgument(
        "cart_traj_position_tolerance", default_value="0.005", description="到位判定容差（米）"
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        condition=IfCondition(LaunchConfiguration("start_sim")),
        launch_arguments={
            "show_rviz": LaunchConfiguration("show_rviz"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    # 任务节点延迟启动：等控制器的“上电自动复位”（ros2_controller.yaml 里 exp_state: reset）
    # 跑完再切状态/发轨迹。默认 6.0 s ≈ 控制器激活(~2.5 s) + reset_duration(3 s) + 余量；
    # 仿真已经在运行时用 task_delay:=0.0 跳过等待。
    task_delay_arg = DeclareLaunchArgument(
        "task_delay",
        default_value="6.0",
        description="启动任务节点前的等待时间（秒），用于等控制器上电自动复位完成",
    )

    task = Node(
        package="tasks",
        executable="task_runner",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                "startup_timeout": ParameterValue(LaunchConfiguration("startup_timeout"), value_type=float),
                "task_sequence": ["cart_traj"],
                "cart_traj_target_offset": cart_target_offset,
                "cart_traj_settle_duration": ParameterValue(
                    LaunchConfiguration("cart_traj_settle_duration"), value_type=float
                ),
                "cart_traj_motion_duration": ParameterValue(
                    LaunchConfiguration("cart_traj_motion_duration"), value_type=float
                ),
                "cart_traj_position_tolerance": ParameterValue(
                    LaunchConfiguration("cart_traj_position_tolerance"), value_type=float
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            start_sim_arg,
            use_sim_time_arg,
            show_rviz_arg,
            startup_timeout_arg,
            settle_arg,
            motion_arg,
            tolerance_arg,
            sim,
            task_delay_arg,
            # 延迟启动任务节点，避免与控制器的上电自动复位抢状态 / 抢轨迹起点
            TimerAction(
                period=LaunchConfiguration("task_delay"),
                actions=[task],
            ),
        ]
    )
