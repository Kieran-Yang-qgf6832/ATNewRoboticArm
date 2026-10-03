"""
@file task_reset.launch.py
@brief 一键启动：MuJoCo 仿真链路 + tasks 的 reset 任务（复位回零）

行为：
  - 任务节点延迟 task_delay 秒（默认 6.0）后再启动，用来等控制器的上电自动复位
    （ros2_controller.yaml 的 exp_state: reset）跑完，避免任务与复位抢状态；
    仿真已经在跑时用 task_delay:=0.0 跳过等待
  - start_sim 为 true（默认）时，先按序拉起 arm_controller_test_sim.launch.py：
    robot_state_publisher -> ros2_control_node(MuJoCo) -> joint_state_broadcaster
    -> sim_pid_controller + arm_controller -> rviz2
  - 同时启动 tasks/task_runner，任务链为 idel -> reset -> idel（runner 自动为 reset
    补 idel 门控）；任务节点自身会等待控制器与 /joint_states 就绪（startup_timeout），
    因此不需要额外 sleep。
  - 各关节进入 reset_tolerance 容差后任务节点自动退出（退出码 0）。

用法：
    # 从零一键起仿真并复位
    ros2 launch launch_pack task_reset.launch.py
    # 仿真已在跑，只跑任务（避免起第二个 controller_manager）
    ros2 launch launch_pack task_reset.launch.py start_sim:=false
    # 仿真收敛慢时放宽容差、延长超时
    ros2 launch launch_pack task_reset.launch.py reset_tolerance:=0.05 reset_timeout:=30.0

@warning 复位目标 reset_joint_pos 由控制器参数决定（config/ros2_controller.yaml，默认全 0），
         本文件只透传“容差/时长/超时”。若修改了控制器的 reset_joint_pos 或 reset_duration，
         请同步修改这里的 reset_joint_pos / reset_duration，否则任务层会一直等不到到位。
@note 仿真已在运行时务必加 start_sim:=false，否则会启动第二个 controller_manager 而互相冲突。
@see task_runner.cpp 参数说明（startup_timeout / reset_* 等）
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

    # ---- 通用参数 ----
    start_sim_arg = DeclareLaunchArgument(
        "start_sim",
        default_value="true",
        description="是否同时启动 MuJoCo 仿真（仿真已在运行时设为 false）",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="使用仿真时间（仿真下必须为 true，实机为 false）",
    )
    show_rviz_arg = DeclareLaunchArgument(
        "show_rviz",
        default_value="true",
        description="start_sim 为 true 时是否同时启动 RViz2",
    )
    startup_timeout_arg = DeclareLaunchArgument(
        "startup_timeout",
        default_value="60.0",
        description="任务节点等待控制器与 /joint_states 就绪的超时（秒）",
    )

    # ---- reset 任务参数（需与控制器同名参数保持一致）----
    reset_duration_arg = DeclareLaunchArgument(
        "reset_duration",
        default_value="3.0",
        description="复位时长（秒），需与控制器 reset_duration 一致；任务层至少等这么久",
    )
    reset_tolerance_arg = DeclareLaunchArgument(
        "reset_tolerance",
        default_value="0.01",
        description="到位容差（rad），控制器与任务层共用此判据",
    )
    reset_timeout_arg = DeclareLaunchArgument(
        "reset_timeout",
        default_value="30.0",
        description="任务层复位超时（秒），超时则整条任务链中止",
    )

    # 启动仿真：把 show_rviz / use_sim_time 透传给 arm_controller_test_sim.launch.py
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
                "task_sequence": ["reset"],
                "reset_duration": ParameterValue(LaunchConfiguration("reset_duration"), value_type=float),
                "reset_tolerance": ParameterValue(LaunchConfiguration("reset_tolerance"), value_type=float),
                "reset_timeout": ParameterValue(LaunchConfiguration("reset_timeout"), value_type=float),
            }
        ],
    )

    return LaunchDescription(
        [
            start_sim_arg,
            use_sim_time_arg,
            show_rviz_arg,
            startup_timeout_arg,
            reset_duration_arg,
            reset_tolerance_arg,
            reset_timeout_arg,
            sim,
            task_delay_arg,
            # 延迟启动任务节点，避免与控制器的上电自动复位抢状态 / 抢轨迹起点
            TimerAction(
                period=LaunchConfiguration("task_delay"),
                actions=[task],
            ),
        ]
    )
