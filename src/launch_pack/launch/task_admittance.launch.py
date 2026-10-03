"""
@file task_admittance.launch.py
@brief 一键启动：MuJoCo 仿真链路 + tasks 的 admittance 任务（导纳柔顺）

行为：
  - start_sim 为 true（默认）时先拉起 arm_controller_test_sim.launch.py，再启动
    tasks/task_runner，任务链为 admittance -> idel
  - 任务节点切 exp_state=admittance，用 TF 取当前末端位姿作起点、目标 = 起点 + 偏移，
    向 /arm_admittance 发一条两点“期望轨迹”；控制器估计外力（实测力矩 − 模型力矩）
    并按导纳模型柔顺跟随 —— 对比 cart_traj，这里是柔顺跟随而不是硬跟踪

用法：
    ros2 launch launch_pack task_admittance.launch.py
    ros2 launch launch_pack task_admittance.launch.py start_sim:=false
    # 另开终端施力（真机手拖 / MuJoCo 界面鼠标施力）即可看到末端柔顺偏离期望轨迹

@note 导纳参数 admittance_mass / admittance_damping / admittance_stiffness 属于控制器参数，
      在 config/ros2_controller.yaml 里配置；本文件只负责期望轨迹与到位容差。
@note AdmittanceCmd.force 控制器当前未使用（外力由控制器自行估计），故不开放该参数。
@warning 仿真已在运行时务必加 start_sim:=false，否则会起第二个 controller_manager。
@see src/tasks/task/admittance_task.hpp
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

    # 默认期望轨迹：相对当前末端位姿，z 抬升 5 cm（改这里即可换目标）
    admittance_target_offset = [0.0, 0.0, 0.05, 0.0, 0.0, 0.0]

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
        "admittance_settle_duration", default_value="0.3", description="切入 admittance 后等待控制器稳定的时间（秒）"
    )
    motion_arg = DeclareLaunchArgument(
        "admittance_motion_duration", default_value="3.0", description="期望轨迹运动时长（秒）"
    )
    tolerance_arg = DeclareLaunchArgument(
        "admittance_position_tolerance",
        default_value="0.02",
        description="到位判定容差（米）；柔顺跟随会偏离期望轨迹，默认比 cart_traj 宽",
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
                "task_sequence": ["admittance"],
                "admittance_target_offset": admittance_target_offset,
                "admittance_settle_duration": ParameterValue(
                    LaunchConfiguration("admittance_settle_duration"), value_type=float
                ),
                "admittance_motion_duration": ParameterValue(
                    LaunchConfiguration("admittance_motion_duration"), value_type=float
                ),
                "admittance_position_tolerance": ParameterValue(
                    LaunchConfiguration("admittance_position_tolerance"), value_type=float
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
            task,
        ]
    )
