"""
@file task_measure.launch.py
@brief 一键启动：MuJoCo 仿真链路 + tasks 的 measure 任务（动力学参数辨识）

行为：
  - 任务节点延迟 task_delay 秒（默认 6.0）后再启动，用来等控制器的上电自动复位
    （ros2_controller.yaml 的 exp_state: reset）跑完，避免任务与复位抢状态；
    仿真已经在跑时用 task_delay:=0.0 跳过等待
  - start_sim 为 true（默认）时先拉起 arm_controller_test_sim.launch.py，再启动
    tasks/task_runner，任务链为 idel -> measure -> idel
    （控制器只允许 measure 从 idel 进出，runner 会自动补两侧的 idel）
  - 控制器收到 measure 后自行完成：生成傅里叶激励轨迹 -> 移动到起点 -> 执行激励轨迹并
    采集位置/速度/力矩 -> 写入 CSV（控制器参数 measure_csv_file_path，默认
    /tmp/measured_for_identification.csv）
  - 任务层观测不到控制器内部阶段，因此等待 measure_duration 后请求切回 idel，
    再等 measure_return_settle 结束本任务

用法：
    ros2 launch launch_pack task_measure.launch.py
    ros2 launch launch_pack task_measure.launch.py start_sim:=false
    ros2 launch launch_pack task_measure.launch.py measure_duration:=25.0

@warning measure_duration 必须 >= 控制器侧总时长：
         measure_move_to_start_duration + measure_trajectory_period ×
         measure_trajectory_repeat_cnt + 激励轨迹生成时间（默认约 3 + 10×1 = 13 s）。
         配小了任务层会过早进入下一个任务，可能导致控制器离开 measure 时 enter() 失败。
@note 连续两次 measure 之间要有间隔：上一轮 CSV 还在写盘时，控制器的 enter() 会返回 false，
      表现为任务节点报 “task measure failed to run”。
@note 测量期间机械臂会按激励轨迹自动运动，请确保工作空间安全。
@warning 仿真已在运行时务必加 start_sim:=false，否则会起第二个 controller_manager。
@see src/tasks/task/measure_task.hpp
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
        "measure_duration",
        default_value="20.0",
        description="任务层在 measure 状态保持的时长（秒），需覆盖控制器整个辨识流程",
    )
    settle_arg = DeclareLaunchArgument(
        "measure_return_settle", default_value="1.0", description="请求回 idel 后再等待的切换时间（秒）"
    )
    task_timeout_arg = DeclareLaunchArgument(
        "task_timeout", default_value="120.0", description="整条任务链超时（秒），需大于 measure_duration"
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
                "task_sequence": ["measure"],
                "measure_duration": ParameterValue(LaunchConfiguration("measure_duration"), value_type=float),
                "measure_return_settle": ParameterValue(
                    LaunchConfiguration("measure_return_settle"), value_type=float
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
            settle_arg,
            task_timeout_arg,
            sim,
            task_delay_arg,
            # 延迟启动任务节点，避免与控制器的上电自动复位抢状态 / 抢轨迹起点
            TimerAction(
                period=LaunchConfiguration("task_delay"),
                actions=[task],
            ),
        ]
    )
