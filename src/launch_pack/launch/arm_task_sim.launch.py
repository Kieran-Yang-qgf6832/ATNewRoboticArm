"""
@file arm_task_sim.launch.py
@brief 仿真 + 任务层一条龙：拉起 arm_controller_test_sim.launch.py 与 tasks/task_runner

用法：
    # 默认打开 Qt 版 joint_console（仿真 + 控制器 + 窗口）
    ros2 launch launch_pack arm_task_sim.launch.py

    # 跑任意 tasks.yaml 序列；任务结束后整个 launch 自动关闭
    ros2 launch launch_pack arm_task_sim.launch.py task_sequence:=pos_move_demo

    # 无界面仿真（Qt 窗口仍需要 DISPLAY，只是不开 MuJoCo/RViz 窗口）
    ros2 launch launch_pack arm_task_sim.launch.py headless:=true show_rviz:=false

    # 想让 task_runner 常驻、之后用 ros2 param set 换序列
    ros2 launch launch_pack arm_task_sim.launch.py exit_when_idle:=false

@note 视觉检测不在这里启动；box_move 演示用 arm_task_vision_sim.launch.py。
@note task_runner 与仿真并行启动，其 wait_for_controller / wait_for_joint_states
      会等控制器就绪；startup_timeout 放宽到 30 s，避免慢启动时误判。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arm_model_share = get_package_share_directory("arm_model")
    launch_pack_share = get_package_share_directory("launch_pack")
    tasks_share = get_package_share_directory("tasks")

    sim_launch = os.path.join(launch_pack_share, "launch", "arm_controller_test_sim.launch.py")
    default_tasks_config = os.path.join(tasks_share, "config", "tasks.yaml")
    default_mujoco_model = os.path.join(arm_model_share, "model", "scene.xml")

    task_sequence_arg = DeclareLaunchArgument(
        "task_sequence",
        default_value="joint_console_ui",
        description="tasks.yaml 里的序列名；默认打开 Qt 版 joint_console",
    )
    tasks_config_arg = DeclareLaunchArgument(
        "tasks_config",
        default_value=default_tasks_config,
        description="任务 YAML 路径；默认 tasks share 下的 config/tasks.yaml",
    )
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="是否关掉 MuJoCo 自带 UI 窗口（Qt joint_console 仍需要 DISPLAY）",
    )
    show_rviz_arg = DeclareLaunchArgument(
        "show_rviz",
        default_value="false",
        description="是否启动 RViz2；任务演示默认关闭以减少负载",
    )
    mujoco_model_arg = DeclareLaunchArgument(
        "mujoco_model",
        default_value=default_mujoco_model,
        description="MuJoCo MJCF 场景；视觉场景可传 scene_vision.xml / scene_vision_raised.xml",
    )
    exit_when_idle_arg = DeclareLaunchArgument(
        "exit_when_idle",
        default_value="true",
        description="序列/UI 结束后 task_runner 是否退出；true 会连带关闭整个 launch",
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "show_rviz": LaunchConfiguration("show_rviz"),
            "mujoco_model": LaunchConfiguration("mujoco_model"),
        }.items(),
    )

    task_runner = Node(
        package="tasks",
        executable="task_runner",
        name="task_runner",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "tasks_config": LaunchConfiguration("tasks_config"),
                "task_sequence": LaunchConfiguration("task_sequence"),
                "exit_when_idle": ParameterValue(LaunchConfiguration("exit_when_idle"), value_type=bool),
                # 仿真控制器可能比 task_runner 晚几秒就绪；task_runner 会重试等待。
                "startup_timeout": 30.0,
            }
        ],
        # 任务/UI 退出后关闭整个仿真场景；想常驻请把 exit_when_idle 设为 false。
        on_exit=Shutdown(),
    )

    return LaunchDescription(
        [
            task_sequence_arg,
            tasks_config_arg,
            headless_arg,
            show_rviz_arg,
            mujoco_model_arg,
            exit_when_idle_arg,
            sim,
            task_runner,
        ]
    )
