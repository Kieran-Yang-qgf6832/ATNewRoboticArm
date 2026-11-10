"""
@file arm_task_vision_sim.launch.py
@brief 视觉一条龙：抬高视觉场景 + 相机 TF/检测 + task_runner（默认 box_move_demo）

用法：
    # 默认：scene_vision_raised.xml（箱体/相机 z=0.6）+ 检测 + box_move_demo
    ros2 launch launch_pack arm_task_vision_sim.launch.py

    # 低处回退验证：原 scene_vision.xml + box_move_low
    ros2 launch launch_pack arm_task_vision_sim.launch.py \
        task_sequence:=box_move_low \
        mujoco_model:=$(ros2 pkg prefix arm_model)/share/arm_model/model/scene_vision.xml

    # 需要 target_face 检测预览时
    ros2 launch launch_pack arm_task_vision_sim.launch.py vision_display:=true

@note view_sim.launch.py 的 scene_path 直接复用同一个 mujoco_model：渲染场景与
      相机 TF 必须来自同一份 MJCF，否则 target_face 会与实际箱体位置不一致。
@note 检测节点刻意不用 use_sim_time（见 view_sim.launch.py 与 docs/bugs.md C8）：
      仿真时间由图像 header.stamp 提供，task_runner 才用 use_sim_time。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    arm_model_share = get_package_share_directory("arm_model")
    launch_pack_share = get_package_share_directory("launch_pack")
    tasks_share = get_package_share_directory("tasks")
    view_share = get_package_share_directory("view")

    task_sim_launch = os.path.join(launch_pack_share, "launch", "arm_task_sim.launch.py")
    view_sim_launch = os.path.join(view_share, "launch", "view_sim.launch.py")

    default_tasks_config = os.path.join(tasks_share, "config", "tasks.yaml")
    default_scene = os.path.join(arm_model_share, "model", "scene_vision_raised.xml")

    task_sequence_arg = DeclareLaunchArgument(
        "task_sequence",
        default_value="box_move_demo",
        description="tasks.yaml 里的序列名；默认 box_move_demo（抬高场景）",
    )
    tasks_config_arg = DeclareLaunchArgument(
        "tasks_config",
        default_value=default_tasks_config,
        description="任务 YAML 路径；默认 tasks share 下的 config/tasks.yaml",
    )
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="是否关掉 MuJoCo 自带 UI 窗口（相机插件在 headless 下仍工作）",
    )
    show_rviz_arg = DeclareLaunchArgument(
        "show_rviz",
        default_value="false",
        description="是否启动 RViz2",
    )
    mujoco_model_arg = DeclareLaunchArgument(
        "mujoco_model",
        default_value=default_scene,
        description="含 vision_camera 与红色箱体的 MJCF；同时作为 view_sim 的 scene_path",
    )
    vision_display_arg = DeclareLaunchArgument(
        "vision_display",
        default_value="false",
        description="是否打开检测节点的 OpenCV 预览窗口",
    )
    exit_when_idle_arg = DeclareLaunchArgument(
        "exit_when_idle",
        default_value="true",
        description="序列结束后 task_runner 是否退出；true 会连带关闭整个 launch",
    )

    task_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(task_sim_launch),
        launch_arguments={
            "task_sequence": LaunchConfiguration("task_sequence"),
            "tasks_config": LaunchConfiguration("tasks_config"),
            "headless": LaunchConfiguration("headless"),
            "show_rviz": LaunchConfiguration("show_rviz"),
            "mujoco_model": LaunchConfiguration("mujoco_model"),
            "exit_when_idle": LaunchConfiguration("exit_when_idle"),
        }.items(),
    )

    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(view_sim_launch),
        launch_arguments={
            # 关键：相机 TF 与 MuJoCo 渲染必须指向同一份场景文件
            "scene_path": LaunchConfiguration("mujoco_model"),
            "display": LaunchConfiguration("vision_display"),
        }.items(),
    )

    return LaunchDescription(
        [
            task_sequence_arg,
            tasks_config_arg,
            headless_arg,
            show_rviz_arg,
            mujoco_model_arg,
            vision_display_arg,
            exit_when_idle_arg,
            task_sim,
            vision,
        ]
    )
