"""
@file arm_real.launch.py
@brief 实机链路（控制器层）：USB CDC 硬件接口 + ArmController + 状态广播

启动内容：
  1. robot_state_publisher  ：xacro 展开 arm_model/model/robotic_arm.urdf（hardware:=real）
                              后发布 /arm_robot_description 与 TF
  2. ros2_control_node      ：controller_manager + ArmRealInterfaces 硬件插件
  3. joint_state_broadcaster：发布 /joint_states
  4. arm_controller         ：状态机控制器，命令直接写 MCU（MIT 阻抗模式）
  5. rviz2                  ：show_rviz:=true 时启动

参数：
  loopback:=true  不打开 USB 设备，把指令读回状态，用于无硬件冒烟（默认 false）
  show_rviz:=true 启动 RViz（默认 false）

用法：
  # 无硬件冒烟
  ros2 launch launch_pack arm_real.launch.py loopback:=true
  # 接真机
  ros2 launch launch_pack arm_real.launch.py

@note 任务层不在这里启动。task_runner 需要 TF 与 /joint_states，确认控制器激活后
      再单独启动。
@warning 实机不支持 measure：MCU 不上报加速度，acc_* 列恒 0，辨识结果不可用。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def generate_launch_description():
    arm_model_share = get_package_share_directory("arm_model")
    launch_pack_share = get_package_share_directory("launch_pack")

    urdf_path = os.path.join(arm_model_share, "model", "robotic_arm.urdf")
    controller_yaml = os.path.join(launch_pack_share, "config", "ros2_controller.yaml")
    real_yaml = os.path.join(launch_pack_share, "config", "ros2_controller_real.yaml")
    rviz_path = os.path.join(launch_pack_share, "rviz", "display_config.rviz")

    # hardware:=real 切换 <ros2_control> 块；loopback 透传给硬件插件。
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_path,
            " hardware:=real loopback:=",
            LaunchConfiguration("loopback"),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # 与仿真链路一致：发布端和订阅端都改到 /arm_robot_description，
    # 避免同 domain 内其他 robot_state_publisher 让 controller_manager 静默加载错模型。
    robot_description_remap = [("robot_description", "arm_robot_description")]

    loopback_arg = DeclareLaunchArgument(
        "loopback",
        default_value="false",
        description="不连 MCU 的本地回环，仅用于无硬件冒烟",
    )
    show_rviz_arg = DeclareLaunchArgument(
        "show_rviz",
        default_value="false",
        description="Whether to start RViz2",
    )
    controller_manager_timeout_arg = DeclareLaunchArgument(
        "controller_manager_timeout",
        default_value="30",
        description="Seconds to wait for the controller manager services",
    )
    switch_timeout_arg = DeclareLaunchArgument(
        "switch_timeout",
        default_value="30",
        description="Seconds to wait for controller activation",
    )
    service_call_timeout_arg = DeclareLaunchArgument(
        "service_call_timeout",
        default_value="30",
        description="Seconds to wait for controller manager service responses",
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        remappings=robot_description_remap,
        output="screen",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            robot_description,
            # 先加载仿真默认值，再用实机差异覆盖；后一个文件里的同名参数生效。
            ParameterFile(controller_yaml),
            ParameterFile(real_yaml),
        ],
        remappings=robot_description_remap,
        on_exit=Shutdown(),
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            LaunchConfiguration("controller_manager_timeout"),
            "--switch-timeout",
            LaunchConfiguration("switch_timeout"),
            "--service-call-timeout",
            LaunchConfiguration("service_call_timeout"),
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            LaunchConfiguration("controller_manager_timeout"),
            "--switch-timeout",
            LaunchConfiguration("switch_timeout"),
            "--service-call-timeout",
            LaunchConfiguration("service_call_timeout"),
            "arm_controller",
        ],
        output="screen",
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path],
        condition=IfCondition(LaunchConfiguration("show_rviz")),
        output="screen",
    )

    load_joint_state_broadcaster = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                LogInfo(msg="ros2_control_node started, spawning joint_state_broadcaster"),
                joint_state_broadcaster,
            ],
        )
    )

    load_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[
                LogInfo(msg="joint_state_broadcaster spawned, spawning arm_controller"),
                arm_controller,
            ],
        )
    )

    loopback_hint = LogInfo(
        msg="[提示] 若使用 loopback:=true，本进程不会打开 USB 设备，"
        "读到的状态来自指令回读；接真机前务必去掉该参数。"
    )

    return LaunchDescription(
        [
            loopback_arg,
            show_rviz_arg,
            controller_manager_timeout_arg,
            switch_timeout_arg,
            service_call_timeout_arg,
            loopback_hint,
            robot_state_pub,
            ros2_control_node,
            load_joint_state_broadcaster,
            load_arm_controller,
            rviz2,
        ]
    )
