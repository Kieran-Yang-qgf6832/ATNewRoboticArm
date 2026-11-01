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
  6. ros2 bag record        ：把**上位机下发的关节轨迹指令**与 /joint_states 录成 bag
                              （record_bag:=false 可关；默认目录见 bag_path）

参数：
  loopback:=true  不打开 USB 设备，把指令读回状态，用于无硬件冒烟（默认 false）
  show_rviz:=true 启动 RViz（默认 false）
  record_bag:=false 不录 bag（默认录）
  bag_path:=...    bag 输出目录（必须尚不存在，默认 ~/ros2_bag/arm_real_<时间戳>）

用法：
  # 无硬件冒烟
  ros2 launch launch_pack arm_real.launch.py loopback:=true
  # 接真机
  ros2 launch launch_pack arm_real.launch.py
  # 真机跑，但这次不录 bag
  ros2 launch launch_pack arm_real.launch.py record_bag:=false

@note 任务层不在这里启动。task_runner 需要 TF 与 /joint_states，确认控制器激活后
      再单独启动。
@note 实机上 measure **可用**（辨识已不依赖加速度，见 docs/parameter_identification_guide.md）。
      CSV 没有 acc_* 列、joint6 冻结（measure_frozen_joints）。但不在这里自动启动：
      确认标定与通信无误后，再单独跑 task_runner 的 measure_only 序列。
@note bag 里录的 /arm_joint_traj 就是上位机（task_runner / joint_calibrate）下发、随后由
      arm_controller 换算成 MIT 指令经 USB 交给 MCU 的那批关节数据；配 /joint_states
      可以事后对比「发了什么 / 实际到了哪」。实机没有仿真时间，bag 用系统时间戳，回放
      不需要 --clock。Ctrl+C 关 launch 时 rosbag2 收 SIGINT 正常收尾。
"""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue

# 录制的话题：上位机下发的关节轨迹指令 + 实际关节反馈。
# 需要别的模式时直接追加：arm_cart_traj（CartTrajCmd）、arm_servo（ServoCtrlCmd）、
# arm_admittance（AdmittanceCmd）。此刻还没有发布者的话题不会报错——rosbag2 会持续
# discovery，发布者一出现就自动订阅。
BAG_TOPICS = ["/arm_joint_traj", "/joint_states"]


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

    # ros2 bag 记录：默认开启，路径带启动时间戳，避免同名目录让 record 直接失败。
    record_bag_arg = DeclareLaunchArgument(
        "record_bag",
        default_value="true",
        description="是否用 ros2 bag 记录上位机下发的关节指令与 /joint_states",
    )
    bag_path_arg = DeclareLaunchArgument(
        "bag_path",
        default_value=os.path.join(
            os.path.expanduser("~/ros2_bag"),
            "arm_real_" + time.strftime("%Y%m%d_%H%M%S"),
        ),
        description="bag 输出目录（必须尚不存在，默认 ~/ros2_bag/arm_real_<时间戳>）",
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

    # 录制进程与 launch 同生命周期：Ctrl+C 时 ExecuteProcess 收到 SIGINT，rosbag2 会
    # 把缓存写完并落 metadata.yaml，不需要额外的收尾动作。
    # 实机没有仿真时间，不加 --use-sim-time，bag 用系统时间戳，回放不需要 --clock。
    # --disable-keyboard-controls：launch 下没有交互终端，别让它去抢按键/信号。
    bag_record = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            "bag",
            "record",
            "--output",
            LaunchConfiguration("bag_path"),
            "--topics",
            *BAG_TOPICS,
            "--disable-keyboard-controls",
        ],
        condition=IfCondition(LaunchConfiguration("record_bag")),
        output="screen",
    )

    bag_hint = LogInfo(
        msg=["[bag] 录制上位机关节指令到 ", LaunchConfiguration("bag_path")],
        condition=IfCondition(LaunchConfiguration("record_bag")),
    )

    return LaunchDescription(
        [
            loopback_arg,
            show_rviz_arg,
            controller_manager_timeout_arg,
            switch_timeout_arg,
            service_call_timeout_arg,
            record_bag_arg,
            bag_path_arg,
            loopback_hint,
            robot_state_pub,
            ros2_control_node,
            bag_hint,
            bag_record,
            load_joint_state_broadcaster,
            load_arm_controller,
            rviz2,
        ]
    )
