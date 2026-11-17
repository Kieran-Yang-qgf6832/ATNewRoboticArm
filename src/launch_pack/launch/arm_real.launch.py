"""
@file arm_real.launch.py
@brief 实机链路（控制器层）：USB CDC 硬件接口 + ArmController + 状态广播

启动内容：
  1. robot_state_publisher  ：xacro 展开 arm_model/model_4dof/robotic_arm.urdf
                              （hardware:=real，4 关节）后发布 /arm_robot_description 与 TF
  2. ros2_control_node      ：controller_manager + ArmRealInterfaces 硬件插件
  3. joint_state_broadcaster：发布 /joint_states
  4. arm_controller         ：状态机控制器，命令直接写 MCU（固件自己的位置/速度双环）
  5. rosbag2                ：录制关节命令与关节状态（record_bag:=false 关闭），
                              见 arm_bag_record.launch.py
  6. rviz2                  ：show_rviz:=true 时启动

参数：
  loopback:=true     不打开 USB 设备，把指令读回状态，用于无硬件冒烟（默认 false）
  show_rviz:=true    启动 RViz（默认 false）
  record_bag:=false  不录 rosbag（默认 true，目录见 bag_dir）
  bag_dir:=...       rosbag 输出根目录（默认 <项目根>/.ros/ros2bags）

用法：
  # 无硬件冒烟
  ros2 launch launch_pack arm_real.launch.py loopback:=true
  # 接真机
  ros2 launch launch_pack arm_real.launch.py
  # 真机但不录 bag
  ros2 launch launch_pack arm_real.launch.py record_bag:=false

@note 任务层不在这里启动。task_runner 需要 TF 与 /joint_states，确认控制器激活后
      再单独启动。
@note 当前阶段只验收**关节控制**（idel / joint_traj / joint_move）：4 轴实机的
      笛卡尔链路（pos_move / cart_traj）尚未做闭环验证，先不要用。
@note 默认录制 /joint_states（下位机回报）与 /arm_controller/joint_command（上位机下发的
     每周期命令）。实机上 bag 的时间戳是系统时间，与消息 header.stamp 一致。
@warning 实机不支持 measure：MCU 不上报加速度，acc_* 列恒 0，辨识结果不可用。
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def _find_stale_controller_manager():
    """检查同一 domain 里是否已有 /controller_manager。

    上一次 launch 的 ros2_control_node 没退出时，两个同名进程会同时应答
    /controller_manager 的服务：spawner 的 list_controllers 被旧进程接走（报
    "Controller already loaded"），configure 落到新进程（报 "no controller with
    this name exists"），两个 spawner 都以 1 退出——而 launch 照旧启动 RViz，
    看起来像成功了。这里在启动前拦住。

    @return 失败原因；没有冲突时返回 None。
    """
    try:
        result = subprocess.run(
            ["ros2", "node", "list", "--no-daemon"],
            capture_output=True,
            text=True,
            timeout=8.0,
            env=os.environ,
        )
    except (OSError, subprocess.SubprocessError) as error:
        return f"无法查询已有 ROS 节点（{error}），跳过残留检查"

    if any(line.strip() == "/controller_manager" for line in result.stdout.splitlines()):
        return (
            "检测到已有 /controller_manager 在运行（上一次 launch 的 ros2_control_node "
            "可能没有退出）。同名进程会让 spawner 在「已加载」与「不存在」之间错乱并静默失败。"
            "先执行 `ros2 node list`，确认没有残留后重试。"
        )
    return None


def generate_launch_description():
    stale_manager = _find_stale_controller_manager()
    if stale_manager is not None:
        # 不启动任何节点：宁可拒绝启动，也不要让用户在不知道的情况下跑在别人的控制器上。
        return LaunchDescription([LogInfo(msg=f"[拒绝启动] {stale_manager}"), Shutdown(reason=stale_manager)])

    arm_model_share = get_package_share_directory("arm_model")
    launch_pack_share = get_package_share_directory("launch_pack")

    urdf_path = os.path.join(arm_model_share, "model_4dof", "robotic_arm.urdf")
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
    record_bag_arg = DeclareLaunchArgument(
        "record_bag",
        default_value="true",
        description="是否用 rosbag2 录制关节命令与关节状态（实现见 arm_bag_record.launch.py）",
    )
    bag_dir_arg = DeclareLaunchArgument(
        "bag_dir",
        default_value="",
        description="rosbag 输出根目录；空 = <项目根>/.ros/ros2bags（目录名再拼 arm_real_<时间戳>）",
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

    # 关节命令/状态录制：与 controller_manager 同时启动。rosbag2 对指定话题先订阅再等数据，
    # 话题暂时不存在不会报错，因此控制器激活后的自动复位运动也能录到。
    bag_record = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_pack_share, "launch", "arm_bag_record.launch.py")),
        launch_arguments={
            "record_bag": LaunchConfiguration("record_bag"),
            "bag_dir": LaunchConfiguration("bag_dir"),
            "bag_name": "arm_real",
        }.items(),
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

    def on_joint_state_broadcaster_exit(event, context):
        """joint_state_broadcaster spawner 退出：失败就整体停机，不要继续起 RViz。"""
        del context
        if event.returncode != 0:
            return [
                LogInfo(
                    msg="[失败] joint_state_broadcaster 没有加载/配置成功（spawner 退出码 "
                    f"{event.returncode}）。若日志里是 “already loaded” + “no controller with this name”，"
                    "说明还有残留的 /controller_manager 在应答，先把旧进程停掉。"
                ),
                Shutdown(reason="joint_state_broadcaster spawner failed"),
            ]
        return [
            LogInfo(msg="joint_state_broadcaster spawned, spawning arm_controller"),
            arm_controller,
        ]

    def on_arm_controller_exit(event, context):
        """arm_controller spawner 退出：失败同样停机，避免“launch 在跑但控制器没激活”。"""
        del context
        if event.returncode != 0:
            return [
                LogInfo(
                    msg="[失败] arm_controller 没有激活（spawner 退出码 "
                    f"{event.returncode}），本次 launch 不提供任何控制能力，已停止。"
                ),
                Shutdown(reason="arm_controller spawner failed"),
            ]
        return [LogInfo(msg="arm_controller 已激活")]

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
        OnProcessExit(target_action=joint_state_broadcaster, on_exit=on_joint_state_broadcaster_exit)
    )

    check_arm_controller = RegisterEventHandler(
        OnProcessExit(target_action=arm_controller, on_exit=on_arm_controller_exit)
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
            record_bag_arg,
            bag_dir_arg,
            loopback_hint,
            robot_state_pub,
            ros2_control_node,
            bag_record,
            load_joint_state_broadcaster,
            load_arm_controller,
            check_arm_controller,
            rviz2,
        ]
    )
