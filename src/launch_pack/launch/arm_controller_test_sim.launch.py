"""
@file arm_controller_test_sim.launch.py
@brief 机械臂 MuJoCo 仿真主场景：拉起 ros2_control 控制链路与全部控制器（当前仓库推荐入口）

启动内容（按实际启动顺序）：
  1. robot_state_publisher  ：用 xacro 处理 arm_model/model/robotic_arm.urdf 后发布
                              /robot_description 与 TF（RViz 显示、tasks 查询末端位姿都依赖它）
  2. mujoco_ros2_control    ：ros2_control_node，MuJoCo 以 SystemInterface 插件形式运行在
                              controller_manager 进程内；硬件插件与 MJCF 路径写在 URDF 的
                              <ros2_control> 标签里（$(find arm_model)/model/scene.xml）
  3. joint_state_broadcaster：发布 /joint_states（tasks 的关节反馈来源）
  4. sim_pid_controller     ：链式 PID，把 arm_controller 的期望量转成 effort 送给 MuJoCo
  5. arm_controller         ：机械臂状态机控制器（idel/reset/cart_traj/joint_traj/servo/
                              admittance/teach_pendant/measure）；tasks 通过 exp_state 参数
                              与各轨迹话题驱动它
  6. rviz2                  ：由 show_rviz 控制是否启动

用法：
    ros2 launch launch_pack arm_controller_test_sim.launch.py
    ros2 launch launch_pack arm_controller_test_sim.launch.py show_rviz:=false
    # 起好之后另开终端跑任务链
    ros2 run tasks task_runner --ros-args -p use_sim_time:=true -p task_sequence:="[reset, cart_traj]"

@note 状态：可用。依赖（arm_model / arm_controller / mujoco_ros2_control / launch_pack）
      均在本工作区；控制器参数取自 config/ros2_controller.yaml。
@warning mujoco 进程退出会触发整条 launch 关闭（on_exit=Shutdown），RViz 也会一起退。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile


def generate_launch_description():
    arm_model_share = get_package_share_directory("arm_model")
    launch_pack_share = get_package_share_directory("launch_pack")

    urdf_path = os.path.join(arm_model_share, "model", "robotic_arm.urdf")
    controller_yaml = os.path.join(launch_pack_share, "config", "ros2_controller.yaml")
    rviz_path = os.path.join(launch_pack_share, "rviz", "display_config.rviz")

    # 通过 xacro 处理 URDF，使硬件标签里的 $(find arm_model) 被正确展开
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_path,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    show_rviz_arg = DeclareLaunchArgument(
        "show_rviz",
        default_value="true",
        description="Whether to start RViz2",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Whether nodes should use simulation time",
    )

    controller_manager_timeout_arg = DeclareLaunchArgument(
        "controller_manager_timeout",
        default_value="60",
        description="Seconds to wait for the controller manager services",
    )

    switch_timeout_arg = DeclareLaunchArgument(
        "switch_timeout",
        default_value="60",
        description="Seconds to wait for controller activation",
    )

    service_call_timeout_arg = DeclareLaunchArgument(
        "service_call_timeout",
        default_value="60",
        description="Seconds to wait for controller manager service responses",
    )

    # 用 xacro 展开 URDF 里的 $(find arm_model)：URDF 自身是纯 URDF，但 <ros2_control>
    # 硬件标签里写了 $(find arm_model)/model/scene.xml，不展开 controller_manager 会加载失败。
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    # 新版 mujoco_ros2_control：MuJoCo 仿真以 SystemInterface 插件形式
    # 运行在 controller_manager 进程内，硬件插件与 MJCF 路径在 URDF
    # 的 <ros2_control> 标签中声明。MJCF 场景：
    #   $(find arm_model)/model/scene.xml
    mujoco = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="both",
        parameters=[
            {"use_sim_time": use_sim_time},
            ParameterFile(controller_yaml),
        ],
        on_exit=Shutdown(),
    )

    # spawner 依赖 controller_manager 就绪，因此按「上一个 spawner 退出后再起下一个」串成链：
    # mujoco 启动 -> joint_state_broadcaster -> (sim_pid_controller + arm_controller)
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

    # --activate-as-group：两个控制器一次性激活，避免中间态——arm_controller 的命令接口
    # 是仿真 PID 控制器的 reference interface（command_interface_prefix 指定），需要它先就位。
    controller_chain = Node(
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
            "--activate-as-group",
            "sim_pid_controller",
            "arm_controller",
        ],
        output="screen",
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("show_rviz")),
        output="screen",
    )

    load_joint_state_broadcaster = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco,
            on_start=[
                LogInfo(msg="MuJoCo ros2_control node started, spawning joint_state_broadcaster"),
                joint_state_broadcaster,
            ],
        )
    )

    load_controller_chain = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[
                LogInfo(msg="joint_state_broadcaster spawned, spawning arm controller chain"),
                controller_chain,
            ],
        )
    )

    # 若已有一个仿真正在运行，本 launch 会再造一个 controller_manager，于是 /clock 与
    # /joint_states 各有两个发布者、时间基准互相回退，robot_state_publisher 会刷
    # "Moved backwards in time, re-publishing joint transforms!"，所以这里先给一句提示。
    double_sim_hint = LogInfo(
        msg="[提示] 本 launch 会启动 MuJoCo + 控制器；若已有仿真/控制器在运行，请改用 "
        "start_sim:=false 只跑任务，否则两个 /clock 与 /joint_states 发布者会造成时间戳回退刷屏。"
    )

    return LaunchDescription(
        [
            show_rviz_arg,
            use_sim_time_arg,
            controller_manager_timeout_arg,
            switch_timeout_arg,
            service_call_timeout_arg,
            double_sim_hint,
            robot_state_pub,
            mujoco,
            load_joint_state_broadcaster,
            load_controller_chain,
            rviz2,
        ]
    )
