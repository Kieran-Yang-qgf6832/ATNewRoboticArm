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

    return LaunchDescription(
        [
            show_rviz_arg,
            use_sim_time_arg,
            controller_manager_timeout_arg,
            switch_timeout_arg,
            service_call_timeout_arg,
            robot_state_pub,
            mujoco,
            load_joint_state_broadcaster,
            load_controller_chain,
            rviz2,
        ]
    )
