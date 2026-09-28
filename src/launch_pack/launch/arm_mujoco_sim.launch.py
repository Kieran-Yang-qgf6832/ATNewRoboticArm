from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    arm_model_share = get_package_share_directory("arm_model")
    launch_pack_share = get_package_share_directory("launch_pack")

    controller_yaml = os.path.join(launch_pack_share, "config", "ros2_controller.yaml")
    rviz_path = os.path.join(launch_pack_share, "rviz", "display_config.rviz")

    # 通过 xacro 处理 URDF，使硬件标签里的 $(find arm_model) 被正确展开，
    # MJCF 场景由 URDF 的 <ros2_control> 硬件标签指定：
    #   $(find arm_model)/model/scene.xml
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            os.path.join(arm_model_share, "model", "robotic_arm.urdf"),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    show_rviz_arg = DeclareLaunchArgument(
        "show_rviz",
        default_value="true",
        description="Whether to start RViz2 together with MuJoCo simulation",
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # 新版 mujoco_ros2_control：MuJoCo 以 SystemInterface 插件运行在
    # controller_manager 进程内，由 URDF 的 <ros2_control> 标签加载。
    mujoco = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="both",
        parameters=[
            {"use_sim_time": True},
            ParameterFile(controller_yaml),
        ],
        on_exit=Shutdown(),
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    sim_pid_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["sim_pid_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_calc = Node(
        package="arm_calc",
        executable="arm_calc",
        output="screen",
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path],
        condition=IfCondition(LaunchConfiguration("show_rviz")),
    )

    load_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco,
            on_start=[
                LogInfo(msg="MuJoCo ros2_control node started, spawning joint_state_broadcaster"),
                joint_state_broadcaster,
            ],
        )
    )

    load_sim_pid_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[
                LogInfo(msg="joint_state_broadcaster spawned, spawning sim_pid_controller"),
                sim_pid_controller,
            ],
        )
    )

    load_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=sim_pid_controller,
            on_exit=[
                LogInfo(msg="sim_pid_controller spawned, spawning arm_controller"),
                arm_controller,
            ],
        )
    )

    return LaunchDescription([
        show_rviz_arg,
        robot_state_pub,
        mujoco,
        load_controller,
        load_sim_pid_controller,
        load_arm_controller,
        arm_calc,
        rviz2,
    ])
