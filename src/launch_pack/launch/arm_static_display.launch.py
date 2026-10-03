"""
@file arm_static_display.launch.py
@brief 纯显示场景：用滑条手动摆关节角看 URDF 模型，不涉及 ros2_control 与仿真

启动内容：
  1. robot_state_publisher     ：读取 URDF 发布 /robot_description 与 TF
  2. joint_state_publisher_gui ：带滑条的 GUI，手动生成 /joint_states
  3. rviz2                     ：用 launch_pack/rviz/display_config.rviz 显示模型

用途：检查模型、关节轴向、连杆坐标系，或做 RViz 截图，不需要控制器和 MuJoCo。

@warning 状态：当前仓库不可用（包名过时）。本文件读取的是 arm 包的 URDF，而本仓库的模型包是
         arm_model（不存在名为 arm 的包），启动会抛 PackageNotFoundError；把
         get_package_share_directory("arm") 改成 "arm_model" 即可使用（此处仅注释说明）。
@note 本文件直接读取 URDF 文本、没有经过 xacro。本仓库的 robotic_arm.urdf 是纯 URDF
      （只额外带一个 ros2_control 扩展块），因此直接读取没有问题。
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 注意：本仓库的模型包是 arm_model，不存在名为 arm 的包，这里必须改成 "arm_model"。
    arm_share = get_package_share_directory("arm")
    launch_pack_share = get_package_share_directory("launch_pack")

    urdf_path = os.path.join(arm_share, "model", "robotic_arm.urdf")
    rviz_path = os.path.join(launch_pack_share, "rviz", "display_config.rviz")

    with open(urdf_path, "r", encoding="utf-8") as inf:
        robot_desc = inf.read()

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
    )

    joint_state_publish = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[{"use_gui": True}],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path],
    )

    return LaunchDescription([
        robot_state_pub,
        joint_state_publish,
        rviz2,
    ])
