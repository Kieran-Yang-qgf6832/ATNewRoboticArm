"""
@file arm_real.launch.py
@brief 实机链路（旧版，当前仓库已不完整）

启动内容：
  1. robot_state_publisher     ：读取 arm 包的 URDF 发布 /robot_description 与 TF
  2. arm_calc                  ：计算节点
  3. arm_task                  ：任务节点
  4. robot_driver              ：实机驱动（与 MCU 通信）
  5. vision                    ：视觉节点
  6. static_transform_publisher：link4 -> camera_link 的固定变换
  7. rviz2                     ：显示配置 config/display_config.rviz

@warning 状态：当前仓库不可用。arm / arm_calc / arm_task / robot_driver / vision 五个包都不在
         本工作区（src/）中，启动会因找不到可执行文件而失败。当前的实机路径应由：
             - arm_controller 的 ArmRealInterfaces（hardware_interface 插件，描述见
               arm_real_plugin.xml）承担与下位机的通信
             - tasks 的 task_runner 承担任务调度
         待相关包补齐后再复用本文件。
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
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
        output="screen",
    )

    # 注意：arm_calc / arm_task / robot_driver / vision 四个包都不在本工作区，
    # 加上上面读取的 "arm" 包也不存在，因此本文件目前无法启动。
    arm_calc = Node(
        package="arm_calc",
        executable="arm_calc",
        output="screen",
    )

    arm_task = Node(
        package="arm_task",
        executable="arm_task",
        output="screen",
    )

    arm_driver = Node(
        package="robot_driver",
        executable="robot_driver",
        output="screen",
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path],
    )

    vision=Node(package="vision",
        executable="vision_node",
        output="screen",
    )

    static_tf_camera = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=[
        "0.1", "0.09", "-0.03",
        "0.0", "0.7071068", "0.0", "0.7071068",
        "link4",
        "camera_link"
    ],
    output="screen",
)

    return LaunchDescription([
        arm_driver,
        robot_state_pub,
        arm_calc,
        rviz2,
        static_tf_camera,
        vision,
        arm_task
    ])
