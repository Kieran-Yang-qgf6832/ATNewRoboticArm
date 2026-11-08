"""联调占位 TF 链：base_link -> 相机光学帧 -> target_face。

手眼标定结果尚未产出时，用它验证 frame 名、时间戳、轴向与失效门禁：

    ros2 launch view view_tf_debug.launch.py
    ros2 topic echo /target_face
    ros2 run tf2_ros tf2_echo base_link target_face

这里的静态外参只是占位，绝不能用于真实抓取；正式部署应由
easy_handeye2 的 publish.launch.py 发布标定得到的静态变换。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_frame = LaunchConfiguration('camera_frame')
    parent_frame_fallback = LaunchConfiguration('parent_frame_fallback')
    child_frame_id = LaunchConfiguration('child_frame_id')
    target_topic = LaunchConfiguration('target_topic')

    declared_arguments = [
        DeclareLaunchArgument(
            'camera_frame', default_value='camera_color_optical_frame',
            description='占位静态变换的子帧，应与彩色消息 header.frame_id 一致'),
        DeclareLaunchArgument(
            'parent_frame_fallback', default_value='camera_color_optical_frame',
            description='彩色消息头没有 frame_id 时的兜底父帧'),
        DeclareLaunchArgument(
            'child_frame_id', default_value='target_face',
            description='目标面 TF 的子帧名'),
        DeclareLaunchArgument(
            'target_topic', default_value='target_face',
            description='TargetFace 消息话题名'),
        DeclareLaunchArgument('origin_x', default_value='0.5',
                              description='占位外参平移 x（米）'),
        DeclareLaunchArgument('origin_y', default_value='0.0',
                              description='占位外参平移 y（米）'),
        DeclareLaunchArgument('origin_z', default_value='0.6',
                              description='占位外参平移 z（米）'),
        DeclareLaunchArgument('camera_roll', default_value='1.5708',
                              description='占位外参 roll（弧度）'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0',
                              description='占位外参 pitch（弧度）'),
        DeclareLaunchArgument('camera_yaw', default_value='-1.5708',
                              description='占位外参 yaw（弧度）'),
    ]

    extrinsics = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='view_debug_extrinsics',
        arguments=[
            '--x', LaunchConfiguration('origin_x'),
            '--y', LaunchConfiguration('origin_y'),
            '--z', LaunchConfiguration('origin_z'),
            '--roll', LaunchConfiguration('camera_roll'),
            '--pitch', LaunchConfiguration('camera_pitch'),
            '--yaw', LaunchConfiguration('camera_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', camera_frame,
        ],
    )

    detector = Node(
        package='view',
        executable='red_quad_detector',
        name='red_quad_detector',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'display': False,
            'publish_target': True,
            'parent_frame_fallback': parent_frame_fallback,
            'child_frame_id': child_frame_id,
            'target_topic': target_topic,
        }],
    )

    return LaunchDescription(declared_arguments + [extrinsics, detector])
