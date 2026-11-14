"""手眼标定一键启动：RealSense + ChArUco 检测 + easy_handeye2 采样界面。

    ros2 launch view calib.launch.py

相机已在别处启动：

    ros2 launch view calib.launch.py start_camera:=false

只起检测节点（自己敲 easy_handeye2 的命令）：

    ros2 launch view calib.launch.py start_handeye:=false

标定类型固定 eye_on_base：相机装在支架上俯视工作区，标靶装在末端 link6 上随机械臂走。
tracking_base_frame 用 camera_link 而不是光学帧——easy_handeye2 自带的 dummy 静态发布器
会给它挂上 base_link 父帧，而光学帧在 RealSense 驱动里已经有父帧，同一个子帧两个父帧
会让整条光学链的位姿出错。

采样时手动摆位（仓库里没有 MoveIt，easy_handeye2 的 handeye_server_robot 起不来）：

    ros2 run tasks task_runner --ros-args -p task_sequence:=joint_console_ui

摆好一个姿态在 rqt 里点 Take Sample，凑 15~20 个姿态覆盖不同平移与朝向。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_camera = LaunchConfiguration('start_camera')
    start_handeye = LaunchConfiguration('start_handeye')
    display = LaunchConfiguration('display')

    declared_arguments = [
        DeclareLaunchArgument('start_camera', default_value='true',
                              description='是否同时启动 realsense2_camera'),
        DeclareLaunchArgument('start_handeye', default_value='true',
                              description='是否同时启动 easy_handeye2 的采样界面'),
        DeclareLaunchArgument('rgb_profile', default_value='848x480x30',
                              description='彩色流 profile，格式 宽x高x帧率'),
        DeclareLaunchArgument('depth_profile', default_value='848x480x30',
                              description='深度流 profile，格式 宽x高x帧率'),
        DeclareLaunchArgument('align_depth', default_value='true',
                              description='发布对齐到彩色的深度话题'),
        DeclareLaunchArgument('display', default_value='true',
                              description='是否开 OpenCV 预览窗口（标定建议开）'),
        DeclareLaunchArgument('image_qos', default_value='sensor',
                              description='图像订阅 QoS：sensor 或 reliable'),
        DeclareLaunchArgument('name', default_value='arm5dof_realsense',
                              description='标定名，决定 .calib 文件名与 rqt 里的条目'),
        DeclareLaunchArgument('calibration_type', default_value='eye_on_base',
                              description='标定类型，本链路固定 eye_on_base'),
        DeclareLaunchArgument('tracking_base_frame', default_value='camera_link',
                              description='相机基准帧；不要填光学帧，会被驱动静态 TF 抢父帧'),
        DeclareLaunchArgument('tracking_marker_frame', default_value='calib_board',
                              description='标靶帧，必须与检测节点的 child_frame_id 一致'),
        DeclareLaunchArgument('robot_base_frame', default_value='base_link',
                              description='机器人基座帧'),
        DeclareLaunchArgument('robot_effector_frame', default_value='link6',
                              description='末端帧'),
        # 版面参数与 PDF 一致；换板时改这四个数即可
        DeclareLaunchArgument('squares_x', default_value='11',
                              description='横向格数'),
        DeclareLaunchArgument('squares_y', default_value='8',
                              description='纵向格数'),
        DeclareLaunchArgument('square_mm', default_value='15.0',
                              description='格边长（毫米）'),
        DeclareLaunchArgument('marker_mm', default_value='11.0',
                              description='marker 边长（毫米）'),
    ]

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'
            ])
        ]),
        condition=IfCondition(start_camera),
        launch_arguments={
            'rgb_camera.color_profile': LaunchConfiguration('rgb_profile'),
            'depth_module.depth_profile': LaunchConfiguration('depth_profile'),
            'align_depth.enable': LaunchConfiguration('align_depth'),
        }.items(),
    )

    detector = Node(
        package='view',
        executable='calib_board_detector',
        name='calib_board_detector',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'display': ParameterValue(display, value_type=bool),
            'image_qos': LaunchConfiguration('image_qos'),
            'child_frame_id': LaunchConfiguration('tracking_marker_frame'),
            'parent_frame_fallback': 'camera_color_optical_frame',
            'squares_x': ParameterValue(LaunchConfiguration('squares_x'), value_type=int),
            'squares_y': ParameterValue(LaunchConfiguration('squares_y'), value_type=int),
            'square_mm': ParameterValue(LaunchConfiguration('square_mm'), value_type=float),
            'marker_mm': ParameterValue(LaunchConfiguration('marker_mm'), value_type=float),
        }],
    )

    handeye = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('easy_handeye2'), 'launch', 'calibrate.launch.py'
            ])
        ]),
        condition=IfCondition(start_handeye),
        launch_arguments={
            'name': LaunchConfiguration('name'),
            'calibration_type': LaunchConfiguration('calibration_type'),
            'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
            'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
        }.items(),
    )

    return LaunchDescription(declared_arguments + [camera, detector, handeye])
