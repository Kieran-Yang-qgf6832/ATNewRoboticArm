"""一键启动 RealSense 相机 + 红色色块实时检测节点。

    ros2 launch view view.launch.py

只启动检测节点（相机已在别处跑）：

    ros2 launch view view.launch.py start_camera:=false

启动时直接给阈值（不必再敲 param set）：

    ros2 launch view view.launch.py h_low:=165 h_high:=12 s_low:=120

运行中改阈值同样生效：

    ros2 param set /red_quad_detector h_low 165

关掉深度分面（只做单面 PnP）：

    ros2 launch view view.launch.py face_split:=false

目标面 TF 与 TargetFace 消息在无图形环境（display:=false）下照常发布：

    ros2 topic echo /target_face
    ros2 run tf2_ros tf2_echo <彩色 frame_id> target_face

父帧优先取彩色消息 header.frame_id，parent_frame_fallback 只是消息头为空时的兜底。
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
    rgb_profile = LaunchConfiguration('rgb_profile')
    depth_profile = LaunchConfiguration('depth_profile')
    align_depth = LaunchConfiguration('align_depth')
    start_camera = LaunchConfiguration('start_camera')
    start_detector = LaunchConfiguration('start_detector')
    display = LaunchConfiguration('display')
    face_split = LaunchConfiguration('face_split')

    declared_arguments = [
        DeclareLaunchArgument(
            'rgb_profile', default_value='848x480x30',
            description='彩色流 profile，格式 宽x高x帧率'),
        DeclareLaunchArgument(
            'depth_profile', default_value='848x480x30',
            description='深度流 profile，格式 宽x高x帧率'),
        DeclareLaunchArgument(
            'align_depth', default_value='true',
            description='发布对齐到彩色的深度话题（测距依赖它）'),
        DeclareLaunchArgument(
            'start_camera', default_value='true',
            description='是否同时启动 realsense2_camera'),
        DeclareLaunchArgument(
            'start_detector', default_value='true',
            description='是否同时启动红色色块检测节点'),
        DeclareLaunchArgument(
            'display', default_value='true',
            description='是否开 OpenCV 预览窗口'),
        DeclareLaunchArgument(
            'face_split', default_value='true',
            description='是否按深度分面（关闭则只做单面 PnP）'),
        DeclareLaunchArgument(
            'publish_target', default_value='true',
            description='是否发布目标面 TF 与 TargetFace 消息'),
        DeclareLaunchArgument(
            'parent_frame_fallback', default_value='camera_color_optical_frame',
            description='彩色消息头没有 frame_id 时的兜底父帧'),
        DeclareLaunchArgument(
            'child_frame_id', default_value='target_face',
            description='目标面 TF 的子帧名'),
        DeclareLaunchArgument(
            'target_topic', default_value='target_face',
            description='TargetFace 消息话题名'),
        DeclareLaunchArgument(
            'camera_axis', default_value='optical',
            description='父帧轴向约定，目前只支持 optical（X 右 Y 下 Z 前）'),
    ]

    # 参数名 -> (默认值, 说明, 类型)
    tunables = {
        'h_low': ('170', 'H 下限（H 用 OpenCV 原生 0~179）', int),
        'h_high': ('10', 'H 上限；小于下限即按色相环绕处理，红色就是 170~10', int),
        's_low': ('100', 'S 下限（0~255）', int),
        's_high': ('255', 'S 上限（0~255）', int),
        'v_low': ('70', 'V 下限（0~255）', int),
        'v_high': ('255', 'V 上限（0~255）', int),
        'min_area': ('300', '轮廓最小面积（像素），滤掉噪点', int),
        'morph_kernel': ('5', '形态学核尺寸，0 表示不做开闭运算', int),
        'square_size_mm': ('350.0', '色块边长（毫米），PnP 的目标尺寸', float),
        'publish_rate': ('30.0', '检测与发布频率（Hz）', float),
        'target_switch_frames': ('5', '候选面连续占优多少帧后切换锁定', int),
        'target_angle_margin_deg': ('20.0', '同一物理面的法线夹角容差（度）', float),
        'target_center_margin_mm': ('80.0', '同一物理面的中心距容差（毫米）', float),
        'target_incidence_margin_deg': ('5.0', '候选面比锁定记录差多少度以上就拒绝切换', float),
        'target_lost_timeout_ms': ('200', '检测失效后继续广播旧 TF 的时长（毫秒）', int),
        'max_stamp_drift_s': ('5.0', '图像时间戳与节点时钟的最大偏差（秒）', float),
    }
    for name, (default, description, _) in tunables.items():
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default, description=description))

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'
            ])
        ]),
        condition=IfCondition(start_camera),
        launch_arguments={
            'rgb_camera.color_profile': rgb_profile,
            'depth_module.depth_profile': depth_profile,
            'align_depth.enable': align_depth,
        }.items(),
    )

    parameters = {
        'display': ParameterValue(display, value_type=bool),
        'face_split': ParameterValue(face_split, value_type=bool),
        'publish_target': ParameterValue(LaunchConfiguration('publish_target'),
                                         value_type=bool),
        'parent_frame_fallback': LaunchConfiguration('parent_frame_fallback'),
        'child_frame_id': LaunchConfiguration('child_frame_id'),
        'target_topic': LaunchConfiguration('target_topic'),
        'camera_axis': LaunchConfiguration('camera_axis'),
    }
    for name, (_, _, value_type) in tunables.items():
        parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=value_type)

    detector = Node(
        package='view',
        executable='red_quad_detector',
        name='red_quad_detector',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(start_detector),
        parameters=[parameters],
    )

    return LaunchDescription(declared_arguments + [camera, detector])
