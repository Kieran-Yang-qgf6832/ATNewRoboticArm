"""仿真视觉链路：仿真相机 TF + 红色色块检测节点（只在 MuJoCo 仿真里用）。

实机请继续用 view.launch.py（RealSense），本文件不会启动任何真实硬件。

用法（三条命令，顺序无关，但仿真要先起来话题才有数据）：

    # 终端 1：用视觉场景启动仿真（scene_vision.xml 里有相机和红色箱子）
    ros2 launch launch_pack arm_controller_test_sim.launch.py \
        mujoco_model:=$(ros2 pkg prefix arm_model)/share/arm_model/model/scene_vision.xml

    # 终端 2：仿真相机 TF + 检测节点
    ros2 launch view view_sim.launch.py

    # 终端 3：数值验收（退出码 0 = 通过）
    ros2 run view check_sim_vision

相机安装位姿在整个工程里只有 scene_vision.xml 一个来源：本 launch 启动时直接解析
该文件的 <camera> 元素（pos + xyaxes/quat），再发布 base_link -> vision_camera_link，
因此渲染用的位姿和算法用的 TF 是同一串数字，不会漂移。

坐标系：MuJoCo 相机帧是 X 右、Y 上、光轴沿 -Z；ROS 光学帧是 X 右、Y 下、光轴沿 +Z。
两者只差一个绕 X 的 180°，所以第二段静态 TF 永久固定为 q=(1, 0, 0, 0)，不含任何
可调参数。检测节点只接受 optical 父帧（camera_axis=optical），父帧不对会直接禁用
目标 TF。

注意：本 launch 不启动仿真，也不持有时钟源。检测节点刻意关闭 use_sim_time，改用
stamp_from_image=true 直接拿图像的仿真时间戳发 TF——mujoco_ros2_control 的 /clock
是 500 Hz，rclpy 的 TimeSource 订阅它单这一项就吃掉约一个核，检测主循环会从 30 Hz
掉到 1 Hz 量级（见 detector.py 中 stamp_from_image 的注释）。
"""

from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 相机名必须与 scene_vision.xml 的 <camera name="..."> 一致
CAMERA_NAME = 'vision_camera'
BASE_FRAME = 'base_link'
LINK_FRAME = 'vision_camera_link'
OPTICAL_FRAME = 'vision_camera_optical_frame'

# 仿真话题；与 ros2_controller.yaml 里 vision_camera_plugin 的配置必须一致
COLOR_TOPIC = '/vision_camera/color'
INFO_TOPIC = '/vision_camera/camera_info'
DEPTH_TOPIC = '/vision_camera/depth'


def _normalized(vector):
    norm = math.sqrt(sum(value * value for value in vector))
    if norm < 1e-12:
        raise ValueError('零向量无法归一化')
    return [value / norm for value in vector]


def _cross(first, second):
    return [first[1] * second[2] - first[2] * second[1],
            first[2] * second[0] - first[0] * second[2],
            first[0] * second[1] - first[1] * second[0]]


def _quat_from_matrix(matrix):
    """旋转矩阵转 (x, y, z, w)，与 view.target.quaternion_from_rotation 同算法。"""
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = 2.0 * math.sqrt(trace + 1.0)
        quat = [(m21 - m12) / scale, (m02 - m20) / scale,
                (m10 - m01) / scale, 0.25 * scale]
    elif m00 > m11 and m00 > m22:
        scale = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        quat = [0.25 * scale, (m01 + m10) / scale,
                (m02 + m20) / scale, (m21 - m12) / scale]
    elif m11 > m22:
        scale = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        quat = [(m01 + m10) / scale, 0.25 * scale,
                (m12 + m21) / scale, (m02 - m20) / scale]
    else:
        scale = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
        quat = [(m02 + m20) / scale, (m12 + m21) / scale,
                0.25 * scale, (m10 - m01) / scale]
    quat = _normalized(quat)
    if quat[3] < 0.0:      # 统一 w >= 0，打印出来的 TF 数值更直观
        quat = [-value for value in quat]
    return quat


def _quat_from_xyaxes(x_axis, y_axis):
    """MuJoCo xyaxes -> 四元数。

    MuJoCo 的语义：前三个数是相机帧 X 轴（图像右），后三个数是 Y 轴（图像上），
    Z = X × Y 指向相机背后。这里按 MuJoCo 的方式重新正交化，保证与渲染一致。
    """
    x_axis = _normalized(x_axis)
    z_axis = _normalized(_cross(x_axis, y_axis))
    y_axis = _cross(z_axis, x_axis)
    matrix = [[x_axis[0], y_axis[0], z_axis[0]],
              [x_axis[1], y_axis[1], z_axis[1]],
              [x_axis[2], y_axis[2], z_axis[2]]]
    return _quat_from_matrix(matrix)


def _camera_pose(scene_path, name):
    """从 MJCF 里读出相机的 (pos, quat)；找不到或用了不支持的朝向写法就报错。"""
    root = ET.parse(scene_path).getroot()
    for camera in root.iter('camera'):
        if camera.get('name') != name:
            continue
        pos = [float(value) for value in camera.get('pos', '0 0 0').split()]
        if len(pos) != 3:
            raise ValueError('camera %s 的 pos 必须是 3 个数' % name)
        if camera.get('quat') is not None:
            quat = [float(value) for value in camera.get('quat').split()]
        elif camera.get('xyaxes') is not None:
            values = [float(value) for value in camera.get('xyaxes').split()]
            if len(values) != 6:
                raise ValueError('camera %s 的 xyaxes 必须是 6 个数' % name)
            quat = _quat_from_xyaxes(values[:3], values[3:])
        else:
            raise ValueError(
                'camera %s 需要显式写 quat 或 xyaxes；euler/axisangle/zaxis 未被本 launch '
                '支持，避免静默按单位朝向处理而发布错误的 TF' % name)
        if len(quat) != 4:
            raise ValueError('camera %s 的 quat 必须是 4 个数' % name)
        return pos, quat
    raise RuntimeError('在 %s 里找不到 camera name="%s"' % (scene_path, name))


def _launch_setup(context, arm_model_share):
    scene_path = LaunchConfiguration('scene_path').perform(context)
    position, rotation = _camera_pose(scene_path, CAMERA_NAME)

    # 第一段：MuJoCo 相机位姿，从场景文件原样搬运
    mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='vision_camera_mount_tf',
        arguments=[
            '--x', '%.9f' % position[0],
            '--y', '%.9f' % position[1],
            '--z', '%.9f' % position[2],
            '--qx', '%.9f' % rotation[0],
            '--qy', '%.9f' % rotation[1],
            '--qz', '%.9f' % rotation[2],
            '--qw', '%.9f' % rotation[3],
            '--frame-id', BASE_FRAME,
            '--child-frame-id', LINK_FRAME,
        ],
        output='screen',
    )

    # 第二段：MuJoCo 相机帧 -> ROS 光学帧，永久固定绕 X 转 180°
    optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='vision_camera_optical_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '1', '--qy', '0', '--qz', '0', '--qw', '0',
            '--frame-id', LINK_FRAME,
            '--child-frame-id', OPTICAL_FRAME,
        ],
        output='screen',
    )

    detector = Node(
        package='view',
        executable='red_quad_detector',
        name='red_quad_detector',
        output='screen',
        emulate_tty=True,
        parameters=[{
            # 刻意**不**用 use_sim_time：mujoco_ros2_control 以物理频率（500 Hz）
            # 发布 /clock，rclpy 的 TimeSource 订阅它仅这一项就占约一个核（实测
            # 空节点 106% CPU vs 1%），GIL 争用会把检测主循环从 30 Hz 压到 1.3 Hz。
            # 仿真时间由图像 header.stamp 提供，stamp_from_image=true 直接拿它发 TF。
            'use_sim_time': False,
            'stamp_from_image': True,
            # 相机插件是 depth=1 的 reliable 发布，best-effort 订阅会丢到 5~7 Hz，
            # 深度帧经常超过 max_depth_age，分面退化为单面（实测 mode=single）
            'image_qos': 'reliable',
            'color_topic': COLOR_TOPIC,
            'camera_info_topic': INFO_TOPIC,
            'aligned_depth_topic': DEPTH_TOPIC,
            'parent_frame_fallback': OPTICAL_FRAME,
            'display': LaunchConfiguration('display'),
        }],
    )

    return [mount_tf, optical_tf, detector]


def generate_launch_description():
    arm_model_share = get_package_share_directory('arm_model')

    scene_path_arg = DeclareLaunchArgument(
        'scene_path',
        default_value=os.path.join(arm_model_share, 'model', 'scene_vision.xml'),
        description='含 <camera name="vision_camera"> 的 MJCF 场景；相机 TF 直接从这里解析')

    display_arg = DeclareLaunchArgument(
        'display', default_value='true',
        description='是否开 OpenCV 预览窗口（无图形环境时检测与发布照常运行）')

    return LaunchDescription([
        scene_path_arg,
        display_arg,
        OpaqueFunction(function=_launch_setup, args=[arm_model_share]),
    ])
