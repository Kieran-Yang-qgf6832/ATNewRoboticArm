"""发布手眼标定得到的相机外参：`base_link -> camera_link` 静态 TF。

    ros2 launch launch_pack handeye.launch.py

标定文件默认取工作区里的 `.ros/easy_handeye2/<name>.calib`（easy_handeye2 自己只会写
`~/.ros2`，标定完需要手动导一份到项目里，见 docs/execute/09-手眼标定.md）。
也可以显式指定：

    ros2 launch launch_pack handeye.launch.py calib_file:=/path/to/xxx.calib

帧名与标定文件里的 `parameters` 段保持一致：`robot_base_frame` 作父帧、
`tracking_base_frame` 作子帧；显式给了 base_frame / camera_frame 时以参数为准。

为什么不直接用 easy_handeye2 的 publish.launch.py：它的 `handeye_publisher` 只接 `name`，
路径硬编码在 `~/.ros2/easy_handeye2/calibrations`，读不了项目里的副本。
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch_ros.actions import Node


def _default_calib_file(name: str) -> Path | None:
    """工作区根目录下的 `.ros/easy_handeye2/<name>.calib`；找不到就返回 None。"""
    candidates = []
    try:
        # <ws>/install/launch_pack -> <ws>
        candidates.append(Path(get_package_prefix('launch_pack')).parents[1])
    except Exception:
        pass
    candidates.append(Path.cwd())
    for root in candidates:
        path = root / '.ros' / 'easy_handeye2' / (name + '.calib')
        if path.is_file():
            return path
    return None


def _load_calibration(path: Path):
    with open(path, 'r') as handle:
        content = yaml.safe_load(handle.read())
    if not isinstance(content, dict) or 'transform' not in content:
        raise RuntimeError('%s 不是 easy_handeye2 的标定文件（缺 transform 段）' % path)
    transform = content['transform']
    translation = transform.get('translation') or {}
    rotation = transform.get('rotation') or {}
    for axis in ('x', 'y', 'z'):
        if axis not in translation:
            raise RuntimeError('%s 的 transform.translation 缺 %s' % (path, axis))
    for axis in ('x', 'y', 'z', 'w'):
        if axis not in rotation:
            raise RuntimeError('%s 的 transform.rotation 缺 %s' % (path, axis))
    return translation, rotation, content.get('parameters') or {}


def _make_nodes(context):
    """参数要到运行期才能读到命令行覆盖，所以节点在这里构造。"""
    values = context.launch_configurations
    name = values.get('name') or 'arm5dof_realsense'
    calib_file = (values.get('calib_file') or '').strip()

    resolved = Path(calib_file) if calib_file else _default_calib_file(name)
    if resolved is None or not resolved.is_file():
        where = resolved if resolved else '<工作区>/.ros/easy_handeye2/%s.calib' % name
        raise RuntimeError(
            '没找到标定文件：%s\n'
            '请先完成标定并导出到工作区的 .ros/easy_handeye2/<name>.calib，'
            '或用 calib_file:= 指定完整路径' % where)

    translation, rotation, parameters = _load_calibration(resolved)
    parent = (values.get('base_frame') or parameters.get('robot_base_frame')
              or 'base_link')
    child = (values.get('camera_frame') or parameters.get('tracking_base_frame')
             or 'camera_link')
    calibration_type = parameters.get('calibration_type') or '?'

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='handeye_static_tf',
        output='screen',
        arguments=[
            '--x', str(translation['x']),
            '--y', str(translation['y']),
            '--z', str(translation['z']),
            '--qx', str(rotation['x']),
            '--qy', str(rotation['y']),
            '--qz', str(rotation['z']),
            '--qw', str(rotation['w']),
            '--frame-id', parent,
            '--child-frame-id', child,
        ],
    )

    return [
        LogInfo(msg='手眼标定: %s（%s）' % (resolved, calibration_type)),
        LogInfo(msg='静态 TF: %s -> %s' % (parent, child)),
        static_tf,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='arm5dof_realsense',
                              description='标定名，用于定位 .ros/easy_handeye2/<name>.calib'),
        DeclareLaunchArgument('calib_file', default_value='',
                              description='标定文件完整路径；留空则按工作区默认位置查找'),
        DeclareLaunchArgument('base_frame', default_value='',
                              description='父帧；留空则取标定文件里的 robot_base_frame'),
        DeclareLaunchArgument('camera_frame', default_value='',
                              description='子帧；留空则取标定文件里的 tracking_base_frame'),
        OpaqueFunction(function=_make_nodes),
    ])
