"""检测参数：字段定义、范围校验与不可变配置快照。

本模块刻意不依赖 rclpy，节点的参数声明与校验回调都基于这里的 FIELDS 生成，
因此参数规则可以脱离 ROS 单独测试。
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any, Mapping


class ParameterError(ValueError):
    """参数值不合法。"""


@dataclass(frozen=True)
class FieldSpec:
    """一个可热更新参数的元信息。"""

    default: Any
    description: str
    kind: type
    minimum: float | None = None
    maximum: float | None = None
    odd: bool = False


# 参数名 -> 元信息。节点按这张表 declare，校验也按这张表走，避免两处各写一份。
FIELDS: dict[str, FieldSpec] = {
    'h_low': FieldSpec(170, 'H 下限（OpenCV 原生 0~179）', int, 0, 179),
    'h_high': FieldSpec(10, 'H 上限；小于下限即按色相环绕处理，红色就是 170~10', int, 0, 179),
    's_low': FieldSpec(100, 'S 下限（0~255）', int, 0, 255),
    's_high': FieldSpec(255, 'S 上限（0~255）', int, 0, 255),
    'v_low': FieldSpec(70, 'V 下限（0~255）', int, 0, 255),
    'v_high': FieldSpec(255, 'V 上限（0~255）', int, 0, 255),
    # 面积以像素整数计；声明成 float 会让 launch 传来的 INTEGER 覆盖被 rclpy 拒绝
    'min_area': FieldSpec(300, '轮廓最小面积（像素），滤掉噪点', int, 1),
    'morph_kernel': FieldSpec(5, '形态学核尺寸（奇数），0 表示不做开闭运算', int, 0, 99, True),
    'depth_window': FieldSpec(5, '测距取样窗口边长（像素）', int, 1, 99),
    'square_size_mm': FieldSpec(350.0, '色块边长（毫米），PnP 的目标尺寸', float, 1.0, 10000.0),
    'display': FieldSpec(True, '是否开 OpenCV 预览窗口', bool),
    'display_max_width': FieldSpec(1280, '预览窗口最大宽度，0 表示不限制', int, 0),
    # 以下为深度分面：把红色区域内点云按平面切开，两个相邻面合出箱体位姿
    'face_split': FieldSpec(True, '是否按深度分面（关闭则退化为只做单面 PnP）', bool),
    'plane_distance_mm': FieldSpec(8.0, 'RANSAC 内点距离阈值（毫米）', float, 1.0, 100.0),
    'plane_iterations': FieldSpec(100, 'RANSAC 迭代次数', int, 10, 5000),
    'plane_min_points': FieldSpec(600, '单个平面最少内点数', int, 50, 100000),
    'plane_max': FieldSpec(4, '最多提取的平面数（三面时第三面常排在碎面之后）', int, 2, 6),
    'plane_points': FieldSpec(3000, '参与平面拟合的降采样点数', int, 200, 50000),
    'face_pair_angle_deg': FieldSpec(12.0, '两面夹角与 90° 的容差', float, 1.0, 45.0),
    'face_size_tolerance_pct': FieldSpec(15.0, '第三面尺寸与已知尺寸的最大偏差（%）', float, 1.0, 100.0),
    'face_max_incidence_deg': FieldSpec(65.0, '第三面允许的最大入射角，超过则不参与融合', float, 1.0, 89.0),
    'box_size_x_mm': FieldSpec(0.0, '箱体沿共用棱的边长，0 表示按测量', float, 0.0),
    'box_size_y_mm': FieldSpec(0.0, '箱体沿面 1 法线的边长，0 表示按测量', float, 0.0),
    'box_size_z_mm': FieldSpec(0.0, '箱体沿面 2 法线的边长，0 表示按测量', float, 0.0),
    # 以下为目标面 TF / TargetFace 发布：帧名与话题名是启动时固定的字符串参数，
    # 不进 FIELDS（本表只校验数值与布尔），由节点单独 declare。
    'publish_target': FieldSpec(True, '是否发布目标面 TF 与 TargetFace 消息', bool),
    'publish_rate': FieldSpec(30.0, '主循环检测与发布频率（Hz）', float, 1.0, 120.0),
    'target_switch_frames': FieldSpec(5, '候选面连续占优多少帧后才切换锁定', int, 1, 60),
    'target_angle_margin_deg': FieldSpec(20.0, '判定同一物理面的法线夹角容差（度）', float, 1.0, 90.0),
    'target_center_margin_mm': FieldSpec(80.0, '判定同一物理面的中心距容差（毫米）', float, 0.0, 500.0),
    'target_incidence_margin_deg': FieldSpec(5.0, '候选面比锁定记录差多少度以上就拒绝切换（度）', float, 0.0, 45.0),
    'target_lost_timeout_ms': FieldSpec(200, '检测失效后继续广播旧 TF 的时长（毫秒）', int, 0, 5000),
    'max_stamp_drift_s': FieldSpec(5.0, '图像时间戳与节点时钟的最大允许偏差（秒）', float, 0.1, 60.0),
}


@dataclass(frozen=True)
class DetectorConfig:
    """一帧检测所需的全部参数；替换整个对象即原子生效。"""

    h_low: int = 170
    h_high: int = 10
    s_low: int = 100
    s_high: int = 255
    v_low: int = 70
    v_high: int = 255
    min_area: int = 300
    morph_kernel: int = 5
    depth_window: int = 5
    square_size_mm: float = 350.0
    display: bool = True
    display_max_width: int = 1280
    face_split: bool = True
    plane_distance_mm: float = 8.0
    plane_iterations: int = 100
    plane_min_points: int = 600
    plane_max: int = 4
    plane_points: int = 3000
    face_pair_angle_deg: float = 12.0
    face_size_tolerance_pct: float = 15.0
    face_max_incidence_deg: float = 65.0
    box_size_x_mm: float = 0.0
    box_size_y_mm: float = 0.0
    box_size_z_mm: float = 0.0
    publish_target: bool = True
    publish_rate: float = 30.0
    target_switch_frames: int = 5
    target_angle_margin_deg: float = 20.0
    target_center_margin_mm: float = 80.0
    target_incidence_margin_deg: float = 5.0
    target_lost_timeout_ms: int = 200
    max_stamp_drift_s: float = 5.0


def validate(name: str, value: Any) -> Any:
    """校验单个参数并归一化类型，非法时抛 ParameterError。"""
    spec = FIELDS.get(name)
    if spec is None:
        raise ParameterError('未知参数: %s' % name)
    if spec.kind is bool:
        if not isinstance(value, bool):
            raise ParameterError('%s 需要布尔值，收到 %r' % (name, value))
        return value
    if spec.kind is int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ParameterError('%s 需要整数，收到 %r' % (name, value))
    else:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ParameterError('%s 需要数值，收到 %r' % (name, value))
        value = float(value)
    if spec.minimum is not None and value < spec.minimum:
        raise ParameterError('%s 不能小于 %s，收到 %r' % (name, spec.minimum, value))
    if spec.maximum is not None and value > spec.maximum:
        raise ParameterError('%s 不能大于 %s，收到 %r' % (name, spec.maximum, value))
    # 0 是各处的“关闭”约定，其余偶数核会让形态学中心偏移
    if spec.odd and value != 0 and value % 2 == 0:
        raise ParameterError('%s 必须是奇数（或 0 表示关闭），收到 %r' % (name, value))
    return value


def config_from_values(values: Mapping[str, Any]) -> tuple[DetectorConfig, list[str]]:
    """启动时按需构造配置；非法项回落默认值并返回说明，避免节点起不来。"""
    accepted: dict[str, Any] = {}
    problems: list[str] = []
    for name, spec in FIELDS.items():
        try:
            accepted[name] = validate(name, values.get(name, spec.default))
        except ParameterError as exc:
            problems.append(str(exc))
            accepted[name] = spec.default
    return DetectorConfig(**accepted), problems


def apply_overrides(config: DetectorConfig, changes: Mapping[str, Any]) -> DetectorConfig:
    """整批校验后生成新配置；任一项非法则整体不生效（抛 ParameterError）。"""
    updates: dict[str, Any] = {}
    for name, value in changes.items():
        if name not in FIELDS:
            continue
        updates[name] = validate(name, value)
    return replace(config, **updates) if updates else config


def describe(config: DetectorConfig) -> str:
    return ('HSV H %d~%d S %d~%d V %d~%d | 最小面积 %d px | 核 %d | '
            '测距窗 %d | 板边长 %.0f mm | 窗口宽度上限 %d | 深度分面 %s'
            '（%d 点 / 阈值 %.0f mm / %d 次迭代 / 夹角容差 %.0f°）' % (
                config.h_low, config.h_high, config.s_low, config.s_high,
                config.v_low, config.v_high, config.min_area,
                config.morph_kernel, config.depth_window,
                config.square_size_mm, config.display_max_width,
                '开' if config.face_split else '关',
                config.plane_points, config.plane_distance_mm,
                config.plane_iterations, config.face_pair_angle_deg)
            + (' | 目标发布 %s（%.0f Hz / 切换 %d 帧 / 失效 %d ms）' % (
                '开' if config.publish_target else '关',
                config.publish_rate, config.target_switch_frames,
                config.target_lost_timeout_ms)))
