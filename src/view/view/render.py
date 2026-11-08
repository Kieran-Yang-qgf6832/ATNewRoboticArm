"""把检测结果画到图像上。

这里全是纯绘制，不依赖 ROS，也不碰窗口生命周期：组合出一张画布交给节点去 imshow。

标签分两类：
- 面身份角标（#0 / F1）：贴在各面左上角，只写身份，用来把画面上的面和数据对上号。
- 数据块：所有数值堆进画布左上角的半透明块里，不再压住画面中心的目标。
"""

from __future__ import annotations

import cv2
import numpy as np

from view import planes
from view.target import TargetSample
from view.vision import CameraModel, Detection

WINDOW_TITLE = 'red quad detector  [q] quit'

_FONT = cv2.FONT_HERSHEY_SIMPLEX
_FONT_SCALE = 0.5
_FONT_THICKNESS = 1

# 目标/面的配色：边框、角标、数据段首三处同色，超过四项后循环
TARGET_COLORS = ((0, 255, 0), (255, 160, 0), (255, 255, 0), (0, 200, 255))
# X 蓝、Y 黄、Z 品红：三种颜色在红板上都能看清
AXIS_COLORS = ((255, 0, 0), (0, 255, 255), (255, 0, 255))
MODE_COLOR = (0, 255, 255)
DATA_COLOR = (215, 215, 215)

# 数据块贴着画布左上角；半透明是为了不把画面那一角彻底封死
BLOCK_ORIGIN = (8, 8)
BLOCK_ALPHA = 0.55
BLOCK_PADDING = 5


def _project(camera: CameraModel, points: np.ndarray) -> np.ndarray:
    """相机系下的三维点投到像素，返回 (N, 2) int。"""
    rvec = np.zeros(3)
    tvec = np.zeros(3)
    projected, _ = cv2.projectPoints(np.asarray(points, dtype=np.float64),
                                      rvec, tvec, camera.matrix,
                                      camera.dist_coeffs)
    return np.round(projected.reshape(-1, 2)).astype(int)


def draw_axes(canvas: np.ndarray, camera: CameraModel, rotation: np.ndarray,
              origin: np.ndarray, length: float) -> None:
    """画一个坐标系的三根轴，箭头指向该轴的正方向。

    不用 cv2.drawFrameAxes 是因为它把 X 画成红色，而目标本身就是红色色块，
    红线画在红板上等于没画；顺带也自己控制箭头形状。
    """
    rotation = np.asarray(rotation, dtype=np.float64)
    origin = np.asarray(origin, dtype=np.float64).reshape(3)
    axis_points = np.array([[0.0, 0.0, 0.0],
                            [length, 0.0, 0.0],
                            [0.0, length, 0.0],
                            [0.0, 0.0, length]], dtype=np.float64)
    pts = _project(camera, axis_points @ rotation.T + origin)
    start = (int(pts[0][0]), int(pts[0][1]))
    for end, color in zip(pts[1:], AXIS_COLORS):
        tip = (int(end[0]), int(end[1]))
        # 板接近侧对相机时某根轴投影后只有几个像素，箭头会糊成一团，退化成短线
        span = ((tip[0] - start[0]) ** 2 + (tip[1] - start[1]) ** 2) ** 0.5
        if span < 10.0:
            cv2.line(canvas, start, tip, color, 2, cv2.LINE_AA)
        else:
            cv2.arrowedLine(canvas, start, tip, color, 2, cv2.LINE_AA,
                            tipLength=0.25)
    cv2.circle(canvas, start, 3, (255, 255, 255), -1, cv2.LINE_AA)


def _text_size(text: str) -> tuple[int, int]:
    return cv2.getTextSize(text, _FONT, _FONT_SCALE, _FONT_THICKNESS)[0]


def _top_left_corner(points: np.ndarray) -> np.ndarray:
    """投影四角中图像坐标 x+y 最小的那个，也就是最靠左上的顶点。

    不用「先比 x 再比 y」：四边形明显倾斜时最左点落在左下角，标签会被拽到面底部。
    相邻两面共棱时两者的这个角本来就重合，改由 draw_corner_tag 的避让集合错开。
    """
    corners = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    return corners[int(np.argmin(corners.sum(axis=1)))]


def _overlaps(first: tuple[int, int, int, int],
              second: tuple[int, int, int, int]) -> bool:
    """两个像素矩形是否相交。"""
    return (first[0] < second[2] and first[2] > second[0]
            and first[1] < second[3] and first[3] > second[1])


def draw_corner_tag(canvas: np.ndarray, text: str, anchor: np.ndarray,
                    color: tuple[int, int, int],
                    blockers: tuple[tuple[int, int, int, int], ...] = (),
                    ) -> tuple[int, int, int, int]:
    """在面左上角外挂一个小角标，只写身份（#0 / F1）。

    数值一律进数据块，角标本身保持最小，避免压住面。blockers 是已经被占用的
    矩形（数据块、先画好的角标）：撞上就整体下移一行，否则目标落在画面左上角
    时会被数据块盖掉，相邻两面共用的那个角也会让两个角标叠在一起。
    """
    width, height = _text_size(text)
    x = int(np.clip(round(float(anchor[0])) - 2, 2,
                    max(2, canvas.shape[1] - width - 8)))
    baseline = int(np.clip(round(float(anchor[1])) - 4, height + 4,
                           canvas.shape[0] - 4))
    rect = (x - 4, baseline - height - 4, x + width + 4, baseline + 4)
    for _ in range(len(blockers) + 1):
        hit = next((blocker for blocker in blockers
                    if _overlaps(rect, blocker)), None)
        if hit is None:
            break
        baseline = min(canvas.shape[0] - 4, hit[3] + height + 10)
        rect = (x - 4, baseline - height - 4, x + width + 4, baseline + 4)
    cv2.rectangle(canvas, (rect[0], rect[1]), (rect[2], rect[3]),
                  (0, 0, 0), -1)
    cv2.putText(canvas, text, (x, baseline), _FONT, _FONT_SCALE, color,
                _FONT_THICKNESS, cv2.LINE_AA)
    return rect


def block_rect(canvas: np.ndarray,
               lines: list[tuple[str, tuple[int, int, int]]],
               origin: tuple[int, int] = BLOCK_ORIGIN,
               ) -> tuple[int, int, int, int] | None:
    """数据块将要占用的像素矩形 (x0, y0, x1, y1)；角标用它避让。

    宽高只取决于文本内容，因此可以在绘制之前先算出来。
    """
    if not lines:
        return None
    sizes = [_text_size(text) for text, _ in lines]
    height = max(size[1] for size in sizes)
    step = height + 5
    width = max(size[0] for size in sizes)
    box_w = width + 2 * BLOCK_PADDING
    box_h = BLOCK_PADDING + height + step * (len(lines) - 1) + 4

    x0 = int(np.clip(origin[0], 0, max(0, canvas.shape[1] - box_w)))
    y0 = int(np.clip(origin[1], 0, max(0, canvas.shape[0] - box_h)))
    x1 = min(canvas.shape[1], x0 + box_w)
    y1 = min(canvas.shape[0], y0 + box_h)
    return None if x1 <= x0 or y1 <= y0 else (x0, y0, x1, y1)


def draw_data_block(canvas: np.ndarray,
                    lines: list[tuple[str, tuple[int, int, int]]],
                    origin: tuple[int, int] = BLOCK_ORIGIN,
                    alpha: float = BLOCK_ALPHA) -> None:
    """把若干行 (文本, 颜色) 画成画布左上角的一整块半透明标签。

    行按顺序紧排、段间不插空行：段与段靠段首的编号区分，段内行缩进对齐。
    """
    rect = block_rect(canvas, lines, origin)
    if rect is None:
        return
    height = max(_text_size(text)[1] for text, _ in lines)
    step = height + 5
    x0, y0, x1, y1 = rect

    # 和黑色做 alpha 混合等价于整块乘 (1-alpha)，比 addWeighted 少一次临时构造
    block = canvas[y0:y1, x0:x1]
    block[:] = (block.astype(np.float32) * (1.0 - alpha)).astype(np.uint8)

    first_baseline = y0 + BLOCK_PADDING + height
    for index, (text, color) in enumerate(lines):
        cv2.putText(canvas, text, (x0 + BLOCK_PADDING,
                                   first_baseline + index * step),
                    _FONT, _FONT_SCALE, color, _FONT_THICKNESS, cv2.LINE_AA)


def tilt_degrees(pose) -> tuple[float, float] | None:
    """板面倾角与侧倾方位，单位度。

    倾角是板法线与相机光轴的夹角，0 表示板面正对相机；取法线绝对值，
    所以正反面翻转不影响这个值。方位是法线在图像平面上的方向，
    0 指向图像右、90 指向图像下。
    """
    if pose is None:
        return None
    rotation, _ = cv2.Rodrigues(pose.rvec)
    normal = rotation[:, 2]
    tilt = float(np.degrees(np.arccos(min(1.0, abs(float(normal[2]))))))
    direction = float(np.degrees(np.arctan2(float(normal[1]),
                                            float(normal[0]))))
    return tilt, direction


def target_lines(quad, distance, pose,
                 color: tuple[int, int, int],
                 ) -> list[tuple[str, tuple[int, int, int]]]:
    """一个目标的数据段：段首带编号与颜色，随后两行缩进列数值。"""
    lines = [('#%d  d=%s  pnp=%s  e=%s' % (
        quad.number,
        '--' if distance is None else '%.2f' % distance,
        '--' if pose is None else '%.2f' % pose.distance,
        '--' if pose is None else '%.1f' % pose.error), color)]
    tilt = tilt_degrees(pose)
    if tilt is None:
        lines.append(('   cam x=-- y=-- z=--', DATA_COLOR))
        lines.append(('   tilt=-- dir=--', DATA_COLOR))
    else:
        # 面中心在相机系的坐标：原点为相机光心，X 右、Y 下、Z 沿光轴朝前
        x, y, z = pose.tvec.reshape(3) * 1000.0
        lines.append(('   cam x=%.0f y=%.0f z=%.0f mm' % (x, y, z), DATA_COLOR))
        lines.append(('   tilt=%.1f dir=%.0f deg' % tilt, DATA_COLOR))
    return lines


def _box_lines(box: planes.Box) -> list[tuple[str, tuple[int, int, int]]]:
    """箱体数据段：中心与三条边长。"""
    center = box.center * 1000.0
    size = box.size * 1000.0
    return [('box x=%.0f y=%.0f z=%.0f mm' % tuple(center), (255, 255, 255)),
            ('   size %.0f x %.0f x %.0f mm' % tuple(size), DATA_COLOR)]


def tf_status_lines(sample: TargetSample | None,
                    valid: bool) -> list[tuple[str, tuple[int, int, int]]]:
    """发布的 TF 目标状态，放进数据块首行，画面与话题可直接对照。"""
    color = MODE_COLOR if valid else (0, 0, 255)
    if sample is None:
        return [('TF no target', color)]
    return [('TF %s  %s  inc=%.1f deg  %s' % (
        'VALID' if valid else 'STALE', sample.mode.upper(),
        sample.incidence_deg, sample.position_source), color)]


def _draw_target_frame(canvas: np.ndarray, camera: CameraModel,
                       sample: TargetSample, valid: bool) -> None:
    """在面心画一个沿 +Z（-normal，指向箱内）的箭头，标出锁定的目标面。"""
    center = np.asarray(sample.center, dtype=np.float64).reshape(1, 3)
    axis = np.asarray(sample.rotation[:, 2], dtype=np.float64).reshape(1, 3)
    tip = center + axis * min(sample.width, sample.height) * 0.4
    start = _project(camera, center)[0]
    end = _project(camera, tip)[0]
    first = (int(start[0]), int(start[1]))
    second = (int(end[0]), int(end[1]))
    color = MODE_COLOR if valid else (0, 0, 255)
    cv2.arrowedLine(canvas, first, second, color, 2, cv2.LINE_AA,
                    tipLength=0.3)
    cv2.circle(canvas, first, 5, color, -1, cv2.LINE_AA)


def draw_detection(bgr: np.ndarray, detection: Detection,
                   camera: CameraModel | None, axis_length: float,
                   extra_lines: tuple[tuple[str, tuple[int, int, int]], ...] = (),
                   ) -> np.ndarray:
    """单面模式：每个目标一套颜色，数值汇总到左上角数据块。"""
    canvas = bgr.copy()
    lines: list[tuple[str, tuple[int, int, int]]] = list(extra_lines)
    lines.append(('SINGLE', MODE_COLOR))
    for quad, distance, pose in zip(detection.quads, detection.distances,
                                    detection.poses):
        color = TARGET_COLORS[quad.number % len(TARGET_COLORS)]
        lines.extend(target_lines(quad, distance, pose, color))
    # 先算出数据块占哪块，角标据此避让：否则目标落在画面左上角时身份标签被块盖掉
    avoid = block_rect(canvas, lines)
    blockers = [avoid] if avoid is not None else []

    for quad, pose in zip(detection.quads, detection.poses):
        color = TARGET_COLORS[quad.number % len(TARGET_COLORS)]
        pts = np.round(quad.points).astype(np.int32)
        cv2.polylines(canvas, [pts], True, color, 2, cv2.LINE_AA)
        cv2.circle(canvas, quad.center, 4, color, -1, cv2.LINE_AA)
        if pose is not None and camera is not None:
            rotation, _ = cv2.Rodrigues(pose.rvec)
            draw_axes(canvas, camera, rotation,
                      pose.tvec.reshape(3), axis_length)
        blockers.append(draw_corner_tag(
            canvas, '#%d' % quad.number, _top_left_corner(quad.points),
            color, blockers))

    draw_data_block(canvas, lines)
    return canvas


def _face_rotation(face) -> np.ndarray:
    return np.column_stack([face.x_axis, face.y_axis, face.z_axis])


def draw_split(bgr: np.ndarray, detection: Detection, camera: CameraModel,
               axis_length: float,
               extra_lines: tuple[tuple[str, tuple[int, int, int]], ...] = (),
               ) -> np.ndarray:
    """双面/三面模式：每个面一套颜色并标 F1/F2，箱体数据进左上角。"""
    canvas = bgr.copy()
    lines: list[tuple[str, tuple[int, int, int]]] = list(extra_lines)
    lines.append(('%s  %d faces' % (detection.mode.upper(), len(detection.faces)),
                  MODE_COLOR))
    if detection.box is not None:
        lines.extend(_box_lines(detection.box))
    avoid = block_rect(canvas, lines)
    blockers = [avoid] if avoid is not None else []

    for index, face in enumerate(detection.faces):
        color = TARGET_COLORS[index % len(TARGET_COLORS)]
        pts = _project(camera, planes.face_corners(face)).reshape(-1, 1, 2)
        cv2.polylines(canvas, [pts], True, color, 2, cv2.LINE_AA)
        draw_axes(canvas, camera, _face_rotation(face), face.center,
                  min(face.width, face.height) * 0.35)
        blockers.append(draw_corner_tag(
            canvas, 'F%d' % (index + 1), _top_left_corner(pts.reshape(4, 2)),
            color, blockers))

    if detection.box is not None:
        box = detection.box
        draw_axes(canvas, camera,
                  np.column_stack([box.x_axis, box.y_axis, box.z_axis]),
                  box.center, min(axis_length, float(box.size.min()) * 0.35))

    draw_data_block(canvas, lines)
    return canvas


def _limit_width(canvas: np.ndarray, max_width: int) -> np.ndarray:
    # HiDPI 下画布过宽会让窗口超出屏幕，先压到可显示宽度
    if max_width <= 0 or canvas.shape[1] <= max_width:
        return canvas
    scale = max_width / float(canvas.shape[1])
    return cv2.resize(canvas,
                      (max_width, max(1, int(round(canvas.shape[0] * scale)))),
                      interpolation=cv2.INTER_AREA)


def compose(color_bgr: np.ndarray, detection: Detection,
            camera: CameraModel | None, axis_length: float,
            max_width: int, target_sample: TargetSample | None = None,
            target_valid: bool = True) -> np.ndarray:
    """左原图叠框、右二值 mask，并标出正在发布的目标面。"""
    status = tuple(tf_status_lines(target_sample, target_valid))
    if detection.mode in ('dual', 'triple') and camera is not None and detection.faces:
        overlay = draw_split(color_bgr, detection, camera, axis_length, status)
    else:
        overlay = draw_detection(color_bgr, detection, camera, axis_length, status)
    if target_sample is not None and camera is not None:
        _draw_target_frame(overlay, camera, target_sample, target_valid)
    mask_bgr = cv2.cvtColor(detection.mask, cv2.COLOR_GRAY2BGR)
    return _limit_width(np.hstack([overlay, mask_bgr]), max_width)


def waiting(width: int = 640, height: int = 240) -> np.ndarray:
    """还没收到彩色图时的占位画面。"""
    canvas = np.zeros((height, width, 3), np.uint8)
    cv2.putText(canvas, 'waiting for color image ...', (16, height // 2),
                _FONT, 0.7, (0, 180, 255), 2, cv2.LINE_AA)
    return canvas
