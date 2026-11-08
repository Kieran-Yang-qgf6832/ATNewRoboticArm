"""sensor_msgs/Image 到 ndarray 的解码。

不引入 cv_bridge：这里只需要处理 RealSense 实际会发的几种编码，
自己按 step 切行反而更直接，也少一个依赖。
"""

from __future__ import annotations

import cv2
import numpy as np
from sensor_msgs.msg import Image


class UnsupportedEncoding(ValueError):
    """收到了没有对应解码规则的编码。"""


# 编码 -> (numpy 类型, 通道数, 原值换算到基本单位的系数)
_FORMATS: dict[str, tuple[type, int, float]] = {
    'bgr8': (np.uint8, 3, 1.0),
    'rgb8': (np.uint8, 3, 1.0),
    '16UC1': (np.uint16, 1, 0.001),
    'mono16': (np.uint16, 1, 0.001),
    '32FC1': (np.float32, 1, 1.0),
}


def decode(msg: Image) -> tuple[np.ndarray, float]:
    """返回 (图像, 换算系数)。彩色为 BGR 三通道，深度为单通道。

    深度按 16UC1 以毫米为单位发布时系数为 0.001，32FC1 已是米故为 1.0。
    """
    try:
        dtype, channels, scale = _FORMATS[msg.encoding]
    except KeyError:
        raise UnsupportedEncoding('不支持的图像编码: ' + msg.encoding) from None

    itemsize = np.dtype(dtype).itemsize
    # 行末可能带对齐填充，必须按 step 切行而不是 width*itemsize
    row_bytes = msg.step or (msg.width * channels * itemsize)
    rows = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, row_bytes // itemsize)
    image = rows[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    if channels == 1:
        image = image[:, :, 0]
    elif msg.encoding == 'rgb8':
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return np.ascontiguousarray(image), scale
