import numpy as np
import pytest
from sensor_msgs.msg import Image

from view import codec


def make_message(array, encoding, padding=0):
    """按给定数组造一条 Image，padding 模拟行末对齐填充。"""
    height, width = array.shape[:2]
    flat = array.reshape(height, -1)
    channels = flat.shape[1]
    itemsize = np.dtype(array.dtype).itemsize

    msg = Image()
    msg.height = height
    msg.width = width
    msg.encoding = encoding
    msg.step = channels * itemsize + padding
    if padding:
        buffer = np.zeros((height, channels + padding // itemsize), array.dtype)
        buffer[:, :channels] = flat
        msg.data = buffer.tobytes()
    else:
        msg.data = flat.tobytes()
    return msg


def test_bgr8_with_row_padding():
    image = np.arange(4 * 5 * 3, dtype=np.uint8).reshape(4, 5, 3)
    decoded, scale = codec.decode(make_message(image, 'bgr8', padding=6))
    assert decoded.shape == (4, 5, 3)
    assert scale == 1.0
    assert np.array_equal(decoded, image)


def test_rgb8_is_converted_to_bgr():
    image = np.zeros((2, 2, 3), np.uint8)
    image[..., 0] = 255                 # 纯红（RGB 顺序）
    decoded, _ = codec.decode(make_message(image, 'rgb8'))
    assert decoded[0, 0].tolist() == [0, 0, 255]


def test_depth_16uc1_reports_millimetre_scale():
    depth = np.full((3, 4), 1234, np.uint16)
    decoded, scale = codec.decode(make_message(depth, '16UC1'))
    assert decoded.dtype == np.uint16
    assert decoded.shape == (3, 4)
    assert scale == 0.001


def test_depth_32fc1_is_already_metres():
    depth = np.full((3, 4), 0.5, np.float32)
    decoded, scale = codec.decode(make_message(depth, '32FC1'))
    assert decoded.dtype == np.float32
    assert scale == 1.0


def test_unsupported_encoding_raises():
    msg = Image()
    msg.height, msg.width = 2, 2
    msg.encoding = 'yuv422_yuy2'
    msg.step = 4
    msg.data = bytes(8)
    with pytest.raises(codec.UnsupportedEncoding):
        codec.decode(msg)
