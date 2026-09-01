"""Tests for ROS image conversion across supported encodings and layouts."""

import numpy as np
import pytest

from sensor_msgs.msg import Image

from environment_memory.perception.ros_image import (
    ImageConversionError,
    image_to_bgr,
    image_to_depth_32fc1,
)


def _image(
    *,
    width: int,
    height: int,
    encoding: str,
    step: int,
    data: bytes,
    is_bigendian: bool = False,
) -> Image:
    message = Image()
    message.width = width
    message.height = height
    message.encoding = encoding
    message.step = step
    message.data = data
    message.is_bigendian = is_bigendian
    return message


def test_rgb8_with_row_padding_converts_to_detached_bgr():
    raw = bytearray(
        [
            10,
            20,
            30,
            40,
            50,
            60,
            255,
            255,
            70,
            80,
            90,
            100,
            110,
            120,
            255,
            255,
        ]
    )
    message = _image(
        width=2,
        height=2,
        encoding="rgb8",
        step=8,
        data=raw,
    )

    converted = image_to_bgr(message)
    message.data[0] = 0

    assert converted.dtype == np.uint8
    assert converted.flags.c_contiguous
    assert converted.tolist() == [
        [[30, 20, 10], [60, 50, 40]],
        [[90, 80, 70], [120, 110, 100]],
    ]


@pytest.mark.parametrize(
    ("encoding", "pixel", "expected"),
    [
        ("bgr8", [3, 2, 1], [3, 2, 1]),
        ("bgra8", [3, 2, 1, 200], [3, 2, 1]),
        ("rgba8", [1, 2, 3, 200], [3, 2, 1]),
        ("mono8", [7], [7, 7, 7]),
    ],
)
def test_supported_color_encodings_convert_to_bgr(encoding, pixel, expected):
    message = _image(
        width=1,
        height=1,
        encoding=encoding,
        step=len(pixel),
        data=bytes(pixel),
    )

    assert image_to_bgr(message)[0, 0].tolist() == expected


def test_little_endian_depth_with_row_padding_converts_to_native_float32():
    values = np.array([1.25, 2.5, 99.0, 3.75, 4.5, 99.0], dtype="<f4")
    message = _image(
        width=2,
        height=2,
        encoding="32FC1",
        step=12,
        data=values.tobytes(),
    )

    converted = image_to_depth_32fc1(message)

    assert converted.dtype == np.dtype(np.float32)
    assert converted.dtype.isnative
    assert converted.flags.c_contiguous
    np.testing.assert_allclose(converted, [[1.25, 2.5], [3.75, 4.5]])


def test_big_endian_depth_converts_to_native_float32():
    values = np.array([1.5, 2.75], dtype=">f4")
    message = _image(
        width=2,
        height=1,
        encoding="32FC1",
        step=8,
        data=values.tobytes(),
        is_bigendian=True,
    )

    converted = image_to_depth_32fc1(message)

    assert converted.dtype.isnative
    np.testing.assert_allclose(converted, [[1.5, 2.75]])


@pytest.mark.parametrize(
    "message",
    [
        _image(width=1, height=1, encoding="yuv422", step=2, data=b"\0\0"),
        _image(width=2, height=1, encoding="rgb8", step=5, data=b"\0" * 6),
        _image(width=2, height=2, encoding="rgb8", step=6, data=b"\0" * 6),
        _image(width=1, height=1, encoding="32FC1", step=3, data=b"\0" * 4),
    ],
)
def test_invalid_image_contracts_are_rejected(message):
    converter = (
        image_to_depth_32fc1
        if message.encoding.upper() == "32FC1"
        else image_to_bgr
    )

    with pytest.raises(ImageConversionError):
        converter(message)


def test_depth_converter_rejects_wrong_encoding():
    message = _image(
        width=1,
        height=1,
        encoding="16UC1",
        step=2,
        data=b"\0\0",
    )

    with pytest.raises(ImageConversionError, match="expected 32FC1"):
        image_to_depth_32fc1(message)
