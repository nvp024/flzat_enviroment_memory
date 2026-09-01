"""Convert ROS RGB-D images without compiled cv_bridge bindings."""

from __future__ import annotations

import numpy as np

from sensor_msgs.msg import Image


class ImageConversionError(ValueError):
    """Raised when a ROS image violates the supported memory contract."""


def image_to_bgr(message: Image) -> np.ndarray:
    """Convert a supported 8-bit ROS image into a detached BGR array."""
    encoding = message.encoding.lower()
    channels_by_encoding = {
        "bgr8": 3,
        "rgb8": 3,
        "bgra8": 4,
        "rgba8": 4,
        "mono8": 1,
    }
    channels = channels_by_encoding.get(encoding)
    if channels is None:
        raise ImageConversionError(
            f"unsupported RGB encoding {message.encoding!r}; expected "
            "bgr8, rgb8, bgra8, rgba8, or mono8"
        )

    pixels = _image_array(message, np.dtype(np.uint8), channels)
    if encoding == "bgr8":
        return pixels
    if encoding == "rgb8":
        return np.ascontiguousarray(pixels[:, :, ::-1])
    if encoding == "bgra8":
        return np.ascontiguousarray(pixels[:, :, :3])
    if encoding == "rgba8":
        return np.ascontiguousarray(pixels[:, :, (2, 1, 0)])
    return np.repeat(pixels[:, :, :1], 3, axis=2)


def image_to_depth_32fc1(message: Image) -> np.ndarray:
    """Convert 32FC1 ROS data to a detached native-endian float32 array."""
    if message.encoding.upper() != "32FC1":
        raise ImageConversionError(
            f"unsupported depth encoding {message.encoding!r}; expected 32FC1"
        )
    byte_order = ">" if bool(message.is_bigendian) else "<"
    source_dtype = np.dtype(f"{byte_order}f4")
    depth = _image_array(message, source_dtype, 1)[:, :, 0]
    native_dtype = np.dtype(np.float32).newbyteorder("=")
    if depth.dtype != native_dtype or not depth.dtype.isnative:
        depth = depth.astype(native_dtype, copy=False)
    return np.ascontiguousarray(depth)


def _image_array(
    message: Image,
    dtype: np.dtype,
    channels: int,
) -> np.ndarray:
    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    if height <= 0 or width <= 0:
        raise ImageConversionError("image width and height must be positive")
    if channels <= 0:
        raise ImageConversionError("image channel count must be positive")

    bytes_per_pixel = dtype.itemsize * channels
    minimum_step = width * bytes_per_pixel
    if step < minimum_step:
        raise ImageConversionError(
            f"image step {step} is smaller than required row size "
            f"{minimum_step}"
        )
    if step % dtype.itemsize:
        raise ImageConversionError(
            f"image step {step} is not aligned to {dtype.itemsize}-byte values"
        )

    try:
        raw = memoryview(message.data).cast("B")
    except (TypeError, ValueError):
        raw = memoryview(bytes(message.data))
    required_bytes = height * step
    if raw.nbytes < required_bytes:
        raise ImageConversionError(
            f"image data contains {raw.nbytes} bytes; expected at least "
            f"{required_bytes}"
        )

    values_per_row = step // dtype.itemsize
    required_values = width * channels
    rows = np.frombuffer(
        raw,
        dtype=dtype,
        count=height * values_per_row,
    ).reshape(height, values_per_row)
    pixels = rows[:, :required_values].reshape(height, width, channels)

    # Always detach from the ROS message buffer before the callback returns.
    return np.array(pixels, copy=True, order="C")
