"""Camera geometry that can be checked without ROS or robot hardware."""

import math


# Aligned color optical frame -> UR controller base frame, in meters.
CAMERA_TO_BASE = (
    (-0.14168, -0.28354, 0.94844, -0.12025),
    (-0.98968, 0.019898, -0.14189, 0.41004),
    (0.021358, -0.95875, -0.28343, 0.39921),
    (0.0, 0.0, 0.0, 1.0),
)


def camera_point_to_base(x, y, z):
    """Apply the measured eye-to-hand transform to a point in meters."""
    if not all(math.isfinite(v) for v in (x, y, z)):
        raise ValueError("Camera coordinates must be finite")
    return tuple(sum(row[i] * (x, y, z)[i] for i in range(3)) + row[3]
                 for row in CAMERA_TO_BASE[:3])


def controller_base_to_base_link(x, y, z):
    """Apply the Foxy UR description's 180 degree Z rotation for planning."""
    if not all(math.isfinite(v) for v in (x, y, z)):
        raise ValueError("Base coordinates must be finite")
    return (-x, -y, z)


def unflip_vertical_pixel(y, height):
    """Map a detection on a vertically flipped image back to the depth image."""
    if not 0 <= y < height:
        raise ValueError("Detection row is outside the image")
    return height - 1 - y


def deproject_pixel(pixel_x, pixel_y, depth_m, info):
    """Convert a pixel and aligned depth to optical-frame meters.

    ``info`` needs width, height, k, d and distortion_model, as in CameraInfo.
    OpenCV is imported only when nonzero distortion must be corrected.
    """
    if (not math.isfinite(depth_m) or depth_m <= 0 or
            not math.isfinite(pixel_x) or not math.isfinite(pixel_y) or
            not 0 <= pixel_x < info.width or not 0 <= pixel_y < info.height):
        return None
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    if not all(math.isfinite(v) for v in (fx, fy, cx, cy)) or fx <= 0 or fy <= 0:
        raise ValueError("Invalid camera intrinsics")
    if any(not math.isfinite(v) for v in info.d):
        raise ValueError("Invalid distortion coefficients")
    if any(abs(v) > 1e-12 for v in info.d):
        if info.distortion_model not in ("plumb_bob", "rational_polynomial"):
            raise ValueError("Unsupported nonzero distortion: " + info.distortion_model)
        import cv2
        import numpy as np

        matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        pixel = np.asarray([[[pixel_x, pixel_y]]], dtype=np.float64)
        normalized = cv2.undistortPoints(
            pixel, matrix, np.asarray(info.d, dtype=np.float64))[0, 0]
        nx, ny = float(normalized[0]), float(normalized[1])
    else:
        nx, ny = (pixel_x - cx) / fx, (pixel_y - cy) / fy
    return (nx * depth_m, ny * depth_m, depth_m)
