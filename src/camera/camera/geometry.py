"""Camera geometry that can be checked without ROS or robot hardware."""

import math
import json
from pathlib import Path


CALIBRATION_FILE = Path(__file__).parent / 'calibration/camera_to_base.json'


def load_calibration(path=None):
    data = json.loads(Path(path or CALIBRATION_FILE).read_text())
    if (data.get('source_frame') != 'camera_color_optical_frame' or
            data.get('target_frame') != 'base' or data.get('units') != 'm'):
        raise ValueError('Calibration must map color optical frame to UR base in meters')
    matrix = data['matrix']
    if len(matrix) != 4 or any(len(row) != 4 for row in matrix):
        raise ValueError('Calibration must be a 4x4 matrix')
    if not all(math.isfinite(value) for row in matrix for value in row):
        raise ValueError('Calibration must be finite')
    if matrix[3] != [0, 0, 0, 1]:
        raise ValueError('Invalid homogeneous calibration row')
    rotation = [row[:3] for row in matrix[:3]]
    for i in range(3):
        for j in range(3):
            product = sum(rotation[k][i]*rotation[k][j] for k in range(3))
            if abs(product - int(i == j)) > 1e-3:
                raise ValueError('Calibration rotation must be orthonormal')
    a,b,c = rotation
    determinant = a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0])
    if abs(determinant-1) > 1e-3:
        raise ValueError('Calibration rotation must have determinant +1')
    if data.get('rotation') != rotation or data.get('translation_m') != [row[3] for row in matrix[:3]]:
        raise ValueError('Calibration matrix and TF fields differ')
    return tuple(tuple(row) for row in matrix)


CAMERA_TO_BASE = load_calibration()


def camera_point_to_base(x, y, z, matrix=CAMERA_TO_BASE):
    """Apply the measured eye-to-hand transform to a point in meters."""
    if not all(math.isfinite(v) for v in (x, y, z)):
        raise ValueError("Camera coordinates must be finite")
    return tuple(sum(row[i] * (x, y, z)[i] for i in range(3)) + row[3]
                 for row in matrix[:3])


def controller_base_to_base_link(x, y, z):
    """Apply the UR description's 180 degree Z rotation for planning."""
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
