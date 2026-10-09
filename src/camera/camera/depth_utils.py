"""Depth image conversion shared by ROS and offline tests."""

import math

import numpy as np


def median_depth_m(frame, encoding, x, y, radius=5):
    """Return the median of valid nearby samples in meters, or NaN."""
    if encoding not in ("16UC1", "32FC1"):
        raise ValueError("Unsupported depth encoding: " + str(encoding))
    if radius < 0 or not (0 <= x < frame.shape[1] and 0 <= y < frame.shape[0]):
        return math.nan
    patch = frame[max(0, y-radius):min(frame.shape[0], y+radius+1),
                  max(0, x-radius):min(frame.shape[1], x+radius+1)]
    valid = patch[np.isfinite(patch) & (patch > 0)]
    if valid.size < 3:
        return math.nan
    depth = float(np.median(valid))
    return depth * 0.001 if encoding == "16UC1" else depth
