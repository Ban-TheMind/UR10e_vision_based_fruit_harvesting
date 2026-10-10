#!/usr/bin/env python3
"""Run camera geometry and depth checks without ROS or physical devices."""

import math
from pathlib import Path
import sys
import types
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "camera"))
from camera.depth_utils import median_depth_m  # noqa: E402
from camera.geometry import (  # noqa: E402
    CAMERA_TO_BASE, camera_point_to_base, controller_base_to_base_link,
    deproject_pixel, unflip_vertical_pixel)


class OfflineGeometryTest(unittest.TestCase):
    def setUp(self):
        self.info = types.SimpleNamespace(
            width=640, height=480,
            k=[606.2232666, 0.0, 326.2407227,
               0.0, 606.2690429, 243.1719818,
               0.0, 0.0, 1.0],
            d=[0.0] * 5, distortion_model="plumb_bob")

    def test_principal_pixel_one_meter_forward(self):
        point = deproject_pixel(self.info.k[2], self.info.k[5], 1.0, self.info)
        self.assertEqual(point, (0.0, 0.0, 1.0))
        base = camera_point_to_base(*point)
        for actual, expected in zip(base, (0.82819, 0.26815, 0.11578)):
            self.assertAlmostEqual(actual, expected, places=5)
        planning = controller_base_to_base_link(*base)
        for actual, expected in zip(planning, (-0.82819, -0.26815, 0.11578)):
            self.assertAlmostEqual(actual, expected, places=5)

    def test_off_axis_pixel_and_vertical_flip(self):
        x = self.info.k[2] + 0.5 * self.info.k[0]
        y = self.info.k[5]
        self.assertAlmostEqual(deproject_pixel(x, y, 0.5, self.info)[0], 0.25)
        self.assertEqual(unflip_vertical_pixel(0, 480), 479)
        self.assertEqual(unflip_vertical_pixel(479, 480), 0)

    def test_matrix_is_a_reasonable_rotation(self):
        rotation = np.asarray([row[:3] for row in CAMERA_TO_BASE[:3]])
        self.assertTrue(np.allclose(rotation @ rotation.T, np.eye(3), atol=0.002))
        self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0, delta=0.002)
        self.assertEqual(CAMERA_TO_BASE[3], (0.0, 0.0, 0.0, 1.0))

    def test_invalid_depth_and_pixels_are_rejected(self):
        for depth in (0.0, -1.0, math.nan, math.inf):
            self.assertIsNone(deproject_pixel(100, 100, depth, self.info))
        self.assertIsNone(deproject_pixel(640, 10, 1.0, self.info))
        with self.assertRaises(ValueError):
            unflip_vertical_pixel(480, 480)

    def test_depth_encoding_and_invalid_samples(self):
        millimeters = np.full((11, 11), 1000, dtype=np.uint16)
        meters = np.full((11, 11), 1.0, dtype=np.float32)
        self.assertEqual(median_depth_m(millimeters, "16UC1", 5, 5), 1.0)
        self.assertEqual(median_depth_m(meters, "32FC1", 5, 5), 1.0)
        meters[0, 0] = np.inf
        meters[0, 1] = np.nan
        self.assertEqual(median_depth_m(meters, "32FC1", 0, 0, radius=2), 1.0)
        self.assertTrue(math.isnan(median_depth_m(meters, "32FC1", -1, 0)))
        with self.assertRaises(ValueError):
            median_depth_m(meters, "mono8", 5, 5)


if __name__ == "__main__":
    unittest.main()
