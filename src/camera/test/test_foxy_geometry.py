"""No-hardware checks for aligned depth units and eye-to-hand coordinates."""

import unittest

import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo

from camera.detection_utils import DetectionHandler
from camera.tf_utils import TFHandler


class FoxyGeometryTest(unittest.TestCase):
    def test_color_optical_forward_maps_to_ur_base(self):
        info = CameraInfo()
        info.width = 640
        info.height = 480
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [606.2232666, 0.0, 326.2407227,
                  0.0, 606.2690429, 243.1719818,
                  0.0, 0.0, 1.0]
        handler = TFHandler.__new__(TFHandler)
        handler.camera_info = info
        optical = handler.pixel_to_3d(info.k[2], info.k[5], 1.0)
        self.assertAlmostEqual(optical[0], 0.0, places=6)
        self.assertAlmostEqual(optical[1], 0.0, places=6)
        self.assertAlmostEqual(optical[2], 1.0, places=6)
        base = handler.transform_to_base(Point(x=optical[0], y=optical[1], z=optical[2]))
        self.assertAlmostEqual(base.x, 0.82819, places=5)
        self.assertAlmostEqual(base.y, 0.26815, places=5)
        self.assertAlmostEqual(base.z, 0.11578, places=5)

    def test_depth_encoding_is_converted_once(self):
        detector = DetectionHandler.__new__(DetectionHandler)
        millimeters = np.full((11, 11), 1000, dtype=np.uint16)
        meters = np.full((11, 11), 1.0, dtype=np.float32)
        self.assertAlmostEqual(detector.get_average_depth(5, 5, millimeters, '16UC1'), 1.0)
        self.assertAlmostEqual(detector.get_average_depth(5, 5, meters, '32FC1'), 1.0)


if __name__ == '__main__':
    unittest.main()
