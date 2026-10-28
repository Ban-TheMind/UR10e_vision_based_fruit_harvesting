#!/usr/bin/env python3
"""No-hardware geometry, mapping configuration and PointCloud2 data checks."""
import copy
import json
import math
from pathlib import Path
import struct
import sys
import types
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/harvesting_config'))
from harvesting_config import deployment
sys.path.insert(0, str(ROOT / 'src/camera'))
from camera.geometry import load_calibration
from camera.perception_geometry import camera_root_in_base, compose, quaternion_from_matrix, transform_matrix

try:
    from camera.pointcloud_filter import finite_records
except ImportError:
    finite_records = None


class MappingTests(unittest.TestCase):
    def test_mapping_is_isolated_and_nonexecuting(self):
        profile = deployment.load_profile(ROOT / 'src/harvesting_bringup/config/site.json')
        plan = deployment.resolve_deployment(profile, 'mapping').plan
        self.assertTrue(plan['robot'] and plan['camera'] and plan['mapping'])
        for key in ('gripper', 'routine', 'fake', 'allow_execution', 'allow_gripper_commands'):
            self.assertFalse(plan[key])
        with self.assertRaisesRegex(ValueError, 'plan-only'):
            deployment.resolve_deployment(profile, 'mapping', {'motion_enabled': True})
        for field, value in [('point_subsample', 0), ('point_subsample', True),
                             ('octomap_resolution', float('nan')), ('padding_offset', -0.1),
                             ('max_update_rate', float('inf')), ('octomap_frame', 'base'),
                             ('scene_cloud_topic', '/camera/depth/color/points'),
                             ('camera_root_frame', 'camera_color_optical_frame')]:
            invalid = copy.deepcopy(profile)
            invalid['perception'][field] = value
            with self.assertRaises(ValueError, msg=field):
                deployment.perception_parameters(invalid)

    def test_external_tf_preserves_driver_internal_transform(self):
        # Deliberately nontrivial axes/translation catch inverse-order errors.
        base_to_optical = transform_matrix((0.4, -0.2, 0.7), (0., 0., math.sin(0.4), math.cos(0.4)))
        root_to_optical = transform_matrix((0.02, 0.01, 0.), (0.5, -0.5, 0.5, -0.5))
        result = compose(camera_root_in_base(base_to_optical, root_to_optical), root_to_optical)
        for actual, expected in zip(sum(result, ()), sum(base_to_optical, ())):
            self.assertAlmostEqual(actual, expected, places=10)

    def test_quaternion_round_trip_and_controller_rotation(self):
        for q in ((0, 0, 0, 1), (1, 0, 0, 0), (0, 1, 0, 0),
                  (0, 0, 1, 0), (0.2, -0.3, 0.1, 0.8)):
            matrix = transform_matrix((1, 2, 3), q)
            recovered = transform_matrix((1, 2, 3), quaternion_from_matrix(matrix))
            for actual, expected in zip(sum(recovered, ()), sum(matrix, ())):
                self.assertAlmostEqual(actual, expected)
        calibrated = load_calibration()
        base_link_to_base = transform_matrix((0, 0, 0), (0, 0, 1, 0))
        base_link_to_optical = compose(base_link_to_base, calibrated)
        self.assertAlmostEqual(base_link_to_optical[0][3], -calibrated[0][3])
        self.assertAlmostEqual(base_link_to_optical[1][3], -calibrated[1][3])
        self.assertAlmostEqual(base_link_to_optical[2][3], calibrated[2][3])

    @unittest.skipIf(finite_records is None, 'NumPy not installed in this offline interpreter')
    def test_filter_preserves_rgb_and_handles_padded_organized_rows(self):
        fields = [types.SimpleNamespace(name=name, offset=i*4, datatype=7, count=1)
                  for i, name in enumerate(('x', 'y', 'z'))]
        data = (struct.pack('<fffI', 0., 1., 2., 123) +
                struct.pack('<fffI', float('inf'), 1., 2., 456) + b'padding!'+
                struct.pack('<fffI', 0., 0., 0., 789) +
                struct.pack('<fffI', -1., 2., 3., 321) + b'padding!')
        cloud = types.SimpleNamespace(is_bigendian=False, width=2, height=2,
                                      point_step=16, row_step=40, fields=fields, data=data)
        count, result = finite_records(cloud)
        self.assertEqual(count, 2)
        self.assertEqual(result, struct.pack('<fffI', 0., 1., 2., 123) + struct.pack('<fffI', -1., 2., 3., 321))
        cloud.data = b''
        with self.assertRaises(ValueError):
            finite_records(cloud)


if __name__ == '__main__':
    unittest.main()
