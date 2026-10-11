#!/usr/bin/env python3
"""Verify tabletop geometry, one-object limit and failure ordering without ROS."""
import importlib.util
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/demo_package'))
from demo_package.tabletop_geometry import tabletop_poses

def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

class TabletopTests(unittest.TestCase):
    def test_camera_xy_with_taught_height_and_offset(self):
        apple = types.SimpleNamespace(x=-0.7, y=-0.3, z=-0.017)
        above, pick = tabletop_poses(apple, 0.25, [0.0, math.pi, 0.0], [0.01, -0.02], 0.12)
        self.assertAlmostEqual(pick[0], -0.69)
        self.assertAlmostEqual(pick[1], -0.32)
        self.assertEqual(pick[2], 0.25)
        self.assertEqual(pick[3:], [0.0, math.pi, 0.0])
        self.assertAlmostEqual(above[2], 0.37)
        self.assertEqual(above[:2] + above[3:], pick[:2] + pick[3:])
        apple.z = 1.5
        self.assertEqual(tabletop_poses(apple, 0.25, pick[3:], [0.01, -0.02], 0.12)[1], pick)

    def test_missing_field_geometry_blocks_only_harvesting(self):
        deployment = load('tabletop_deployment', ROOT / 'src/harvesting_bringup/launch/deployment.py')
        profile = deployment.load_profile(ROOT / 'src/harvesting_bringup/config/site.json')
        self.assertIsNone(profile['tabletop']['grasp_height_m'])
        for mode in ('fake', 'robot', 'camera', 'gripper', 'all'):
            deployment.build_plan(profile, mode, False)
        with self.assertRaisesRegex(ValueError, '桌面抓取参数尚未填写'):
            deployment.build_plan(profile, 'demo', True)
        profile['tabletop'].update(grasp_height_m=-0.1, tool_rpy_rad=[0, math.pi, 0], xy_offset_m=[0, 0])
        self.assertTrue(deployment.build_plan(profile, 'demo', True)['routine'])
        for name, invalid in (('tool_rpy_rad', [0, 1]), ('xy_offset_m', [0, math.nan]),
                              ('grasp_height_m', math.inf), ('clearance_m', 0)):
            previous = profile['tabletop'][name]
            profile['tabletop'][name] = invalid
            with self.assertRaises(ValueError): deployment.tabletop_parameters(profile, required=True)
            profile['tabletop'][name] = previous

    def routine(self):
        modules = {name: types.ModuleType(name) for name in ('rclpy', 'rclpy.node', 'custom_interface', 'custom_interface.srv')}
        modules['rclpy.node'].Node = object
        for name in ('CameraSrv', 'MovementRequest', 'GripperCmd', 'ResetGripperCmd'):
            setattr(modules['custom_interface.srv'], name, object)
        with patch.dict(sys.modules, modules):
            from demo_package.vertical_fruit_gripping_demo import DemoRoutine
        node = DemoRoutine.__new__(DemoRoutine)
        node.__dict__.update(grasp_strategy='tabletop', grasp_height_m=0.25,
                             tool_rpy_rad=[0.0, math.pi, 0.0], xy_offset_m=[0.01, 0.02],
                             clearance_m=0.12, birds_eye_joint_pos=[0.0]*6,
                             drop_joint_pos=[1.0]*6, drop_motion='joint',
                             gripper_open_width=85, gripper_close_width=0,
                             settle_seconds=0.0, grip_seconds=0.0,
                             return_to_scan_after_drop=False)
        events = []
        node.send_movement_request = lambda pose, command='cartesian', constraint='NONE': events.append(('move', list(pose), command, constraint))
        node.send_gripper_request = lambda width: events.append(('grip', width)) or types.SimpleNamespace(message='object_state=2')
        node.get_logger = lambda: types.SimpleNamespace(info=lambda _: None)
        node.run_detection_at_curr_pos = lambda: [types.SimpleNamespace(x=0.4, y=0.2, z=0.1),
                                                   types.SimpleNamespace(x=0.8, y=0.9, z=0.5)]
        return node, events

    def test_one_apple_uses_existing_ompl_and_retreat_before_drop(self):
        node, events = self.routine()
        node.run_demo()
        self.assertEqual([event[0] for event in events], ['move', 'grip', 'move', 'move', 'grip', 'move', 'move', 'grip'])
        self.assertEqual([event[1] for event in events if event[0] == 'grip'], [85, 0, 85])
        self.assertEqual(events[2], events[5])  # same above pose before and after grasp
        self.assertAlmostEqual(events[3][1][0], 0.41)
        self.assertEqual(events[3][1][2], 0.25)
        self.assertEqual(events[6][1], node.drop_joint_pos)
        self.assertTrue(all(event[2:] == ('cartesian', 'NONE') for event in (events[2], events[3], events[5])))

    def test_failed_grasp_retreat_or_drop_never_releases_or_advances(self):
        for phase in ('close', 'retreat', 'drop'):
            node, events = self.routine()
            original_move, original_grip = node.send_movement_request, node.send_gripper_request
            calls = []
            def move(pose, command='cartesian', constraint='NONE'):
                original_move(pose, command, constraint)
                calls.append(command)
                if (phase == 'retreat' and len(calls) == 4) or (phase == 'drop' and len(calls) == 5):
                    raise RuntimeError(phase)
            def grip(width):
                result = original_grip(width)
                if phase == 'close' and width == 0: raise RuntimeError(phase)
                return result
            node.send_movement_request, node.send_gripper_request = move, grip
            with self.assertRaisesRegex(RuntimeError, phase): node.run_demo()
            self.assertEqual([event[1] for event in events if event[0] == 'grip'], [85, 0])

    def test_empty_detection_and_invalid_geometry_do_not_grasp(self):
        node, events = self.routine()
        node.run_detection_at_curr_pos = lambda: []
        node.run_demo()
        self.assertEqual(len(events), 1)  # Home only
        node.grasp_height_m = math.nan
        events.clear()
        with self.assertRaises(ValueError): node.run_demo()
        self.assertEqual(events, [])

if __name__ == '__main__':
    unittest.main()
