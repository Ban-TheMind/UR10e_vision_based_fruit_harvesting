#!/usr/bin/env python3
"""Verify stage isolation and actuator ordering without ROS or hardware."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / 'src/harvesting_bringup/launch'

def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

deployment = load('deployment_test', LAUNCH / 'deployment.py')
PROFILE = deployment.load_profile(ROOT / 'src/harvesting_bringup/config/site.json')

class Action:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.__dict__.update(kwargs)

class Configuration:
    def __init__(self, name): self.name = name
    def perform(self, context): return context[self.name]

class WorkflowTests(unittest.TestCase):
    def test_actual_launch_composition(self):
        modules = {}
        for name in ('ament_index_python', 'ament_index_python.packages', 'launch',
                     'launch.actions', 'launch.launch_description_sources',
                     'launch.substitutions', 'launch_ros', 'launch_ros.actions'):
            modules[name] = types.ModuleType(name)
        modules['ament_index_python.packages'].get_package_share_directory = lambda package: '/share/' + package
        modules['launch'].LaunchDescription = Action
        for name in ('DeclareLaunchArgument', 'IncludeLaunchDescription', 'OpaqueFunction', 'TimerAction'):
            setattr(modules['launch.actions'], name, Action)
        modules['launch.launch_description_sources'].PythonLaunchDescriptionSource = Action
        modules['launch.substitutions'].LaunchConfiguration = Configuration
        modules['launch_ros.actions'].Node = Action
        with patch.dict(sys.modules, modules):
            launch = load('system_launch_test', LAUNCH / 'system.launch.py')
            for mode, expected in [('fake', set()), ('robot', set()),
                                   ('camera', {'camera'}), ('gripper', {'gripper'}),
                                   ('all', {'camera', 'gripper'})]:
                context = {'profile': str(ROOT / 'src/harvesting_bringup/config/site.json'),
                           'mode': mode, 'motion_enabled': 'false', 'rviz': 'false'}
                actions = launch.compose(context)
                flattened = [child for action in actions for child in
                             (action.actions if hasattr(action, 'actions') else [action])]
                nodes = [action for action in flattened if hasattr(action, 'package')]
                self.assertEqual({node.package for node in nodes}, expected)
                for node in nodes:
                    params = node.parameters[0]
                    self.assertFalse(params.get('allow_execution', False))
                    self.assertFalse(params.get('allow_gripper_commands', False))
                if mode in ('fake', 'robot', 'all'):
                    includes = [a for a in flattened if hasattr(a, 'launch_arguments')]
                    moveit = next(a for a in includes if a.args[0].args[0].endswith('foxy_moveit.launch.py'))
                    args = dict(moveit.launch_arguments)
                    self.assertEqual(args['launch_planning_server'], 'true')
                    self.assertEqual(args['allow_execution'], 'false')
                if mode == 'fake':
                    includes = [a for a in flattened if hasattr(a, 'launch_arguments')]
                    self.assertEqual(len(includes), 2)
                    for action in includes:
                        args = dict(action.launch_arguments)
                        self.assertEqual(args['use_fake_hardware'], 'true')
                        self.assertEqual(args['robot_controller'], 'joint_trajectory_controller')
                if mode == 'robot':
                    includes = [a for a in flattened if hasattr(a, 'launch_arguments')]
                    for action in includes:
                        self.assertEqual(dict(action.launch_arguments)['robot_controller'], 'scaled_joint_trajectory_controller')

    def test_enablement_and_calibration(self):
        for mode, enabled in [('demo', False), ('fake', True)]:
            with self.assertRaises(ValueError): deployment.build_plan(PROFILE, mode, enabled)
        self.assertFalse(PROFILE['task']['calibration_verified'])
        plan = deployment.build_plan(PROFILE, 'demo', True)
        self.assertTrue(plan['routine'])
        self.assertTrue(plan['allow_execution'])
        self.assertTrue(plan['allow_gripper_commands'])
        profile = copy.deepcopy(PROFILE)
        profile['task']['calibration_verified'] = True
        self.assertTrue(deployment.build_plan(profile, 'demo', True)['routine'])
        for actual, degrees in zip(profile['task']['birds_eye_joint_pos'], [-180, -90, 127, -123, 270, 0]):
            self.assertAlmostEqual(actual, math.radians(degrees))
        for actual, degrees in zip(profile['task']['drop_joint_pos'], [-148.38, -87.44, 132.69, -131.78, 269.45, -0.33]):
            self.assertAlmostEqual(actual, math.radians(degrees))
        with self.assertRaises(ValueError): deployment.build_plan(profile, 'typo')
        with self.assertRaises(ValueError): deployment.boolean('yes')

    def test_routine_init_requires_enablement_but_not_calibration_flag(self):
        modules = {name: types.ModuleType(name) for name in
                   ('rclpy', 'rclpy.node', 'custom_interface', 'custom_interface.srv')}
        overrides = {'motion_enabled': True, 'calibration_verified': False}
        warnings, clients, destroyed = [], [], []
        configured = []

        class FakeNode:
            def __init__(self, name): self.values = {}
            def declare_parameter(self, name, value):
                self.values[name] = overrides.get(name, value)
            def get_parameter(self, name):
                return types.SimpleNamespace(value=self.values[name])
            def get_logger(self):
                return types.SimpleNamespace(warning=warnings.append, info=lambda _: None)
            def destroy_node(self): destroyed.append(True)
            def create_client(self, service, name):
                clients.append(name)
                return types.SimpleNamespace(srv_name=name, wait_for_service=lambda **_: True)

        modules['rclpy.node'].Node = FakeNode
        for name in ('CameraSrv', 'MovementRequest', 'GripperCmd', 'ResetGripperCmd'):
            setattr(modules['custom_interface.srv'], name, object)
        defaults = dict(birds_eye_joint_pos=PROFILE['task']['birds_eye_joint_pos'],
                        bird_eye_position=[0.0] * 6, drop_position=[0.0] * 6,
                        approach_offset=[0.0] * 3, pick_offset=[0.0] * 3,
                        pick_orientation=[0.0] * 3, retry_scan_delta=[0.0] * 6,
                        distance_tolerance=0.06)
        with patch.dict(sys.modules, modules):
            module = load('routine_init_gate_test', ROOT / 'src/demo_package/demo_package/routine_base.py')
            module.configure_execution_timeout = lambda node, seconds: configured.append(seconds)
            node = module.RoutineBase(defaults)
            self.assertEqual(configured, [600.0])
            self.assertFalse(node.calibration_verified)
            self.assertEqual(len(clients), 4)
            self.assertEqual(len(warnings), 1)
            self.assertIn('unverified', warnings[0])
            overrides['calibration_verified'] = True
            warnings.clear()
            module.RoutineBase(defaults)
            self.assertEqual(warnings, [])
            overrides['motion_enabled'] = False
            configured.clear()
            clients.clear()
            with self.assertRaisesRegex(RuntimeError, 'motion_enabled'):
                module.RoutineBase(defaults)
            self.assertEqual(clients, [])
            self.assertEqual(configured, [])
            self.assertEqual(destroyed, [True])

    def test_wrapper_rejects_before_ros(self):
        for args in (['demo'], ['all', 'motion_enabled:=yes'], ['fake', 'robot_ip:=1.2.3.4'], ['nonsense'], ['demo', 'motion_enabled:=true', 'execution_timeout:=nan'],
                     ['demo', 'motion_enabled:=true', 'execution_timeout:=0']):
            result = subprocess.run(['bash', 'scripts/project'] + args, cwd=ROOT, capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertNotIn('Foxy environment missing', result.stderr)
        help_result = subprocess.run(['bash', 'scripts/project'], cwd=ROOT, capture_output=True, text=True)
        self.assertEqual(help_result.returncode, 0)
        self.assertIn('check', help_result.stdout)

    def test_routine_carries_before_releasing_and_aborts_on_failure(self):
        # Import the real routine with only ROS transport replaced.
        modules = {name: types.ModuleType(name) for name in ('rclpy', 'rclpy.node', 'custom_interface', 'custom_interface.srv')}
        modules['rclpy.node'].Node = object
        for name in ('CameraSrv', 'MovementRequest', 'GripperCmd', 'ResetGripperCmd'):
            setattr(modules['custom_interface.srv'], name, type(name, (), {'Request': type('Request', (), {})}))
        sys.path.insert(0, str(ROOT / 'src/demo_package'))
        with patch.dict(sys.modules, modules):
            from demo_package.vertical_fruit_gripping_demo import DemoRoutine
            from demo_package.routine_base import RoutineBase
            node = DemoRoutine.__new__(DemoRoutine)
            values = dict(max_cycles=1, birds_eye_joint_pos=PROFILE['task']['birds_eye_joint_pos'],
                          bird_eye_position=[0.54, 0.174, 0.9, -1.56, 0., -1.571],
                          drop_position=[0.822, 0.583, 0.556, 0., 3.14, 0.],
                          approach_offset=[-0.5, 0., -0.1], pick_offset=[-0.18, 0.06, -0.03],
                          pick_orientation=[-1.56, 0., -1.571], distance_tolerance=0.06,
                          gripper_open_width=85, gripper_close_width=0, grip_seconds=0.,
                          scan_at_home_only=False, drop_motion='cartesian',
                          return_to_scan_after_drop=True)
            node.__dict__.update(values)
            point = types.SimpleNamespace(x=0.8, y=0.2, z=0.5)
            node.run_detection_at_curr_pos = lambda: [point]
            node.run_detection_at_pos = lambda *args: [point]
            node.get_logger = lambda: types.SimpleNamespace(info=lambda message: None)
            events = []
            node.send_movement_request = lambda pose, *args: events.append(('move', pose))
            node.send_gripper_request = lambda width: events.append(('grip', width))
            node.run_demo()
            close = events.index(('grip', 0))
            self.assertEqual(events[close + 1], ('move', values['drop_position']))
            self.assertEqual(events[close + 2], ('grip', 85))
            events.clear()
            def fail_on_close(width):
                events.append(('grip', width))
                if width == 0: raise RuntimeError('gripper failure')
            node.send_gripper_request = fail_on_close
            with self.assertRaises(RuntimeError): node.run_demo()
            self.assertEqual(events[-1], ('grip', 0))
            self.assertEqual(node.filter_apples_for_pickup([point], 0.3), [])
            # Failed transport leaves the grasp closed and aborts before release.
            events.clear()
            node.send_gripper_request = lambda width: events.append(('grip', width))
            def fail_on_drop(pose, *args):
                events.append(('move', pose))
                if pose == values['drop_position']: raise RuntimeError('arm failure')
            node.send_movement_request = fail_on_drop
            with self.assertRaises(RuntimeError): node.run_demo()
            self.assertEqual(events[-1], ('move', values['drop_position']))
            self.assertEqual(events[-2], ('grip', 0))
            # Site mode must skip the inherited scan and use joint-mode drop.
            node.__dict__.update(PROFILE['task'])
            node.settle_seconds = 0.0
            node.run_detection_at_pos = lambda *args: self.fail('Unexpected inherited scan')
            events.clear()
            node.send_movement_request = lambda pose, command='cartesian', *args: events.append(('move', pose, command))
            node.run_demo()
            self.assertEqual(events[0], ('move', PROFILE['task']['birds_eye_joint_pos'], 'joint'))
            close = events.index(('grip', 0))
            self.assertEqual(events[close + 1], ('move', PROFILE['task']['drop_joint_pos'], 'joint'))
            self.assertEqual(events[close + 2], ('grip', 85))
            self.assertEqual(events[-1], ('grip', 85))
            self.assertEqual(sum(event[0] == 'move' for event in events), 3)
            events.clear()
            def fail_on_joint_drop(pose, command='cartesian', *args):
                events.append(('move', pose, command))
                if command == 'joint' and pose == PROFILE['task']['drop_joint_pos']:
                    raise RuntimeError('drop failure')
            node.send_movement_request = fail_on_joint_drop
            with self.assertRaises(RuntimeError): node.run_demo()
            self.assertEqual(events[-2], ('grip', 0))
            self.assertEqual(events[-1], ('move', PROFILE['task']['drop_joint_pos'], 'joint'))
            # Empty detections exhaust the configured limit and then return.
            node.max_detect_attempts = 3
            node.detection_confidence = 0.5
            node.detection_min_confidence = 0.2
            node.detection_class = 0
            calls = []
            node.send_camera_request = lambda *args: calls.append(args) or types.SimpleNamespace(success=False, coordinates=[])
            with patch('demo_package.vertical_fruit_gripping_demo.time.sleep'):
                self.assertEqual(DemoRoutine.run_detection_at_curr_pos(node), [])
            self.assertEqual(len(calls), 3)
            # A failed actuator response must propagate, not advance the sequence.
            future = types.SimpleNamespace(done=lambda: True, result=lambda: types.SimpleNamespace(success=False, message='failed'))
            modules['rclpy'].spin_until_future_complete = lambda *a, **kw: None
            client = types.SimpleNamespace(srv_name='test', call_async=lambda req: future)
            node.service_timeout = 1.0
            with self.assertRaises(RuntimeError): RoutineBase.call(node, client, object(), actuator=True)
            cancelled = []
            future = types.SimpleNamespace(done=lambda: False, cancel=lambda: cancelled.append(True))
            with self.assertRaisesRegex(RuntimeError, 'execution state unknown'):
                RoutineBase.call(node, client, object(), actuator=True)
            self.assertEqual(cancelled, [True])

if __name__ == '__main__':
    unittest.main()
