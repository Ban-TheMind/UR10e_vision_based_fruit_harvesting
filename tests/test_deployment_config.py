#!/usr/bin/env python3
"""Exercise configuration through real launch/CLI entry points, without ROS."""
import copy
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/harvesting_config'))
from harvesting_config import deployment

PROFILE = ROOT / 'src/harvesting_bringup/config/site.json'
PLUGIN = {'sensors': ['harvesting_pointcloud'], 'harvesting_pointcloud': {
    'sensor_plugin': 'occupancy_map_monitor/PointCloudOctomapUpdater'}}


class DeploymentTests(unittest.TestCase):
    def camera(self):
        return {'camera': {'serial_no': '406122071837', 'profile': '640,480,15',
                           'inference_python': '/existing/python'}}

    def mapping(self):
        profile = deployment.load_profile(PROFILE)
        return {'robot': profile['robot'], 'camera': {
            key: value for key, value in profile['camera'].items() if key != 'inference_python'}}

    def test_only_selected_subsystems_are_required(self):
        cases = [('fake', {}), ('camera', self.camera()), ('mapping', self.mapping()),
                 ('robot', {'robot': self.mapping()['robot']}),
                 ('gripper', {'gripper': {'serial_port': '/dev/ttyUSB0', 'baudrate': 115200, 'slave_id': 9}})]
        for mode, profile in cases:
            with self.subTest(mode=mode):
                result = deployment.resolve_deployment(profile, mode)
                self.assertFalse(result.plan['allow_execution'])
                self.assertFalse(result.plan['routine'])
        # Unused invalid/placeholder settings must not prevent camera diagnostics.
        profile = dict(self.camera(), robot=None, gripper={'baudrate': -1},
                       task={'execution_timeout': 'not taught'}, perception={'max_range': -1})
        self.assertTrue(deployment.resolve_deployment(profile, 'camera').plan['camera'])

    def test_required_subsystems_and_bad_active_settings_fail(self):
        for mode in ('camera', 'mapping', 'robot', 'gripper'):
            with self.subTest(mode=mode), self.assertRaisesRegex(ValueError, 'Missing'):
                deployment.resolve_deployment({}, mode)
        for field, value in [('serial_no', ''), ('profile', '1280,720,30'), ('inference_python', '')]:
            profile = self.camera()
            profile['camera'][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                deployment.resolve_deployment(profile, 'camera')
        with self.assertRaises(ValueError):
            deployment.resolve_deployment({'robot': None}, 'robot')

    def test_mapping_needs_neither_inference_nor_grasp_geometry(self):
        result = deployment.resolve_deployment(self.mapping(), 'mapping')
        self.assertEqual(result.profile['camera']['calibration_file'], '')
        self.assertFalse(result.profile['task']['calibration_verified'])
        self.assertEqual(result.perception['octomap_frame'], 'base_link')
        self.assertEqual(result.tabletop, {})
        self.assertIsNone(result.execution_timeout)
        for mode in ('fake', 'mapping'):
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                deployment.resolve_deployment({}, mode, {'motion_enabled': 'true'})

    def test_demo_still_requires_enablement_and_taught_geometry(self):
        with self.assertRaisesRegex(ValueError, 'motion_enabled'):
            deployment.resolve_deployment(PROFILE, 'demo')
        with self.assertRaisesRegex(ValueError, '桌面抓取参数'):
            deployment.resolve_deployment(PROFILE, 'demo', {'motion_enabled': True})
        profile = deployment.load_profile(PROFILE)
        profile['tabletop'].update(grasp_height_m=0.25, tool_rpy_rad=[0, 3.14, 0], xy_offset_m=[0, 0])
        resolved = deployment.resolve_deployment(profile, 'demo', {'motion_enabled': True})
        self.assertTrue(resolved.plan['routine'])
        self.assertFalse(resolved.profile['task']['calibration_verified'])

    def test_override_priority_and_input_ownership(self):
        profile = deployment.load_profile(PROFILE)
        profile['tabletop'].update(grasp_height_m=0.25, tool_rpy_rad=[0, 3.14, 0], xy_offset_m=[0, 0])
        profile['task']['execution_timeout'] = 300
        original = copy.deepcopy(profile)
        default = deployment.resolve_deployment(profile, 'demo', {'motion_enabled': True})
        override = deployment.resolve_deployment(profile, 'demo', {
            'motion_enabled': 'true', 'execution_timeout': '90', 'rviz': 'false'})
        self.assertEqual(default.execution_timeout, 300.0)
        self.assertEqual(override.execution_timeout, 90.0)
        self.assertFalse(override.rviz)
        self.assertEqual(profile, original)
        override.profile['robot']['robot_ip'] = '127.0.0.1'
        self.assertEqual(profile, original)
        self.assertEqual(deployment.execution_timeout({'task': {}}), 600.0)

    def test_invalid_overrides_cannot_be_silently_ignored(self):
        for arguments in (['robot_ip:=1.2.3.4'], ['rviz'], ['rviz:=true', 'rviz:=false']):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                deployment.parse_overrides(arguments)
        for override in ({'rviz': 'yes'}, {'execution_timeout': 'nan'}, {'robot_ip': '1.2.3.4'}):
            with self.subTest(override=override), self.assertRaises(ValueError):
                deployment.resolve_deployment(self.camera(), 'camera', override)

    def test_profile_and_direct_moveit_use_the_same_perception_rules(self):
        profile = self.mapping()
        profile['perception'] = {'scene_cloud_topic': '/experiment/scene', 'max_update_rate': 7.5,
                                 'octomap_resolution': 0.02}
        result = deployment.resolve_deployment(profile, 'mapping')
        arguments = {key: str(value) for key, value in result.perception.items() if key != 'camera_root_frame'}
        scene, guard = deployment.moveit_mapping_parameters(arguments, PLUGIN)
        self.assertEqual(scene['harvesting_pointcloud']['point_cloud_topic'], '/experiment/scene')
        self.assertEqual(scene['harvesting_pointcloud']['max_update_rate'], 7.5)
        self.assertEqual(guard['octomap_resolution'], scene['octomap_resolution'])
        self.assertFalse(scene['allow_trajectory_execution'])
        self.assertEqual(PLUGIN['harvesting_pointcloud'], {
            'sensor_plugin': 'occupancy_map_monitor/PointCloudOctomapUpdater'})
        for key, value in [('max_update_rate', 31), ('point_subsample', 0),
                           ('padding_offset', -1), ('max_range', float('nan')),
                           ('octomap_frame', 'base'), ('scene_cloud_topic', '/camera/depth/color/points')]:
            invalid = dict(profile, perception={key: value})
            with self.subTest(key=key), self.assertRaises(ValueError):
                deployment.resolve_deployment(invalid, 'mapping')
            launch_args = dict(deployment.perception_launch_defaults(), **{key: str(value)})
            with self.subTest(launch_key=key), self.assertRaises(ValueError):
                deployment.moveit_mapping_parameters(launch_args, PLUGIN)
        with self.assertRaisesRegex(ValueError, 'plan-only'):
            deployment.moveit_mapping_parameters({}, PLUGIN, allow_execution='true')
        with self.assertRaisesRegex(ValueError, 'sensors_3d'):
            deployment.moveit_mapping_parameters({}, {})

    def test_above_check_does_not_require_unused_devices_or_taught_height(self):
        profile = {'task': {'grasp_strategy': 'tabletop'}, 'tabletop': {'tool_offset_m': [0, 0, 0.1628]}}
        normalized, geometry = deployment.above_check_settings(profile)
        self.assertEqual(geometry['clearance_m'], 0.12)
        with self.assertRaisesRegex(ValueError, 'robot'):
            deployment.above_check_settings(profile, execute=True)
        profile.update(robot={'robot_ip': '192.168.11.60'})
        profile['task']['birds_eye_joint_pos'] = [0.0] * 6
        deployment.above_check_settings(profile, execute=True)

    def test_cli_precheck_and_actual_launch_accept_the_same_partial_profile(self):
        # Only ROS launch construction is substituted; the real configuration runs.
        class Action:
            def __init__(self, *args, **kwargs):
                self.args = args
                self.__dict__.update(kwargs)

        class Configuration:
            def __init__(self, name): self.name = name
            def perform(self, context): return context[self.name]

        modules = {name: types.ModuleType(name) for name in (
            'ament_index_python', 'ament_index_python.packages', 'launch', 'launch.actions',
            'launch.launch_description_sources', 'launch.substitutions', 'launch_ros', 'launch_ros.actions')}
        modules['ament_index_python.packages'].get_package_share_directory = lambda name: '/share/' + name
        modules['launch'].LaunchDescription = Action
        for name in ('DeclareLaunchArgument', 'IncludeLaunchDescription', 'OpaqueFunction', 'TimerAction'):
            setattr(modules['launch.actions'], name, Action)
        modules['launch.launch_description_sources'].PythonLaunchDescriptionSource = Action
        modules['launch.substitutions'].LaunchConfiguration = Configuration
        modules['launch_ros.actions'].Node = Action
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory) / 'camera.json'
            filename.write_text(json.dumps(self.camera()))
            env = dict(os.environ)
            env.pop('PYTHONPATH', None)
            command = [sys.executable, str(ROOT / 'scripts/check_profile.py'), str(filename), 'camera', 'rviz:=false']
            checked = subprocess.run(command, env=env, capture_output=True, text=True)
            self.assertEqual(checked.returncode, 0, checked.stderr)
            with patch.dict(sys.modules, modules):
                source = ROOT / 'src/harvesting_bringup/launch/system.launch.py'
                spec = importlib.util.spec_from_file_location('partial_profile_launch', source)
                launch = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(launch)
                actions = launch.compose(dict(profile=str(filename), mode='camera',
                                              motion_enabled='false', rviz='false', execution_timeout=''))
            nodes = [action for action in actions if getattr(action, 'package', None)]
            self.assertEqual([node.executable for node in nodes], ['camera_node'])
            self.assertEqual(nodes[0].parameters[0]['inference_python'], '/existing/python')
            self.assertEqual(nodes[0].parameters[0]['calibration_file'], '')

    def test_installable_package_is_independent_of_source_and_ros(self):
        with tempfile.TemporaryDirectory() as directory:
            built = Path(directory) / 'python'
            subprocess.run([sys.executable, 'setup.py', 'build_py', '--build-lib', str(built)],
                           cwd=ROOT / 'src/harvesting_config', check=True, capture_output=True, text=True)
            code = ("import sys; sys.path.insert(0, " + repr(str(built)) + "); "
                    "from harvesting_config.deployment import resolve_deployment; "
                    "assert resolve_deployment({}, 'fake').plan['fake']; "
                    "assert 'rclpy' not in sys.modules and 'launch' not in sys.modules")
            subprocess.run([sys.executable, '-I', '-c', code], cwd=directory,
                           check=True, capture_output=True, text=True)

    def test_bad_json_or_root_shape_fails_before_any_launch(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'invalid.json'
            for content in ('{broken', '[]', 'null'):
                path.write_text(content)
                with self.subTest(content=content), self.assertRaises(ValueError):
                    deployment.resolve_deployment(path, 'camera')


if __name__ == '__main__':
    unittest.main()
