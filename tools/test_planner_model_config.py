#!/usr/bin/env python3
"""Exercise the actual MoveIt launch model wiring without ROS or devices."""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/harvesting_config'))
SOURCE = ROOT / 'src/end_effector_description/launch/foxy_moveit.launch.py'

class Action:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.__dict__.update(kwargs)

class Configuration:
    def __init__(self, name): self.name = name
    def perform(self, context): return context[self.name]

class PlannerModelTests(unittest.TestCase):
    def setUp(self):
        modules = {name: types.ModuleType(name) for name in (
            'yaml', 'ament_index_python', 'ament_index_python.packages',
            'launch', 'launch.actions', 'launch.conditions', 'launch.substitutions',
            'launch_ros', 'launch_ros.actions', 'launch_ros.substitutions')}
        modules['ament_index_python.packages'].get_package_share_directory = lambda name: '/share/' + name
        modules['launch'].LaunchDescription = Action
        modules['launch.actions'].DeclareLaunchArgument = Action
        modules['launch.actions'].OpaqueFunction = Action
        modules['launch.conditions'].IfCondition = Action
        for name in ('Command', 'FindExecutable', 'PathJoinSubstitution'):
            setattr(modules['launch.substitutions'], name, Action)
        modules['launch.substitutions'].LaunchConfiguration = Configuration
        modules['launch_ros.actions'].Node = Action
        modules['launch_ros.substitutions'].FindPackageShare = Action
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location('planner_model_launch_test', SOURCE)
            self.launch = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.launch)
        configs = {
            'config/sensors_3d.yaml': {
                'sensors': ['harvesting_pointcloud'],
                'harvesting_pointcloud': {'sensor_plugin': 'occupancy_map_monitor/PointCloudOctomapUpdater'}},
            'config/kinematics.yaml': {'ur_manipulator': {'kinematics_solver': 'ur_kinematics/URKinematicsPlugin'}},
            'config/ompl_planning.yaml': {'planner_configs': {}},
            'config/controllers.yaml': {'joint_trajectory_controller': {'type': 'FollowJointTrajectory'}},
        }
        self.launch.load_yaml = lambda package, path: dict(configs[path])

    def test_same_model_and_solver_for_planner_and_move_group(self):
        for enabled in (False, True):
            context = dict(robot_controller='scaled_joint_trajectory_controller',
                           launch_planning_server='true', allow_execution=str(enabled).lower(),
                           pointcloud_avoidance='false')
            nodes = self.launch.launch_setup(context)
            planner = next(node for node in nodes if node.package == 'moveit_path_planner')
            group = next(node for node in nodes if node.package == 'moveit_ros_move_group')
            self.assertEqual(sum(node.package == 'moveit_path_planner' for node in nodes), 1)
            for key in ('robot_description', 'robot_description_semantic', 'robot_description_kinematics'):
                planner_value = next(params[key] for params in planner.parameters if key in params)
                group_value = next(params[key] for params in group.parameters if key in params)
                self.assertIs(planner_value, group_value)
            self.assertEqual(planner.parameters[-1]['allow_execution'], enabled)
            self.assertIs(type(planner.parameters[-1]['allow_execution']), bool)
            self.assertEqual(planner.condition.args[0].perform(context), 'true')

    def test_standalone_moveit_keeps_optional_service_disabled(self):
        description = self.launch.generate_launch_description()
        arguments = {item.args[0]: item.default_value for item in description.args[0]
                     if hasattr(item, 'default_value')}
        self.assertEqual(arguments['launch_planning_server'], 'false')
        self.assertEqual(arguments['allow_execution'], 'false')

    def test_mapping_config_is_loaded_into_actual_move_group(self):
        context = dict(self.launch.PERCEPTION_DEFAULTS, robot_controller='scaled_joint_trajectory_controller',
                       launch_planning_server='true', allow_execution='false', pointcloud_avoidance='true')
        nodes = self.launch.launch_setup(context)
        group = next(node for node in nodes if node.package == 'moveit_ros_move_group')
        planner = next(node for node in nodes if node.package == 'moveit_path_planner')
        mapping = next(params for params in group.parameters if 'sensors' in params)
        self.assertEqual(mapping['harvesting_pointcloud']['point_cloud_topic'], '/harvesting/scene_points')
        self.assertFalse(mapping['allow_trajectory_execution'])
        self.assertEqual(mapping['octomap_frame'], 'base_link')
        self.assertTrue(planner.parameters[-1]['pointcloud_avoidance'])
        self.assertFalse(planner.parameters[-1]['allow_execution'])
        context['allow_execution'] = 'true'
        with self.assertRaisesRegex(ValueError, 'plan-only'):
            self.launch.launch_setup(context)

    def test_mapping_overrides_reach_scene_and_guard_and_share_validation(self):
        context = dict(self.launch.PERCEPTION_DEFAULTS, robot_controller='scaled_joint_trajectory_controller',
                       launch_planning_server='true', allow_execution='false', pointcloud_avoidance='true',
                       scene_cloud_topic='/trial/scene', max_update_rate='7.5', octomap_resolution='0.02')
        nodes = self.launch.launch_setup(context)
        group = next(node for node in nodes if node.package == 'moveit_ros_move_group')
        planner = next(node for node in nodes if node.package == 'moveit_path_planner')
        mapping = next(params for params in group.parameters if 'sensors' in params)
        self.assertEqual(mapping['harvesting_pointcloud']['point_cloud_topic'], '/trial/scene')
        self.assertEqual(mapping['harvesting_pointcloud']['max_update_rate'], 7.5)
        self.assertEqual(mapping['octomap_resolution'], 0.02)
        self.assertEqual(planner.parameters[-1]['octomap_resolution'], 0.02)
        context['max_update_rate'] = '31'
        with self.assertRaisesRegex(ValueError, '30 Hz'):
            self.launch.launch_setup(context)

if __name__ == '__main__':
    unittest.main()
