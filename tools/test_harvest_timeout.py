#!/usr/bin/env python3
"""Exercise the harvesting timeout transport and launch contract without hardware."""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]

def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

class Transport:
    def __init__(self):
        self.modules = {name: types.ModuleType(name) for name in (
            'rclpy', 'rclpy.node', 'custom_interface', 'custom_interface.srv',
            'rcl_interfaces', 'rcl_interfaces.srv', 'rcl_interfaces.msg')}
        self.modules['rclpy.node'].Node = object
        self.modules['rclpy'].spin_until_future_complete = lambda *args, **kwargs: None
        for name in ('CameraSrv', 'MovementRequest', 'GripperCmd', 'ResetGripperCmd'):
            setattr(self.modules['custom_interface.srv'], name,
                    types.SimpleNamespace(Request=types.SimpleNamespace))
        for name in ('SetParameters', 'GetParameters'):
            setattr(self.modules['rcl_interfaces.srv'], name,
                    types.SimpleNamespace(Request=types.SimpleNamespace))
        self.modules['rcl_interfaces.msg'].Parameter = types.SimpleNamespace
        self.modules['rcl_interfaces.msg'].ParameterValue = types.SimpleNamespace
        self.modules['rcl_interfaces.msg'].ParameterType = types.SimpleNamespace(
            PARAMETER_BOOL=1, PARAMETER_DOUBLE=3)
        self.values, self.clients, self.destroyed, self.messages = {}, [], [], []
        self.refuse = self.mismatch = self.pending = self.unavailable = False

    def create_client(self, service, name):
        self.clients.append(name)
        def request(req):
            if name.endswith('set_parameters'):
                self.values = {param.name: param.value for param in req.parameters}
                response = types.SimpleNamespace(results=[types.SimpleNamespace(
                    successful=not self.refuse) for _ in req.parameters])
            else:
                values = [self.values[key] for key in req.names]
                if self.mismatch:
                    values = [types.SimpleNamespace(type=3, double_value=1.2)] + values[1:]
                response = types.SimpleNamespace(values=values)
            return types.SimpleNamespace(done=lambda: not self.pending,
                                         result=lambda: response, cancel=lambda: None)
        return types.SimpleNamespace(srv_name=name, call_async=request,
                                     wait_for_service=lambda **_: not self.unavailable)

    def destroy_client(self, client): self.destroyed.append(client.srv_name)
    def get_logger(self): return types.SimpleNamespace(info=self.messages.append)

class HarvestTimeoutTests(unittest.TestCase):
    def test_manager_runtime_configuration_and_readback(self):
        transport = Transport()
        with patch.dict(sys.modules, transport.modules):
            routine = load('timeout_routine', ROOT / 'src/demo_package/demo_package/routine_base.py')
            routine.configure_execution_timeout(transport, 90.0)
        prefix = 'trajectory_execution.'
        self.assertEqual(transport.values[prefix + 'allowed_execution_duration_scaling'].double_value, 0.0)
        self.assertEqual(transport.values[prefix + 'allowed_goal_duration_margin'].double_value, 90.0)
        self.assertTrue(transport.values[prefix + 'execution_duration_monitoring'].bool_value)
        self.assertTrue(all(name.startswith('/moveit_simple_controller_manager/') for name in transport.clients))
        self.assertEqual(transport.destroyed, transport.clients)
        self.assertIn('90.0', transport.messages[0])

    def test_configuration_failures_abort_and_clean_up(self):
        for failure in ('refuse', 'mismatch', 'pending', 'unavailable'):
            with self.subTest(failure=failure):
                transport = Transport()
                setattr(transport, failure, True)
                with patch.dict(sys.modules, transport.modules):
                    routine = load('timeout_failure', ROOT / 'src/demo_package/demo_package/routine_base.py')
                    with self.assertRaises(RuntimeError):
                        routine.configure_execution_timeout(transport, 600.0)
                self.assertEqual(transport.destroyed, transport.clients)
                self.assertEqual(transport.messages, [])

    def test_motion_waits_for_execution_budget_but_gripper_keeps_service_timeout(self):
        transport = Transport()
        with patch.dict(sys.modules, transport.modules):
            routine = load('timeout_requests', ROOT / 'src/demo_package/demo_package/routine_base.py')
            node = routine.RoutineBase.__new__(routine.RoutineBase)
            node.execution_timeout, node.service_timeout = 600.0, 120.0
            node.gripper_force = 40
            waits = []
            transport.modules['rclpy'].spin_until_future_complete = lambda *a, **kw: waits.append(kw['timeout_sec'])
            future = types.SimpleNamespace(done=lambda: True, result=lambda: types.SimpleNamespace(success=True))
            client = types.SimpleNamespace(call_async=lambda req: future)
            node.movement_client = node.gripper_client = client
            node.send_movement_request([0.0] * 6, 'joint')
            node.send_movement_request([0.0] * 6, 'cartesian')
            node.send_gripper_request(85)
            self.assertEqual(waits, [660.0, 660.0, 120.0])

    def test_finite_positive_profile_and_override(self):
        deployment = load('timeout_deployment', ROOT / 'src/harvesting_bringup/launch/deployment.py')
        self.assertEqual(deployment.execution_timeout({'task': {}}), 600.0)
        self.assertEqual(deployment.execution_timeout({'task': {'execution_timeout': 300.0}}), 300.0)
        self.assertEqual(deployment.execution_timeout({'task': {'execution_timeout': 300.0}}, '90'), 90.0)
        for value in (True, 0, -1, 'nan', 'inf', 'invalid'):
            with self.assertRaises((ValueError, TypeError)):
                deployment.execution_timeout({'task': {'execution_timeout': value}})

    def test_launch_delivers_override_to_actual_harvesting_node(self):
        sys.path.insert(0, str(ROOT / 'tests'))
        from test_project_workflow import Action, Configuration, LAUNCH
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
        with patch.dict(sys.modules, modules):
            launch = load('timeout_launch', LAUNCH / 'system.launch.py')
            for override, expected in (('', 600.0), ('90', 90.0)):
                actions = launch.compose(dict(profile=str(ROOT / 'src/harvesting_bringup/config/site.json'),
                                              mode='demo', motion_enabled='true', rviz='false',
                                              execution_timeout=override))
                node = next(action for action in actions if getattr(action, 'package', '') == 'demo_package')
                self.assertEqual(node.parameters[0]['execution_timeout'], expected)

if __name__ == '__main__':
    unittest.main()
