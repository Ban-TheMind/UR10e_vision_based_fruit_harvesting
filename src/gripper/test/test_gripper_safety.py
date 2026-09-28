"""Verify that the gripper service cannot contact hardware by default."""

import importlib.util
import pathlib
import sys
import types
import unittest


network_calls = []


class FakeNode:
    def __init__(self, name):
        self.parameters = {}

    def declare_parameter(self, name, default):
        self.parameters[name] = default

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self.parameters[name])

    def create_service(self, *args):
        return object()

    def get_logger(self):
        return types.SimpleNamespace(info=lambda *args: None)


rclpy = types.ModuleType('rclpy')
rclpy.node = types.ModuleType('rclpy.node')
rclpy.node.Node = FakeNode
sys.modules['rclpy'] = rclpy
sys.modules['rclpy.node'] = rclpy.node

custom_interface = types.ModuleType('custom_interface')
custom_interface.srv = types.ModuleType('custom_interface.srv')
custom_interface.srv.GripperCmd = object
custom_interface.srv.ResetGripperCmd = object
sys.modules['custom_interface'] = custom_interface
sys.modules['custom_interface.srv'] = custom_interface.srv

requests = types.ModuleType('requests')
requests.get = lambda *args, **kwargs: network_calls.append((args, kwargs))
sys.modules['requests'] = requests

module_path = pathlib.Path(__file__).resolve().parent.parent / 'gripper' / 'gripper_server.py'
spec = importlib.util.spec_from_file_location('gripper_server_under_test', module_path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class GripperSafetyTest(unittest.TestCase):
    def setUp(self):
        network_calls.clear()
        self.server = module.GripperServer()

    def test_width_command_is_blocked_without_opt_in(self):
        request = types.SimpleNamespace(width=50, force=10)
        response = types.SimpleNamespace(success=None, message='')
        result = self.server.gripper_callback(request, response)
        self.assertFalse(result.success)
        self.assertEqual(network_calls, [])

    def test_reset_is_blocked_without_opt_in(self):
        request = types.SimpleNamespace(reset_gripper=True)
        response = types.SimpleNamespace(success=None, message='')
        result = self.server.reset_gripper_callback(request, response)
        self.assertFalse(result.success)
        self.assertEqual(network_calls, [])


if __name__ == '__main__':
    unittest.main()
