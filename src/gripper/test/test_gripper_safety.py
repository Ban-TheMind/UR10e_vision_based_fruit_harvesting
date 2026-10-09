"""Import ROS callbacks with stubs; assert disabled requests never open serial."""
import importlib
import sys
import types
from pathlib import Path
import unittest
from unittest.mock import Mock, patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


class SafetyTests(unittest.TestCase):
    def setUp(self):
        node = types.ModuleType('rclpy.node')
        node.Node = object
        srv = types.ModuleType('custom_interface.srv')
        srv.GripperCmd = srv.ResetGripperCmd = object
        with patch.dict(sys.modules, {'rclpy': types.ModuleType('rclpy'),
                                     'rclpy.node': node, 'custom_interface.srv': srv}):
            sys.modules.pop('gripper.gripper_server', None)
            self.module = importlib.import_module('gripper.gripper_server')
        self.server = self.module.GripperServer.__new__(self.module.GripperServer)
        self.server.get_parameter = Mock(return_value=types.SimpleNamespace(value=False))

    def test_disabled_and_invalid_requests_do_not_open_device(self):
        request = types.SimpleNamespace(width=85, force=40, reset_gripper=True)
        with patch.object(self.server, '_driver') as driver:
            for method in (self.server.gripper_callback, self.server.reset_gripper_callback):
                reply = method(request, types.SimpleNamespace())
                self.assertFalse(reply.success)
            self.server.get_parameter.return_value.value = True
            request.width = 100
            self.assertFalse(self.server.gripper_callback(request, types.SimpleNamespace()).success)
            request.reset_gripper = False
            self.assertFalse(self.server.reset_gripper_callback(request, types.SimpleNamespace()).success)
            driver.assert_not_called()

    def test_serial_failure_is_failed_response(self):
        self.server.get_parameter.return_value.value = True
        with patch.object(self.server, '_driver', side_effect=OSError('port unavailable')):
            result = self.server.gripper_callback(types.SimpleNamespace(width=85, force=40), types.SimpleNamespace())
        self.assertFalse(result.success)
        self.assertIn('port unavailable', result.message)


if __name__ == '__main__':
    unittest.main()
