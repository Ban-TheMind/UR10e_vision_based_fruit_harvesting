"""Hardware-free tests of service-to-official-SDK process boundary."""
import sys
import types
from pathlib import Path
import unittest
from unittest.mock import Mock, patch
import subprocess
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gripper.robotiq_sdk import RobotiqSDK


class SDKAdapterTests(unittest.TestCase):
    def setUp(self):
        package = types.ModuleType('ament_index_python.packages')
        package.get_package_prefix = Mock(return_value='/workspace/install/robotiq_sdk_bridge')
        self.modules = patch.dict(sys.modules, {'ament_index_python': types.ModuleType('ament_index_python'),
                                              'ament_index_python.packages': package})
        self.modules.start()
        self.addCleanup(self.modules.stop)

    def test_creation_and_context_do_not_access_device(self):
        with patch('gripper.robotiq_sdk.subprocess.run') as run:
            with RobotiqSDK():
                pass
            run.assert_not_called()

    def test_status_and_move_use_official_bridge_with_explicit_operation(self):
        reply = types.SimpleNamespace(returncode=0, stdout='{"flags":185,"fault":0,"requested":255,"position":180,"current":10}', stderr='')
        with patch('gripper.robotiq_sdk.subprocess.run', return_value=reply) as run:
            driver = RobotiqSDK()
            driver.read_status()
            self.assertEqual(run.call_args.args[0][1], 'status')
            status = driver.move(0, 40)
            self.assertEqual(run.call_args.args[0][1:], ['move','/dev/ttyUSB0','115200','9','0.5','10.0','64','0','40'])
            self.assertTrue(status.ready)
            self.assertEqual(status.object_state, 2)
            self.assertFalse(run.call_args.kwargs.get('shell', False))

    def test_invalid_motion_never_invokes_sdk(self):
        with patch('gripper.robotiq_sdk.subprocess.run') as run:
            for width, force in ((100, 10), (85, 256), (float('nan'), 10)):
                with self.assertRaises(ValueError):
                    RobotiqSDK().move(width, force)
            run.assert_not_called()

    def test_failure_timeout_and_malformed_response_are_not_success(self):
        driver = RobotiqSDK()
        with patch('gripper.robotiq_sdk.subprocess.run', return_value=types.SimpleNamespace(returncode=1, stderr='SDK serial failure')):
            with self.assertRaisesRegex(IOError, 'SDK serial failure'):
                driver.move(85, 10)
        with patch('gripper.robotiq_sdk.subprocess.run', side_effect=subprocess.TimeoutExpired('sdk', 10)):
            with self.assertRaisesRegex(TimeoutError, 'state unknown'):
                driver.activate()
        with patch('gripper.robotiq_sdk.subprocess.run', return_value=types.SimpleNamespace(returncode=0, stdout='bad', stderr='')):
            with self.assertRaisesRegex(IOError, 'Invalid'):
                driver.read_status()


if __name__ == '__main__':
    unittest.main()
