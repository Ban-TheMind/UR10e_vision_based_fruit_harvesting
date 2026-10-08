"""Offline regression tests: never launch devices or publish actuator requests."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import yaml
from launch import LaunchContext

ROOT = Path(__file__).resolve().parent.parent

def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

bringup = load('system_launch', ROOT / 'src/harvesting_bringup/launch/system.launch.py')
preflight = load('preflight', ROOT / 'scripts/check_project.py')
from demo_package.routine_base import RoutineBase
from camera.tf_utils import TFHandler


class LaunchTests(unittest.TestCase):
    def context(self, mode, enabled='false', profile=None):
        ctx = LaunchContext()
        ctx.launch_configurations.update(mode=mode, motion_enabled=enabled, rviz='false',
            profile=str(profile or ROOT / 'src/harvesting_bringup/config/lab.yaml'))
        return ctx

    def test_fake_isolated_and_plan_only(self):
        with patch.object(bringup, 'include') as include, patch.object(bringup, 'Node') as node, patch.object(bringup, 'TimerAction'):
            bringup.start(self.context('fake'))
        self.assertEqual([call.args[0] for call in include.call_args_list], ['harvesting_bringup', 'ur10e_moveit_config_official'])
        driver = include.call_args_list[0].args[2]
        moveit = include.call_args_list[1].args[2]
        self.assertNotIn('robot_ip', driver)
        self.assertEqual(driver['description_file'], moveit['description_file'])
        self.assertEqual(driver['kinematics_params_file'], moveit['kinematics_params_file'])
        self.assertEqual(len(node.call_args_list), 1)
        self.assertEqual(node.call_args.kwargs['parameters'][-1], {'execute_enabled': False})

    def test_gripper_disabled_without_explicit_enable(self):
        with patch.object(bringup, 'Node') as node:
            bringup.start(self.context('gripper'))
        self.assertEqual(node.call_args.kwargs['parameters'][-1], {'commands_enabled': False})

    def test_unverified_demo_cannot_start(self):
        with patch.object(bringup, 'Node') as node, self.assertRaises(ValueError):
            bringup.start(self.context('demo', 'true'))
        node.assert_not_called()

    def test_all_launch_descriptions_construct(self):
        for path in (ROOT / 'src/harvesting_bringup/launch').glob('*.py'):
            load(path.stem, path).generate_launch_description()
        load('moveit_launch', ROOT / 'src/ur10e_moveit_config_official/launch/ur_moveit.launch.py').generate_launch_description()

    def test_invalid_mode_rejected(self):
        with self.assertRaises(ValueError):
            bringup.start(self.context('typo'))

    def test_profile_rejects_invalid_geometry_and_legacy_verification(self):
        data = yaml.safe_load((ROOT / 'src/harvesting_bringup/config/lab.yaml').read_text())
        preflight.validate_profile(data)
        data['moveit_path_planning_server']['ros__parameters']['table'][0] = -1
        with self.assertRaises(ValueError):
            preflight.validate_profile(data)
        data['moveit_path_planning_server']['ros__parameters']['table'][0] = 3
        data['deployment']['calibration_verified'] = True
        with self.assertRaises(ValueError):
            preflight.validate_profile(data)


class RequestTests(unittest.TestCase):
    def test_actuator_failure_aborts(self):
        client = Mock(srv_name='/moveit_path_plan')
        client.call_async.return_value = Mock(done=lambda: True, result=lambda: SimpleNamespace(success=False))
        with patch('rclpy.spin_until_future_complete'), self.assertRaises(RuntimeError):
            RoutineBase.call(SimpleNamespace(service_timeout=1.0), client, object(), actuator=True)

    def test_timeout_aborts_and_marks_unknown_result(self):
        future = Mock(done=lambda: False)
        client = Mock(srv_name='/moveit_path_plan')
        client.call_async.return_value = future
        with patch('rclpy.spin_until_future_complete'), self.assertRaisesRegex(RuntimeError, 'execution state unknown'):
            RoutineBase.call(SimpleNamespace(service_timeout=1.0), client, object(), actuator=True)
        future.cancel.assert_called_once()

    def test_empty_detection_is_not_an_actuator_failure(self):
        response = SimpleNamespace(success=True, coordinates=[])
        client = Mock(srv_name='/camera_srv')
        client.call_async.return_value = Mock(done=lambda: True, result=lambda: response)
        with patch('rclpy.spin_until_future_complete'):
            self.assertIs(RoutineBase.call(SimpleNamespace(service_timeout=1.0), client, object()), response)


class CoordinateTests(unittest.TestCase):
    def test_tf_mode_uses_xy_meters_without_legacy_offset(self):
        params = {'coordinate_mode': 'tf', 'depth_scale': 0.001}
        node = Mock()
        node.get_parameter.side_effect = lambda name: SimpleNamespace(value=params[name])
        handler = SimpleNamespace(node=node, intrinsics=object())
        with patch('camera.tf_utils.rs.rs2_deproject_pixel_to_point', return_value=[0.1, 0.2, 1.0]) as deproject:
            result = TFHandler.pixel_to_3d(handler, 20, 30, 1000)
        self.assertEqual(deproject.call_args.args[1:], ([20, 30], 1.0))
        self.assertEqual(result, [0.1, 0.2, 1.0])



class TaskSequenceTests(unittest.TestCase):
    def fixture(self):
        from demo_package.vertical_fruit_gripping_demo import DemoRoutine
        point = SimpleNamespace(x=1.0, y=0.1, z=0.8)
        state = SimpleNamespace(
            max_cycles=1, birds_eye_joint_pos=[0.0] * 6,
            bird_eye_position=[0.0] * 6, drop_position=[0.0] * 6,
            approach_offset=[-0.5, 0.0, -0.1], pick_offset=[-0.18, 0.06, -0.03],
            pick_orientation=[0.0] * 3, gripper_open_width=100, gripper_close_width=0,
            grip_seconds=0.0, get_logger=Mock(),
            run_detection_at_curr_pos=Mock(return_value=[point]),
            run_detection_at_pos=Mock(return_value=[point]),
            filter_apples_for_pickup=Mock(return_value=[point]),
            send_movement_request=Mock(), send_gripper_request=Mock())
        return DemoRoutine, state

    def test_gripper_stays_closed_until_drop(self):
        routine, state = self.fixture()
        events = []
        state.send_gripper_request.side_effect = lambda width: events.append(('gripper', width))
        state.send_movement_request.side_effect = lambda pose, *args: events.append(('move', pose))
        routine.run_demo(state)
        self.assertEqual([v for kind, v in events if kind == 'gripper'], [100, 0, 100])
        closed = events.index(('gripper', 0))
        released = events.index(('gripper', 100), closed)
        self.assertTrue(any(kind == 'move' for kind, _ in events[closed+1:released]))

    def test_failed_pick_motion_prevents_close_and_transport(self):
        routine, state = self.fixture()
        state.send_movement_request.side_effect = [None, RuntimeError('execution failed')]
        with self.assertRaises(RuntimeError):
            routine.run_demo(state)
        self.assertEqual(state.send_movement_request.call_count, 2)
        state.send_gripper_request.assert_called_once_with(100)

if __name__ == '__main__':
    unittest.main()
