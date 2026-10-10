#!/usr/bin/env python3
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as S
import unittest
import contextlib
import io
import sys
import types
from unittest.mock import patch, Mock

spec = importlib.util.spec_from_file_location("entry", Path(__file__).with_name("ur10e_control.py"))
entry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(entry)


def trajectory():
    q = [math.radians(v) for v in entry.HOME]
    points = [S(positions=q[:], velocities=[], accelerations=[], time_from_start=S(sec=t, nanosec=0))
              for t in (0, 10)]
    return S(joint_trajectory=S(joint_names=entry.JOINTS[:], points=points),
             multi_dof_joint_trajectory=S(points=[])), q


class EntryTests(unittest.TestCase):
    def test_home_and_path_within_limits(self):
        path, actual = trajectory()
        for angle, target in zip(entry.check_angles(actual), entry.HOME):
            self.assertAlmostEqual(angle, target)
        self.assertEqual(entry.check_trajectory(path, actual), 10)

    def test_intermediate_excursion_is_rejected(self):
        path, actual = trajectory()
        middle = S(positions=actual[:], velocities=[], accelerations=[], time_from_start=S(sec=5, nanosec=0))
        middle.positions[0] = math.radians(10.56)
        path.joint_trajectory.points.insert(1, middle)
        with self.assertRaises(RuntimeError):
            entry.check_trajectory(path, actual)

    def test_wrong_order_endpoint_or_stale_start_is_rejected(self):
        path, actual = trajectory()
        path.joint_trajectory.joint_names.reverse()
        with self.assertRaises(RuntimeError):
            entry.check_trajectory(path, actual)
        path, actual = trajectory()
        path.joint_trajectory.points[-1].positions[-1] = 1.
        with self.assertRaises(RuntimeError):
            entry.check_trajectory(path, actual)
        path, actual = trajectory()
        actual[0] += 0.02
        with self.assertRaises(RuntimeError):
            entry.check_trajectory(path, actual)

    def test_stopped_zero_speed_and_excess_speed_are_rejected(self):
        state = dict(runtime_state=2, speed_scaling=0.05, target_fraction=1.,
                     protective=False, emergency=False, q=[math.radians(v) for v in entry.HOME])
        entry.check_running(state)
        for key, value in (("runtime_state", 1), ("speed_scaling", 0), ("speed_scaling", 1.01), ("speed_scaling", float("nan")),
                           ("protective", True), ("emergency", True)):
            changed = dict(state)
            changed[key] = value
            with self.assertRaises(RuntimeError):
                entry.check_running(changed)

    def test_all_positive_speed_fractions_up_to_full_speed(self):
        for speed in (0.001, 0.05, 0.5, 1.0):
            entry.check_running(dict(runtime_state=2, speed_scaling=speed, target_fraction=1.,
                                     protective=False, emergency=False,
                                     q=[math.radians(v) for v in entry.HOME]))

    def test_timeout_options_and_trajectory_budget(self):
        defaults = entry.parse_options([])
        self.assertEqual((defaults.execution_timeout, defaults.stall_timeout), (600, 15))
        options = entry.parse_options(['--home', '--execution-timeout', '1200', '--stall-timeout', '30'])
        self.assertEqual((options.execution_timeout, options.stall_timeout), (1200, 30))
        for flags in (['--execution-timeout', '0'], ['--execution-timeout', 'nan'],
                      ['--stall-timeout', '-1'], ['--execution-timeout', 'inf'],
                      ['--execution-timeout', '10', '--stall-timeout', '15']):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                entry.parse_options(flags)
        path, actual = trajectory()
        with self.assertRaises(RuntimeError):
            entry.check_trajectory(path, actual, execution_timeout=5)
        self.assertEqual(entry.check_trajectory(path, actual, execution_timeout=20), 10)

    def test_monitoring_requires_set_acceptance_and_boolean_readback(self):
        class Message:
            def __init__(self, **kwargs): self.__dict__.update(kwargs)
        srv = types.ModuleType('rcl_interfaces.srv')
        srv.SetParameters = srv.GetParameters = S(Request=Message)
        msg = types.ModuleType('rcl_interfaces.msg')
        msg.Parameter = msg.ParameterValue = Message
        msg.ParameterType = S(PARAMETER_BOOL=1)
        modules = {'rcl_interfaces': types.ModuleType('rcl_interfaces'),
                   'rcl_interfaces.srv': srv, 'rcl_interfaces.msg': msg}
        for accepted, value_type, value, succeeds in ((True, 1, False, True),
                (False, 1, False, False), (True, 1, True, False), (True, 0, False, False)):
            setter, getter = Mock(), Mock()
            node = Mock()
            node.create_client.side_effect = [setter, getter]
            responses = [S(results=[S(successful=accepted)]),
                         S(values=[S(type=value_type, bool_value=value)])]
            with patch.dict(sys.modules, modules), patch.object(entry, 'wait_future', side_effect=responses):
                if succeeds:
                    entry.configure_execution_monitoring(node)
                    parameter = setter.call_async.call_args[0][0].parameters[0]
                    self.assertEqual(parameter.name, 'trajectory_execution.execution_duration_monitoring')
                    self.assertFalse(parameter.value.bool_value)
                    self.assertEqual(node.create_client.call_args_list[0].args[1],
                                     '/moveit_simple_controller_manager/set_parameters')
                else:
                    with self.assertRaises(RuntimeError): entry.configure_execution_monitoring(node)
                self.assertEqual(node.destroy_client.call_count, 2)

    def test_nonfinite_angles_and_nonmonotonic_time_are_rejected(self):
        path, actual = trajectory()
        path.joint_trajectory.points[0].positions[2] = float("nan")
        with self.assertRaises(RuntimeError):
            entry.check_trajectory(path, actual)
        path, actual = trajectory()
        path.joint_trajectory.points[1].time_from_start.sec = 0
        with self.assertRaises(RuntimeError):
            entry.check_trajectory(path, actual)


if __name__ == "__main__":
    unittest.main()
