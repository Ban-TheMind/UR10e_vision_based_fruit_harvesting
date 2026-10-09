#!/usr/bin/env python3
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as S
import unittest

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
        entry.check_running(state, True)
        for key, value in (("runtime_state", 1), ("speed_scaling", 0), ("speed_scaling", 1),
                           ("protective", True), ("emergency", True)):
            changed = dict(state)
            changed[key] = value
            with self.assertRaises(RuntimeError):
                entry.check_running(changed, True)

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
