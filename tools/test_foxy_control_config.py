#!/usr/bin/env python3
"""Exercise the real URDF compatibility function without importing ROS launch."""
import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "src/end_effector_description/launch/foxy_ur_control.launch.py"
tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
function = next(node for node in tree.body
                if isinstance(node, ast.FunctionDef) and node.name == "configure_headless_mode")
namespace = {}
exec(compile(ast.Module(body=[function], type_ignores=[]), str(SOURCE), "exec"), namespace)
configure = namespace["configure_headless_mode"]

URDF = """<robot name="ur10e"><link name="tool0"/><ros2_control name="URSystem" type="system">
<hardware><plugin>ur_robot_driver/URPositionHardwareInterface</plugin>
<param name="headless_mode">0</param><param name="reverse_port">50001</param>
<param name="robot_ip">192.168.11.60</param></hardware></ros2_control></robot>"""


class HeadlessModeTests(unittest.TestCase):
    def test_installed_foxy_hardcoded_zero_is_overridden(self):
        original = ET.fromstring(URDF)
        result = ET.fromstring(configure(URDF, True, False))
        parameter = result.find("./ros2_control/hardware/param[@name='headless_mode']")
        self.assertEqual(parameter.text, "1")
        parameter.text = "0"
        self.assertEqual(ET.tostring(result), ET.tostring(original))

    def test_external_control_mode_remains_available(self):
        result = configure(URDF.replace(">0</param>", ">1</param>"), False, False)
        self.assertEqual(ET.fromstring(result).findtext(
            "./ros2_control/hardware/param[@name='headless_mode']"), "0")

    def test_fake_hardware_is_unchanged(self):
        fake = URDF.replace("ur_robot_driver/URPositionHardwareInterface", "fake_components/GenericSystem")
        self.assertEqual(configure(fake, True, True), fake)

    def test_missing_or_ambiguous_hardware_fails_before_start(self):
        with self.assertRaises(ValueError):
            configure(URDF.replace("headless_mode", "unknown"), True, False)
        with self.assertRaises(ValueError):
            configure(URDF.replace("URPositionHardwareInterface", "Unknown"), True, False)
        with self.assertRaises(ValueError):
            configure(URDF.replace('</hardware>', '<param name="headless_mode">0</param></hardware>'), True, False)


if __name__ == "__main__":
    unittest.main()
