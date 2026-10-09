"""Deprecated entry. Use harvesting_bringup system.launch.py with an explicit mode."""
from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription([LogInfo(msg='Use scripts/project camera, gripper, robot or all; auxiliary no longer starts devices implicitly.')])
