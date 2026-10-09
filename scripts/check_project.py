"""Read-only preflight. Does not initialize ROS nodes or contact hardware."""
import ast
import importlib
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory


def validate_profile(config):
    deployment = config['deployment']
    for name in ('robot_ip', 'camera_serial'):
        if not isinstance(deployment[name], str) or not deployment[name]:
            raise ValueError(f'deployment.{name} must be a nonempty string')
    for name in ('calibration_verified', 'publish_camera_tf'):
        if not isinstance(deployment.get(name), bool):
            raise ValueError(f'deployment.{name} must be boolean')
    camera = config['camera_server']['ros__parameters']
    if (camera['coordinate_mode'] != 'eye_to_hand' or deployment.get('camera_mount') != 'external'
            or camera['camera_frame'] != 'camera_color_optical_frame' or camera['target_frame'] != 'base_link'):
        raise ValueError('Expected external eye_to_hand camera, color optical input and base_link output')
    planner = config['moveit_path_planning_server']['ros__parameters']
    if len(set(planner['joint_names'])) != 6:
        raise ValueError('Expected six unique joint names')
    if not 1 <= planner['max_planning_attempts'] <= 10:
        raise ValueError('Planning attempts must be in 1..10')
    for name in ('velocity_scaling', 'acceleration_scaling'):
        if not 0 < planner[name] <= 1:
            raise ValueError(f'{name} must be in (0, 1]')
    for name in ('back_wall', 'side_wall', 'table', 'ceiling'):
        value = planner[name]
        if len(value) != 6 or not all(math.isfinite(x) for x in value) or any(x <= 0 for x in value[:3]):
            raise ValueError(f'Invalid scene geometry: {name}')
    gripper = config['gripper_server']['ros__parameters']
    if (not gripper['serial_port'] or gripper['baudrate'] <= 0
            or not 1 <= gripper['slave_id'] <= 247
            or not 0 <= gripper['speed'] <= 255
            or not 0.001 <= gripper['serial_timeout'] <= 3600
            or not all(math.isfinite(gripper[key]) and gripper[key] > 0
                       for key in ('serial_timeout', 'action_timeout'))):
        raise ValueError('Invalid gripper configuration')
    task = config['demo_routine']['ros__parameters']
    for name, size in {'bird_eye_position': 6, 'birds_eye_joint_pos': 6, 'drop_position': 6, 'approach_offset': 3, 'pick_offset': 3, 'pick_orientation': 3, 'retry_scan_delta': 6, 'retry_grip_delta': 6}.items():
        if len(task[name]) != size or not all(math.isfinite(x) for x in task[name]):
            raise ValueError(f'Invalid task parameter: {name}')
    for name in ('service_timeout', 'startup_timeout', 'max_cycles', 'max_attempts', 'max_detect_attempts'):
        if task[name] <= 0:
            raise ValueError(f'{name} must be positive')
    if not 0 <= task['detection_min_confidence'] <= task['detection_confidence'] <= 1:
        raise ValueError('Invalid detection confidence')
    if not 0 <= task['gripper_open_width'] <= 85 or not 0 <= task['gripper_close_width'] <= 85 or not 0 <= task['gripper_force'] <= 255:
        raise ValueError('Invalid gripper task parameters')


def main():
    root = Path(__file__).resolve().parent.parent
    profile = Path(sys.argv[1])
    errors = []
    def check(name, operation):
        try:
            operation()
            print(f'PASS {name}')
        except Exception as error:
            errors.append(name)
            print(f'FAIL {name}: {error}')
    config = yaml.safe_load(profile.read_text())
    if '--profile-only' in sys.argv:
        validate_profile(config)
        return 0
    check('profile', lambda: validate_profile(config))
    def sources():
        for file in (root / 'src').rglob('*.py'):
            ast.parse(file.read_text(), filename=str(file))
        for file in (root / 'src').rglob('package.xml'):
            ET.parse(file)
    check('source syntax / package manifests', sources)
    for name in ('rclpy', 'cv_bridge', 'pyrealsense2', 'tf2_geometry_msgs', 'tf_transformations', 'message_filters'):
        check(f'import {name}', lambda name=name: importlib.import_module(name))
    for name in ('harvesting_bringup', 'camera', 'gripper', 'robotiq_sdk_bridge', 'demo_package', 'moveit_path_planner', 'ur_robot_driver', 'ur10e_moveit_config_official', 'realsense2_camera'):
        check(f'installed package {name}', lambda name=name: get_package_share_directory(name))
    from camera.geometry import load_calibration
    check('fixed camera calibration frames / matrix', lambda: load_calibration(config['deployment'].get('camera_calibration_file') or config['camera_server']['ros__parameters'].get('calibration_file') or None))
    def sdk_binary():
        from ament_index_python.packages import get_package_prefix
        import os
        binary = Path(get_package_prefix('robotiq_sdk_bridge')) / 'lib/robotiq_sdk_bridge/robotiq_sdk_command'
        if not os.access(binary, os.X_OK):
            raise ValueError('Official Robotiq SDK bridge missing; rebuild workspace')
    check('official Robotiq SDK executable (no device access)', sdk_binary)
    def model_file():
        model = Path(config['camera_server']['ros__parameters']['model_file'])
        if not model.is_absolute():
            model = Path(get_package_share_directory('camera')) / 'models' / model
        if not model.is_file() or model.stat().st_size < 1024:
            raise ValueError(f'Model absent or Git LFS pointer: {model}')
    check('YOLO model file', model_file)
    def robot_model():
        model = Path(get_package_share_directory('end_effector_description')) / ('urdf/end_effector_external.xacro' if config['deployment'].get('camera_mount') == 'external' else 'urdf/end_effector_withDriverSupport.xacro')
        calibration = config['deployment']['robot_calibration_file'] or str(model.parent.parent / 'etc/robot_calibration.yaml')
        result = subprocess.run(['xacro', str(model), 'ur_type:=ur10e', 'name:=ur',
                                 'robot_ip:=192.0.2.1', 'use_fake_hardware:=true',
                                 f'kinematics_params:={calibration}'], capture_output=True, text=True, timeout=30, check=True)
        robot = ET.fromstring(result.stdout)
        joints = {joint.attrib['name'] for joint in robot.findall('joint')}
        if not set(config['moveit_path_planning_server']['ros__parameters']['joint_names']) <= joints:
            raise ValueError('Configured joints are absent from the robot model')
        if 'simple_ee_link' not in {link.attrib['name'] for link in robot.findall('link')}:
            raise ValueError('End-effector missing from robot model')
        if 'camera_link' in {link.attrib['name'] for link in robot.findall('link')}:
            raise ValueError('Fixed camera must not be attached to robot model')
        base_joint = next(joint for joint in robot.findall('joint')
                          if joint.find('parent').get('link') == 'base_link' and joint.find('child').get('link') == 'base')
        origin = base_joint.find('origin')
        rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        if any(abs(v) > 1e-9 for v in xyz + rpy[:2]) or not math.isclose(abs(rpy[2]), math.pi, abs_tol=1e-9):
            raise ValueError('UR base/base_link relation differs from camera conversion')
    check('UR10e model expansion / joints / end effector', robot_model)
    print('FIELD STATUS: calibration not verified; automatic harvesting unavailable' if not config['deployment']['calibration_verified'] else 'FIELD STATUS: marked verified in profile; this check does not prove physical calibration')
    return bool(errors)


if __name__ == '__main__':
    sys.exit(main())
