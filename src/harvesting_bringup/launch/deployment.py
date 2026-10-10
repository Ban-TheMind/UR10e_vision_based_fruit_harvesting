"""Hardware-independent deployment contract; also used by offline tests."""
import json
import ipaddress
import math

MODES = ('fake', 'robot', 'camera', 'gripper', 'all', 'demo')

def boolean(value):
    if isinstance(value, bool):
        return value
    if value in ('true', 'false'):
        return value == 'true'
    raise ValueError('Expected true or false, got ' + str(value))

def load_profile(path):
    with open(path, encoding='utf-8') as stream:
        profile = json.load(stream)
    robot, camera, gripper, task = [profile[key] for key in ('robot', 'camera', 'gripper', 'task')]
    ipaddress.ip_address(robot['robot_ip'])
    if robot['ur_type'] != 'ur10e':
        raise ValueError('This deployment is calibrated for UR10e')
    boolean(robot['headless_mode'])
    if not str(camera['serial_no']).isdigit() or camera['profile'] != '640,480,15':
        raise ValueError('Specify the camera serial and validated 640,480,15 profile')
    if not camera['inference_python'] or not gripper['serial_port']:
        raise ValueError('Inference interpreter and serial port are required')
    if gripper['baudrate'] != 115200 or not 1 <= gripper['slave_id'] <= 247:
        raise ValueError('Invalid Robotiq serial configuration')
    if type(task['calibration_verified']) is not bool or type(task['max_cycles']) is not int or task['max_cycles'] < 1:
        raise ValueError('Calibration must be boolean and cycles a positive integer')
    joints = task['birds_eye_joint_pos']
    if len(joints) != 6 or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in joints):
        raise ValueError('Scan joints require six finite radians in standard UR order')
    return profile

def build_plan(profile, mode, motion_enabled=False):
    if mode not in MODES:
        raise ValueError('Unknown launch mode: ' + mode)
    enabled = boolean(motion_enabled)
    if mode == 'fake' and enabled:
        raise ValueError('fake mode is isolated and does not enable physical actuators')
    if mode == 'demo' and not (enabled and profile['task']['calibration_verified']):
        raise ValueError('demo requires motion_enabled:=true and field-verified calibration in the profile')
    # fake deliberately contains no camera, serial server or autonomous routine.
    return {'robot': mode in ('fake', 'robot', 'all', 'demo'),
            'camera': mode in ('camera', 'all', 'demo'),
            'gripper': mode in ('gripper', 'all', 'demo'),
            'routine': mode == 'demo', 'fake': mode == 'fake',
            'controller': 'joint_trajectory_controller' if mode == 'fake' else 'scaled_joint_trajectory_controller',
            'allow_execution': enabled, 'allow_gripper_commands': enabled}
