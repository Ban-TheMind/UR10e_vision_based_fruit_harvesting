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

def execution_timeout(profile, override=''):
    value = override if override != '' else profile['task'].get('execution_timeout', 600.0)
    if isinstance(value, bool):
        raise ValueError('execution_timeout must be finite positive seconds')
    seconds = float(value)
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError('execution_timeout must be finite positive seconds')
    return seconds

def tabletop_parameters(profile, required=False):
    if profile['task'].get('grasp_strategy', 'side') != 'tabletop':
        return {}
    geometry = profile.get('tabletop', {})
    missing = [name for name in ('grasp_height_m', 'tool_rpy_rad', 'xy_offset_m')
               if geometry.get(name) is None]
    if required and missing:
        raise ValueError('桌面抓取参数尚未填写: ' + ', '.join(missing) +
                         '；到现场后填写 site.json 的 tabletop；不是标定确认开关')
    result = {}
    for name in ('grasp_height_m', 'tool_rpy_rad', 'xy_offset_m', 'clearance_m', 'tool_offset_m'):
        defaults = {'clearance_m': 0.12, 'tool_offset_m': [0.0, 0.0, 0.1628]}
        value = geometry.get(name, defaults.get(name))
        if value is None:
            continue
        size = {'tool_rpy_rad': 3, 'xy_offset_m': 2, 'tool_offset_m': 3}.get(name)
        values = value if size else [value]
        if (size and (not isinstance(value, list) or len(value) != size)):
            raise ValueError('Invalid tabletop ' + name)
        if not all(type(x) in (int, float) and math.isfinite(x) for x in values):
            raise ValueError('Tabletop ' + name + ' requires finite numbers')
        if name == 'clearance_m' and value <= 0:
            raise ValueError('Tabletop clearance_m must be positive')
        result[name] = [float(x) for x in value] if size else float(value)
    return result

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
    for name in ('scan_at_home_only', 'return_to_scan_after_drop'):
        if name in task and type(task[name]) is not bool:
            raise ValueError(name + ' must be boolean')
    if task.get('drop_motion', 'cartesian') not in ('joint', 'cartesian'):
        raise ValueError('drop_motion must be joint or cartesian')
    if 'drop_joint_pos' in task or task.get('drop_motion') == 'joint':
        drop = task.get('drop_joint_pos', [])
        if len(drop) != 6 or not all(type(x) in (int, float) and math.isfinite(x) for x in drop):
            raise ValueError('Drop joints require six finite radians in standard UR order')
    execution_timeout(profile)
    if task.get('grasp_strategy', 'side') not in ('side', 'tabletop'):
        raise ValueError('grasp_strategy must be side or tabletop')
    tabletop_parameters(profile)
    return profile

def build_plan(profile, mode, motion_enabled=False):
    if mode not in MODES:
        raise ValueError('Unknown launch mode: ' + mode)
    enabled = boolean(motion_enabled)
    if mode == 'fake' and enabled:
        raise ValueError('fake mode is isolated and does not enable physical actuators')
    if mode == 'demo' and not enabled:
        raise ValueError('demo requires motion_enabled:=true')
    if mode == 'demo':
        tabletop_parameters(profile, required=True)
    # fake deliberately contains no camera, serial server or autonomous routine.
    return {'robot': mode in ('fake', 'robot', 'all', 'demo'),
            'camera': mode in ('camera', 'all', 'demo'),
            'gripper': mode in ('gripper', 'all', 'demo'),
            'routine': mode == 'demo', 'fake': mode == 'fake',
            'controller': 'joint_trajectory_controller' if mode == 'fake' else 'scaled_joint_trajectory_controller',
            'allow_execution': enabled, 'allow_gripper_commands': enabled}
