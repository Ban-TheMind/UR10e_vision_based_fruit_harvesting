"""Pure deployment configuration shared by launch, diagnostics and offline checks.

resolve_deployment is the entry point: resolve overrides, enforce the selected
mode's prerequisites, and return normalized data before any device is started.
This module never imports ROS, queries TF, or claims that a live map is ready.
"""
import copy
from dataclasses import dataclass
from typing import Optional
import ipaddress
import json
import math

MODES = ('fake', 'robot', 'camera', 'gripper', 'all', 'demo', 'mapping')
OVERRIDE_NAMES = ('motion_enabled', 'rviz', 'execution_timeout')
PERCEPTION_DEFAULTS = dict(
    point_cloud_topic='/camera/depth/color/points',
    scene_cloud_topic='/harvesting/scene_points',
    filtered_cloud_topic='/harvesting/filtered_points',
    camera_root_frame='camera_link', octomap_frame='base_link',
    octomap_resolution=0.01, max_range=2.0, max_update_rate=2.0,
    point_subsample=1, padding_offset=0.01, padding_scale=1.0,
    pointcloud_max_age=3.0, scene_service_timeout=2.0)


def perception_parameters(profile):
    """Settings for the dedicated, non-executing mapping commissioning stage."""
    result = dict(section(profile, 'perception', required=False))
    defaults = PERCEPTION_DEFAULTS
    if set(result) - set(defaults):
        raise ValueError('Unknown perception settings: ' + ', '.join(sorted(set(result)-set(defaults))))
    result = dict(defaults, **result)
    for name in ('point_cloud_topic', 'scene_cloud_topic', 'filtered_cloud_topic',
                 'camera_root_frame', 'octomap_frame'):
        if not isinstance(result[name], str) or not result[name] or result[name].strip() != result[name]:
            raise ValueError('Invalid perception ' + name)
    topics = [result[key] for key in ('point_cloud_topic', 'scene_cloud_topic', 'filtered_cloud_topic')]
    if len(set(topics)) != 3 or not all(topic.startswith('/') for topic in topics):
        raise ValueError('Perception topics must be three distinct absolute names')
    if result['camera_root_frame'] in ('base', 'base_link', 'world', 'camera_color_optical_frame'):
        raise ValueError('camera_root_frame must be the RealSense root frame')
    # Current UR model's planning frame is base_link, not controller base.
    if result['octomap_frame'] != 'base_link':
        raise ValueError('Current mapping profile requires planning frame base_link')
    for name in ('octomap_resolution', 'max_range', 'max_update_rate', 'padding_scale',
                 'pointcloud_max_age', 'scene_service_timeout'):
        if type(result[name]) not in (int, float) or not math.isfinite(result[name]) or result[name] <= 0:
            raise ValueError('Perception ' + name + ' must be finite and positive')
        result[name] = float(result[name])
    if not 0 < result['max_update_rate'] <= 30:
        raise ValueError('Perception max_update_rate must be <= 30 Hz')
    if type(result['point_subsample']) is not int or result['point_subsample'] < 1:
        raise ValueError('point_subsample must be an integer >= 1')
    padding = result['padding_offset']
    if type(padding) not in (int, float) or not math.isfinite(padding) or padding < 0:
        raise ValueError('padding_offset must be finite and nonnegative')
    result['padding_offset'] = float(padding)
    return result


def boolean(value):
    if isinstance(value, bool):
        return value
    if value in ('true', 'false'):
        return value == 'true'
    raise ValueError('Expected true or false, got ' + str(value))


def execution_timeout(profile, override=''):
    value = override if override != '' else section(profile, 'task', required=False).get('execution_timeout', 600.0)
    if isinstance(value, bool):
        raise ValueError('execution_timeout must be finite positive seconds')
    seconds = float(value)
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError('execution_timeout must be finite positive seconds')
    return seconds


def section(profile, name, required=True):
    if name not in profile:
        if required:
            raise ValueError('Missing configuration section: ' + name)
        return {}
    value = profile[name]
    if not isinstance(value, dict):
        raise ValueError('Configuration section must be an object: ' + name)
    return value


def required_values(values, names, label):
    missing = [name for name in names if name not in values]
    if missing:
        raise ValueError('Missing ' + label + ' settings: ' + ', '.join(missing))


def finite_vector(value, size, label):
    if (not isinstance(value, (list, tuple)) or len(value) != size or
            not all(type(x) in (int, float) and math.isfinite(x) for x in value)):
        raise ValueError(label + ' requires ' + str(size) + ' finite numbers')


def load_profile(path):
    """Read JSON; device/experiment prerequisites are checked when resolving a mode."""
    with open(path, encoding='utf-8') as stream:
        profile = json.load(stream)
    if not isinstance(profile, dict):
        raise ValueError('Deployment profile must be a JSON object')
    return profile


def parse_overrides(arguments):
    result = {}
    for argument in arguments:
        if ':=' not in argument:
            raise ValueError('Expected NAME:=VALUE, got ' + argument)
        name, value = argument.split(':=', 1)
        if name not in OVERRIDE_NAMES:
            raise ValueError('Supported overrides: motion_enabled:=true/false rviz:=true/false execution_timeout:=SECONDS')
        if name in result:
            raise ValueError('Duplicate override: ' + name)
        result[name] = value
    return result


def mode_plan(mode, motion_enabled):
    if mode not in MODES:
        raise ValueError('Unknown launch mode: ' + str(mode))
    enabled = boolean(motion_enabled)
    if mode == 'fake' and enabled:
        raise ValueError('fake mode is isolated and does not enable physical actuators')
    if mode == 'mapping' and enabled:
        raise ValueError('mapping is plan-only: motion_enabled must remain false')
    if mode == 'demo' and not enabled:
        raise ValueError('demo requires motion_enabled:=true')
    return {'robot': mode in ('fake', 'robot', 'all', 'demo', 'mapping'),
            'camera': mode in ('camera', 'all', 'demo', 'mapping'),
            'mapping': mode == 'mapping',
            'gripper': mode in ('gripper', 'all', 'demo'),
            'routine': mode == 'demo', 'fake': mode == 'fake',
            'controller': 'joint_trajectory_controller' if mode == 'fake' else 'scaled_joint_trajectory_controller',
            'allow_execution': enabled, 'allow_gripper_commands': enabled}


def validate_robot(profile, fake):
    robot = section(profile, 'robot', required=not fake)
    if fake:
        robot = dict({'ur_type': 'ur10e', 'robot_ip': '0.0.0.0', 'headless_mode': False}, **robot)
    else:
        required_values(robot, ('ur_type', 'robot_ip', 'headless_mode'), 'robot')
        ipaddress.ip_address(robot['robot_ip'])
    if robot['ur_type'] != 'ur10e':
        raise ValueError('This deployment is calibrated for UR10e')
    robot['headless_mode'] = boolean(robot['headless_mode'])
    profile['robot'] = robot


def validate_camera(profile, mapping):
    camera = section(profile, 'camera')
    required_values(camera, ('serial_no', 'profile'), 'camera')
    if not str(camera['serial_no']).isdigit() or camera['profile'] != '640,480,15':
        raise ValueError('Specify the camera serial and validated 640,480,15 profile')
    camera.setdefault('calibration_file', '')
    if not isinstance(camera['calibration_file'], str):
        raise ValueError('camera.calibration_file must be a path string or empty')
    if not mapping:
        value = camera.get('inference_python')
        if not isinstance(value, str) or not value.strip():
            raise ValueError('Inference interpreter is required for camera detection')


def validate_gripper(profile):
    gripper = section(profile, 'gripper')
    required_values(gripper, ('serial_port', 'baudrate', 'slave_id'), 'gripper')
    if not isinstance(gripper['serial_port'], str) or not gripper['serial_port'].strip():
        raise ValueError('Gripper serial port is required')
    if (type(gripper['baudrate']) is not int or gripper['baudrate'] != 115200 or
            type(gripper['slave_id']) is not int or not 1 <= gripper['slave_id'] <= 247):
        raise ValueError('Invalid Robotiq serial configuration')


def validate_task(profile):
    task = section(profile, 'task')
    required_values(task, ('calibration_verified', 'max_cycles', 'birds_eye_joint_pos'), 'task')
    if (type(task['calibration_verified']) is not bool or
            type(task['max_cycles']) is not int or task['max_cycles'] < 1):
        raise ValueError('Calibration must be boolean and cycles a positive integer')
    finite_vector(task['birds_eye_joint_pos'], 6, 'Scan joints')
    for name in ('scan_at_home_only', 'return_to_scan_after_drop'):
        if name in task and type(task[name]) is not bool:
            raise ValueError(name + ' must be boolean')
    if task.get('drop_motion', 'cartesian') not in ('joint', 'cartesian'):
        raise ValueError('drop_motion must be joint or cartesian')
    if 'drop_joint_pos' in task or task.get('drop_motion') == 'joint':
        finite_vector(task.get('drop_joint_pos', []), 6, 'Drop joints')
    if task.get('grasp_strategy', 'side') not in ('side', 'tabletop'):
        raise ValueError('grasp_strategy must be side or tabletop')


def tabletop_parameters(profile, required=False):
    if section(profile, 'task', required=False).get('grasp_strategy', 'side') != 'tabletop':
        return {}
    geometry = section(profile, 'tabletop', required=False)
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


@dataclass
class Deployment:
    profile: dict
    plan: dict
    rviz: bool
    perception: dict
    tabletop: dict
    execution_timeout: Optional[float]


def resolve_deployment(source, mode, overrides=None):
    """Resolve only the selected mode's prerequisites; never touch hardware.

    Priority: explicit supported override, then profile value, then default.
    A valid override cannot relax the fake/mapping execution prohibitions.
    Inactive sections are left unvalidated and are never used by the launch.
    """
    overrides = dict(overrides or {})
    unknown = set(overrides) - set(OVERRIDE_NAMES)
    if unknown:
        raise ValueError('Unknown deployment overrides: ' + ', '.join(sorted(unknown)))
    profile = copy.deepcopy(source if isinstance(source, dict) else load_profile(source))
    plan = mode_plan(mode, overrides.get('motion_enabled', False))
    rviz = boolean(overrides.get('rviz', True))
    timeout_override = overrides.get('execution_timeout', '')
    timeout = execution_timeout(profile, timeout_override) if plan['routine'] else None
    if not plan['routine'] and timeout_override != '':
        timeout = execution_timeout({}, timeout_override)
    if plan['robot']:
        validate_robot(profile, plan['fake'])
    if plan['camera']:
        validate_camera(profile, plan['mapping'])
    if plan['gripper']:
        validate_gripper(profile)
    perception, tabletop = {}, {}
    if plan['mapping']:
        perception = perception_parameters(profile)
        task = section(profile, 'task', required=False)
        verified = task.get('calibration_verified', False)
        if type(verified) is not bool:
            raise ValueError('calibration_verified must be boolean')
        profile['task'] = dict(task, calibration_verified=verified)
    if plan['routine']:
        validate_task(profile)
        tabletop = tabletop_parameters(profile, required=True)
    return Deployment(profile, plan, rviz, perception, tabletop, timeout)


def above_check_settings(source, execute=False):
    """Validate only what the existing above-target diagnostic actually uses."""
    profile = copy.deepcopy(source if isinstance(source, dict) else load_profile(source))
    geometry = tabletop_parameters(profile)
    execution_timeout(profile)
    if execute:
        robot = section(profile, 'robot')
        required_values(robot, ('robot_ip',), 'robot')
        ipaddress.ip_address(robot['robot_ip'])
        task = section(profile, 'task')
        finite_vector(task.get('birds_eye_joint_pos', []), 6, 'Scan joints')
    return profile, geometry


def perception_launch_defaults():
    """ROS-independent string encoding for the existing Foxy launch arguments."""
    return {name: str(value) for name, value in PERCEPTION_DEFAULTS.items()
            if name != 'camera_root_frame'}


def moveit_mapping_parameters(arguments, sensor_template, allow_execution=False):
    """Translate existing launch arguments using the same JSON-profile rules."""
    if boolean(allow_execution):
        raise ValueError('Point-cloud commissioning is plan-only; allow_execution must be false')
    values = dict(arguments)
    for key, default in PERCEPTION_DEFAULTS.items():
        if key not in values:
            continue
        if type(default) is float:
            if isinstance(values[key], bool):
                raise ValueError(key + ' must be a number')
            values[key] = float(values[key])
        elif type(default) is int:
            raw = values[key]
            if isinstance(raw, bool) or (not isinstance(raw, str) and type(raw) is not int):
                raise ValueError(key + ' must be an integer')
            values[key] = int(raw)
    values = perception_parameters({'perception': values})
    if (not isinstance(sensor_template, dict) or sensor_template.get('sensors') != ['harvesting_pointcloud'] or
            not isinstance(sensor_template.get('harvesting_pointcloud'), dict) or
            sensor_template['harvesting_pointcloud'].get('sensor_plugin') != 'occupancy_map_monitor/PointCloudOctomapUpdater'):
        raise ValueError('Missing or invalid project sensors_3d.yaml')
    scene = copy.deepcopy(sensor_template)
    sensor = scene['harvesting_pointcloud']
    sensor.update({key: values[key] for key in ('max_range', 'max_update_rate', 'point_subsample',
                   'padding_offset', 'padding_scale', 'filtered_cloud_topic')})
    sensor['point_cloud_topic'] = values['scene_cloud_topic']
    scene.update({key: values[key] for key in ('octomap_frame', 'octomap_resolution')})
    scene['allow_trajectory_execution'] = False
    guard = {key: values[key] for key in ('point_cloud_topic', 'filtered_cloud_topic',
             'octomap_frame', 'octomap_resolution', 'pointcloud_max_age', 'scene_service_timeout')}
    return scene, guard
