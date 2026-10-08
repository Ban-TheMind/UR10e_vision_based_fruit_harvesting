"""Launch isolated subsystems; never start an autonomous routine implicitly."""
from pathlib import Path
import json
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, filename, arguments):
    path = Path(get_package_share_directory(package)) / 'launch' / filename
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                    launch_arguments=arguments.items())


def start(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('fake', 'robot', 'camera', 'gripper', 'all', 'demo'):
        raise ValueError('mode must be fake, robot, camera, gripper, all or demo')
    profile = Path(LaunchConfiguration('profile').perform(context)).resolve()
    config = yaml.safe_load(profile.read_text())
    deployment = config['deployment']
    enabled = LaunchConfiguration('motion_enabled').perform(context).lower()
    if enabled not in ('true', 'false'):
        raise ValueError('motion_enabled must be true or false')
    motion = enabled == 'true'
    rviz = LaunchConfiguration('rviz').perform(context)
    if deployment.get('camera_mount', 'wrist') not in ('wrist', 'external'):
        raise ValueError('camera_mount must be wrist or external')
    if deployment.get('calibration_verified') and config['camera_server']['ros__parameters']['coordinate_mode'] != 'tf':
        raise ValueError('Verified calibration requires explicit tf coordinate mode')
    actions = []
    if mode in ('fake', 'robot', 'all'):
        fake = mode == 'fake'
        calibration = deployment.get('robot_calibration_file') or str(
            Path(get_package_share_directory('end_effector_description')) / 'etc/robot_calibration.yaml')
        description = str(Path(get_package_share_directory('end_effector_description')) /
                          ('urdf/end_effector_external.xacro' if deployment.get('camera_mount') == 'external' else 'urdf/end_effector_withDriverSupport.xacro'))
        if fake:
            actions.append(include('harvesting_bringup', 'fake_robot.launch.py', {
                'description_file': description, 'kinematics_params_file': calibration,
            }))
        else:
            actions.append(include('ur_robot_driver', 'ur_control.launch.py', {
                'ur_type': 'ur10e', 'robot_ip': str(deployment['robot_ip']),
                'use_fake_hardware': 'false', 'launch_rviz': 'false',
                'description_file': description, 'kinematics_params_file': calibration,
                'initial_joint_controller': 'scaled_joint_trajectory_controller',
            }))
        actions.append(TimerAction(period=4.0, actions=[include(
            'ur10e_moveit_config_official', 'ur_moveit.launch.py', {
                'ur_type': 'ur10e', 'use_fake_hardware': str(fake).lower(),
                'launch_rviz': rviz, 'launch_servo': 'false',
                'camera_mount': deployment.get('camera_mount', 'wrist'),
                'description_file': description, 'kinematics_params_file': calibration,
                })]))
        # MoveGroup must be available before constructing the planning module.
        actions.append(TimerAction(period=8.0, actions=[Node(
            package='moveit_path_planner', executable='moveit_path_planning_server',
            name='moveit_path_planning_server', output='screen',
            parameters=[config['moveit_path_planning_server']['ros__parameters'], {'execute_enabled': motion}],
        )]))
    if mode in ('camera', 'all'):
        if deployment.get('publish_camera_tf', False):
            source = deployment.get('camera_source_frame')
            target = deployment.get('camera_target_frame')
            params = config['camera_server']['ros__parameters']
            if not deployment.get('calibration_verified') or not source or not target:
                raise ValueError('Static camera TF requires verified calibration and explicit frame names')
            if params['coordinate_mode'] != 'tf' or params['camera_frame'] != source or params['target_frame'] != target:
                raise ValueError('Camera TF frames must match camera parameters in tf mode')
            matrix_file = deployment.get('camera_calibration_file') or str(profile.parent / 'calibration/camera_to_base.json')
            if deployment.get('camera_mount') != 'external' or deployment.get('camera_publish_tf', True):
                raise ValueError('Optical-frame static calibration requires external mount and camera_publish_tf: false to avoid duplicate TF parents')
            data = json.loads(Path(matrix_file).read_text())
            import numpy as np
            from tf_transformations import quaternion_from_matrix
            rotation = np.asarray(data['rotation'], dtype=float)
            if rotation.shape != (3, 3) or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3) or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-3):
                raise ValueError('Invalid calibration rotation')
            translation = data['translation_m']
            if len(translation) != 3 or not np.all(np.isfinite(translation)):
                raise ValueError('Invalid calibration translation')
            matrix = np.eye(4)
            matrix[:3, :3] = rotation
            quaternion = quaternion_from_matrix(matrix)
            args = []
            for flag, value in zip(('--x', '--y', '--z', '--qx', '--qy', '--qz', '--qw'), [*translation, *quaternion]):
                args.extend([flag, str(value)])
            actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                                arguments=args + ['--frame-id', target, '--child-frame-id', source]))
        actions.append(include('realsense2_camera', 'rs_launch.py', {
            'serial_no': str(deployment['camera_serial']), 'publish_tf': str(deployment.get('camera_publish_tf', True)).lower(), 'enable_color': 'true',
            'enable_depth': 'true', 'align_depth.enable': 'true', 'enable_sync': 'true',
            'enable_rgbd': 'true', 'pointcloud.enable': 'true',
            'rgb_camera.color_profile': '640x480x5', 'depth_module.depth_profile': '640x480x5',
        }))
        actions.append(Node(package='camera', executable='camera_node',
                            name='camera_server', parameters=[config['camera_server']['ros__parameters']], output='screen'))
    if mode in ('gripper', 'all'):
        actions.append(Node(package='gripper', executable='gripper_server',
                            name='gripper_server', parameters=[config['gripper_server']['ros__parameters'], {'commands_enabled': motion}], output='screen'))
    if mode == 'demo':
        if config['camera_server']['ros__parameters']['target_frame'] != config['moveit_path_planning_server']['ros__parameters']['pose_reference_frame']:
            raise ValueError('Camera target frame must match planner pose_reference_frame')
        if not motion or not deployment.get('calibration_verified', False):
            raise ValueError('Demo requires motion_enabled:=true and deployment.calibration_verified: true')
        actions.append(Node(package='demo_package', executable='vertical_fruit_gripping_demo',
                            name='demo_routine', output='screen',
                            parameters=[config['demo_routine']['ros__parameters'], {'motion_enabled': True, 'calibration_verified': True}]))
    return actions


def generate_launch_description():
    default_profile = str(Path(get_package_share_directory('harvesting_bringup')) / 'config/lab.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='fake'),
        DeclareLaunchArgument('profile', default_value=default_profile),
        DeclareLaunchArgument('motion_enabled', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=start),
    ])
