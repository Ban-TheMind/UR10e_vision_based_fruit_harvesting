"""Compose individual field stages without starting a routine by default."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from harvesting_config import deployment

def include(package, filename, arguments):
    path = os.path.join(get_package_share_directory(package), 'launch', filename)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=arguments.items())

def compose(context):
    resolved = deployment.resolve_deployment(
        LaunchConfiguration('profile').perform(context),
        LaunchConfiguration('mode').perform(context),
        {name: LaunchConfiguration(name).perform(context) for name in deployment.OVERRIDE_NAMES})
    profile, plan = resolved.profile, resolved.plan
    rviz, perception = resolved.rviz, resolved.perception
    actions = []
    if plan['robot']:
        common = {key: str(value).lower() if isinstance(value, bool) else str(value)
                  for key, value in profile['robot'].items()}
        common.update({'use_fake_hardware': str(plan['fake']).lower(),
                       'robot_controller': plan['controller'], 'launch_rviz': 'false'})
        actions.append(include('end_effector_description', 'foxy_ur_control.launch.py', common))
        moveit = dict(common, launch_rviz=str(rviz).lower(),
                      launch_planning_server='true',
                      allow_execution=str(plan['allow_execution']).lower())
        moveit['pointcloud_avoidance'] = str(plan['mapping']).lower()
        if plan['mapping']:
            moveit.update({key: str(value) for key, value in perception.items()
                           if key != 'camera_root_frame'})
        actions.append(TimerAction(period=4.0, actions=[
            include('end_effector_description', 'foxy_moveit.launch.py', moveit)]))
    if plan['camera']:
        camera = profile['camera']
        actions.append(include('realsense2_camera', 'rs_launch.py', {
            'serial_no': '_' + str(camera['serial_no']), 'enable_sync': 'true',
            'align_depth.enable': 'true', 'enable_color': 'true', 'enable_depth': 'true',
            'rgb_camera.profile': camera['profile'], 'depth_module.profile': camera['profile'],
            'pointcloud.enable': str(plan['mapping']).lower()}))
        if plan['mapping']:
            actions.append(Node(package='camera', executable='calibration_tf_node', output='screen', parameters=[{
                'calibration_file': camera['calibration_file'],
                'camera_root_frame': perception['camera_root_frame'],
                'calibration_verified': profile['task']['calibration_verified']}]))
            actions.append(Node(package='camera', executable='pointcloud_filter_node', output='screen', parameters=[{
                'input_topic': perception['point_cloud_topic'],
                'output_topic': perception['scene_cloud_topic'],
                'max_update_rate': perception['max_update_rate']}]))
        else:
            actions.append(Node(package='camera', executable='camera_node', output='screen', parameters=[{
                'inference_python': camera['inference_python'], 'calibration_file': camera['calibration_file'],
                'enable_cv_window': False}]))
    if plan['gripper']:
        parameters = dict(profile['gripper'], allow_gripper_commands=plan['allow_gripper_commands'])
        actions.append(Node(package='gripper', executable='gripper_server', output='screen', parameters=[parameters]))
    if plan['routine']:
        parameters = dict(profile['task'], motion_enabled=True,
                          execution_timeout=resolved.execution_timeout)
        parameters.update(resolved.tabletop)
        actions.append(Node(package='demo_package', executable='vertical_fruit_gripping_demo',
                            output='screen', parameters=[parameters]))
    return actions

def generate_launch_description():
    default = os.path.join(get_package_share_directory('harvesting_bringup'), 'config', 'site.json')
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value=default),
        DeclareLaunchArgument('mode', default_value='fake', choices=list(deployment.MODES)),
        DeclareLaunchArgument('motion_enabled', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('execution_timeout', default_value=''),
        OpaqueFunction(function=compose)])
