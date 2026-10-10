"""Compose individual field stages without starting a routine by default."""
import importlib.util
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

spec = importlib.util.spec_from_file_location('harvesting_deployment', os.path.join(os.path.dirname(__file__), 'deployment.py'))
deployment = importlib.util.module_from_spec(spec)
spec.loader.exec_module(deployment)

def include(package, filename, arguments):
    path = os.path.join(get_package_share_directory(package), 'launch', filename)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=arguments.items())

def compose(context):
    profile = deployment.load_profile(LaunchConfiguration('profile').perform(context))
    mode = LaunchConfiguration('mode').perform(context)
    plan = deployment.build_plan(profile, mode, LaunchConfiguration('motion_enabled').perform(context))
    rviz = deployment.boolean(LaunchConfiguration('rviz').perform(context))
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
        actions.append(TimerAction(period=4.0, actions=[
            include('end_effector_description', 'foxy_moveit.launch.py', moveit)]))
    if plan['camera']:
        camera = profile['camera']
        actions.append(include('realsense2_camera', 'rs_launch.py', {
            'serial_no': '_' + str(camera['serial_no']), 'enable_sync': 'true',
            'align_depth.enable': 'true', 'enable_color': 'true', 'enable_depth': 'true',
            'rgb_camera.profile': camera['profile'], 'depth_module.profile': camera['profile'],
            'pointcloud.enable': 'false'}))
        actions.append(Node(package='camera', executable='camera_node', output='screen', parameters=[{
            'inference_python': camera['inference_python'], 'calibration_file': camera['calibration_file'],
            'enable_cv_window': False}]))
    if plan['gripper']:
        parameters = dict(profile['gripper'], allow_gripper_commands=plan['allow_gripper_commands'])
        actions.append(Node(package='gripper', executable='gripper_server', output='screen', parameters=[parameters]))
    if plan['routine']:
        parameters = dict(profile['task'], motion_enabled=True)
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
        OpaqueFunction(function=compose)])
