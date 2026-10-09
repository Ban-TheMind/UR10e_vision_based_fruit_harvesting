"""Local mock control only; no UR network-facing processes."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', LaunchConfiguration('description_file'),
        ' ur_type:=ur10e name:=ur use_fake_hardware:=true robot_ip:=192.0.2.1 kinematics_params:=',
        LaunchConfiguration('kinematics_params_file'),
    ]), value_type=str)
    params = {'robot_description': description}
    controllers = str(Path(get_package_share_directory('harvesting_bringup')) / 'config/fake_controllers.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('description_file'),
        DeclareLaunchArgument('kinematics_params_file'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[params], output='screen'),
        Node(package='controller_manager', executable='ros2_control_node', parameters=[params, controllers], output='screen'),
        Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'], output='screen'),
        Node(package='controller_manager', executable='spawner', arguments=['joint_trajectory_controller', '--controller-manager-timeout', '60'], output='screen'),
    ])
