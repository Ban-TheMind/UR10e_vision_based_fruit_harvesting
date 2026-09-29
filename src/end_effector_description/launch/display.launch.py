import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# Toggle between simulated or real UR10e hardware
use_fake = False
use_fake_str = 'true'
ur_type = 'ur10e'
ip_address = 'yyy.yyy.yyy.yyy'

if not use_fake:
    print("not fake")
    ip_address = '192.168.11.60'
    use_fake_str = 'false'


def get_realsense_launch():
    """Setup Realsense camera launch with complete pointcloud configuration."""
    realsense_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py'
    )

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            # RealSense ROS requires '_' to keep a numeric serial as a string.
            'serial_no': '_406122071837',
            'enable_sync': 'true',
            'align_depth.enable': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'rgb_camera.profile': '640,480,15',
            'depth_module.profile': '640,480,15',
            'pointcloud.enable': 'false',
        }.items()
    )

def get_ur_control_launch():
    """Configure UR control launch for the UR10e arm."""
    ur_control_launch_args = {
        'ur_type': ur_type,
        'robot_ip': ip_address,
        'use_fake_hardware': use_fake_str,
        'launch_rviz': 'false',  
        'description_package': 'end_effector_description',
        'description_file': 'end_effector_withDriverSupport.xacro',
        'robot_controller': 'joint_trajectory_controller',
    }

    # Add controller if using simulated hardware
    if use_fake:
        ur_control_launch_args['fake_sensor_commands'] = 'false'

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('end_effector_description'), 'launch', 'foxy_ur_control.launch.py'])
        ),
        launch_arguments=ur_control_launch_args.items(),
    )

def get_moveit_launch():
    """Configure MoveIt launch with a delay to ensure UR control is initialized."""
    moveit_launch_args = {
        'ur_type': ur_type,
        'robot_ip': ip_address,
        'launch_rviz': 'true',
        'use_fake_hardware': use_fake_str,
        'description_package': 'end_effector_description',
        'description_file': 'end_effector_withDriverSupport.xacro',
    }

    return TimerAction(
        period=4.0,  # Delay to prevent conflicts in RViz
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('end_effector_description'), 'launch', 'foxy_moveit.launch.py'])
                ),
                launch_arguments=moveit_launch_args.items(),
            )
        ]
    )
# def get_moveit_launch():
#     moveit_launch_path = os.path.join(
#         get_package_share_directory('ur10e_moveit_config'), 'launch', 'move_group.launch.py'
#     )

#     return IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(moveit_launch_path)
#     )

def get_rviz_launch():
    moveit_launch_path = os.path.join(
        get_package_share_directory('ur10e_moveit_config'), 'launch', 'moveit_rviz.launch.py'
    )

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_path)
    )

def get_auxiliary_launch():
    moveit_launch_path = os.path.join(
        get_package_share_directory('end_effector_description'), 'launch', 'auxiliary.launch.py'
    )

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_path)
    )



def generate_launch_description():
    """Main function to generate the complete launch description."""
    launch_description = [
        get_ur_control_launch(),
        get_moveit_launch(),
        # get_rviz_launch(),
    ]

    # Only add camera launch if using real hardware
    if not use_fake:
        launch_description.append(get_realsense_launch())
        launch_description.append(get_auxiliary_launch())

    return LaunchDescription(launch_description)
