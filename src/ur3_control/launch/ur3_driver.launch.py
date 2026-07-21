import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_dir = get_package_share_directory('ur3_control')
    ur_driver_dir = get_package_share_directory('ur_robot_driver')

    robot_ip = LaunchConfiguration('robot_ip')
    reverse_ip = LaunchConfiguration('reverse_ip')
    headless_mode = LaunchConfiguration('headless_mode')
    launch_rviz = LaunchConfiguration('launch_rviz')
    kinematics_params_file = LaunchConfiguration('kinematics_params_file')

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.56.1'),
        DeclareLaunchArgument('reverse_ip', default_value='192.168.56.2'),
        DeclareLaunchArgument('headless_mode', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument(
            'kinematics_params_file',
            default_value=os.path.join(
                package_dir, 'config', 'ur3_calibration.yaml'),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ur_driver_dir, 'launch', 'ur_control.launch.py')
            ),
            launch_arguments={
                'ur_type': 'ur3',
                'robot_ip': robot_ip,
                'reverse_ip': reverse_ip,
                'headless_mode': headless_mode,
                'launch_rviz': launch_rviz,
                'kinematics_params_file': kinematics_params_file,
            }.items(),
        ),
    ])
