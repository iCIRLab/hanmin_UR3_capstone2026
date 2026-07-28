from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_launch = PathJoinSubstitution([
        FindPackageShare('capstone_bringup'),
        'launch',
        'realsense_d435i.launch.py',
    ])
    viewer_config = PathJoinSubstitution([
        FindPackageShare('capstone_perception'),
        'config',
        'compressed_camera_viewer.yaml',
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
        ),
        Node(
            package='capstone_perception',
            executable='compressed_camera_viewer',
            name='compressed_camera_viewer',
            parameters=[viewer_config],
            output='screen',
        ),
    ])
