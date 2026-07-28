from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='sensors',
            name='d435i',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'camera_name': 'd435i',
                'camera_namespace': 'sensors',
                'enable_depth': True,
                'enable_color': True,
                'pointcloud.enable': True,
                'align_depth.enable': True,
                'enable_gyro': False,
                'enable_accel': False,
                'publish_tf': True,
            }],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
    ])
