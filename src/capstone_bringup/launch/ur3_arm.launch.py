#!/usr/bin/env python3
"""Select MPlib or MoveIt and MuJoCo or real UR3 from one safe launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from capstone_bringup.launch_config import (
    parse_launch_bool,
    validate_arm_launch_options,
)


def _include(package: str, filename: str, arguments):
    share = get_package_share_directory(package)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', filename)
        ),
        launch_arguments=arguments.items(),
    )


def _launch_setup(context):
    backend = LaunchConfiguration('backend').perform(context)
    planner = LaunchConfiguration('planner').perform(context)
    plan_only = parse_launch_bool(
        LaunchConfiguration('plan_only').perform(context),
        'plan_only',
    )
    execute = parse_launch_bool(
        LaunchConfiguration('execute').perform(context),
        'execute',
    )
    confirm_real = parse_launch_bool(
        LaunchConfiguration('confirm_real_hardware').perform(context),
        'confirm_real_hardware',
    )
    backend, planner = validate_arm_launch_options(
        backend,
        planner,
        plan_only=plan_only,
        execute=execute,
        confirm_real_hardware=confirm_real,
    )

    use_sim_time = backend == 'mujoco'
    actions = []
    if backend == 'mujoco':
        actions.append(
            _include(
                'capstone_mujoco',
                'ur3_mujoco.launch.py',
                {
                    'headless': LaunchConfiguration('headless'),
                    'initial_pose': LaunchConfiguration('initial_pose'),
                    'show_ui': LaunchConfiguration('show_ui'),
                    'show_right_ui': LaunchConfiguration('show_right_ui'),
                    'show_profiler': LaunchConfiguration('show_profiler'),
                    'show_sensor': LaunchConfiguration('show_sensor'),
                    'window_width': LaunchConfiguration('window_width'),
                    'window_height': LaunchConfiguration('window_height'),
                    'render_fps': LaunchConfiguration('render_fps'),
                    'render_vsync': LaunchConfiguration('render_vsync'),
                    'ui_font_scale': LaunchConfiguration('ui_font_scale'),
                    'render_device': LaunchConfiguration('render_device'),
                },
            )
        )
        controller_action = (
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
    else:
        actions.append(
            _include(
                'capstone_bringup',
                'ur3_driver.launch.py',
                {
                    'robot_ip': LaunchConfiguration('robot_ip'),
                    'reverse_ip': LaunchConfiguration('reverse_ip'),
                    'kinematics_params_file': LaunchConfiguration(
                        'kinematics_params_file'
                    ),
                    'launch_rviz': LaunchConfiguration('launch_rviz'),
                    'headless_mode': LaunchConfiguration('headless'),
                },
            )
        )
        controller_action = (
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

    if planner in {'moveit', 'interactive'}:
        actions.append(
            _include(
                'capstone_planning',
                'ur3_move_group.launch.py',
                {'use_sim_time': str(use_sim_time).lower()},
            )
        )

    actions.append(
        Node(
            package='capstone_manipulation',
            executable='trajectory_executor',
            output='screen',
            parameters=[{
                'enabled': execute,
                'controller_action': controller_action,
                'use_sim_time': use_sim_time,
            }],
        )
    )

    if planner != 'none':
        if planner == 'interactive':
            executable = 'interactive_plan'
        else:
            executable = 'mplib_plan' if planner == 'mplib' else 'moveit_plan'
        actions.append(
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package='capstone_planning',
                        executable=executable,
                        output='screen',
                        parameters=[{
                            'plan_only': plan_only,
                            'planning_time': LaunchConfiguration(
                                'planning_time'
                            ),
                            'use_sim_time': use_sim_time,
                            **(
                                {}
                                if planner == 'interactive'
                                else {
                                    'named_goal': LaunchConfiguration(
                                        'named_goal'
                                    )
                                }
                            ),
                        }],
                    )
                ],
            )
        )
    return actions


def generate_launch_description():
    calibration = os.path.expanduser('~/capstone_ws/ur3_calibration.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='mujoco'),
        DeclareLaunchArgument('planner', default_value='mplib'),
        DeclareLaunchArgument('named_goal', default_value='scan'),
        DeclareLaunchArgument('planning_time', default_value='5.0'),
        DeclareLaunchArgument('plan_only', default_value='true'),
        DeclareLaunchArgument('execute', default_value='false'),
        DeclareLaunchArgument(
            'confirm_real_hardware',
            default_value='false',
        ),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('initial_pose', default_value='home'),
        DeclareLaunchArgument('show_ui', default_value='true'),
        DeclareLaunchArgument('show_right_ui', default_value='false'),
        DeclareLaunchArgument('show_profiler', default_value='false'),
        DeclareLaunchArgument('show_sensor', default_value='false'),
        DeclareLaunchArgument('window_width', default_value='1100'),
        DeclareLaunchArgument('window_height', default_value='620'),
        DeclareLaunchArgument('render_fps', default_value='60'),
        DeclareLaunchArgument('render_vsync', default_value='false'),
        DeclareLaunchArgument('ui_font_scale', default_value='100'),
        DeclareLaunchArgument('render_device', default_value='default'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.56.1'),
        DeclareLaunchArgument('reverse_ip', default_value='192.168.56.2'),
        DeclareLaunchArgument(
            'kinematics_params_file',
            default_value=calibration,
        ),
        OpaqueFunction(function=_launch_setup),
    ])
