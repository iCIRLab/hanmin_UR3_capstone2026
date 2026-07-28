#!/usr/bin/env python3
"""Launch the calibrated UR3 with the released MuJoCo ros2_control backend."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

from capstone_mujoco.robot_description import build_robot_description
from capstone_mujoco.runtime_environment import ensure_mujoco_runtime


def _as_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized not in {'true', 'false'}:
        raise ValueError(f'Expected true or false, got {value!r}')
    return normalized == 'true'


def _launch_setup(context):
    runtime_prefix = ensure_mujoco_runtime()
    planning_share = get_package_share_directory('capstone_planning')
    mujoco_share = get_package_share_directory('capstone_mujoco')
    source_urdf = os.path.join(
        planning_share,
        'description',
        'ur3_calibrated.urdf',
    )
    scene = os.path.join(mujoco_share, 'description', 'ur3_scene.xml')
    controllers = os.path.join(mujoco_share, 'config', 'controllers.yaml')
    headless = _as_bool(LaunchConfiguration('headless').perform(context))
    show_ui = _as_bool(
        LaunchConfiguration('show_ui').perform(context)
    )
    show_right_ui = _as_bool(
        LaunchConfiguration('show_right_ui').perform(context)
    )
    show_profiler = _as_bool(
        LaunchConfiguration('show_profiler').perform(context)
    )
    show_sensor = _as_bool(
        LaunchConfiguration('show_sensor').perform(context)
    )
    window_width = LaunchConfiguration('window_width').perform(context)
    window_height = LaunchConfiguration('window_height').perform(context)
    render_fps = float(LaunchConfiguration('render_fps').perform(context))
    render_vsync = _as_bool(
        LaunchConfiguration('render_vsync').perform(context)
    )
    ui_font_scale = int(
        LaunchConfiguration('ui_font_scale').perform(context)
    )
    render_device = LaunchConfiguration('render_device').perform(
        context
    ).strip().lower()
    if render_device not in {'default', 'nvidia'}:
        raise ValueError(
            "render_device must be either 'default' or 'nvidia'"
        )
    render_environment = {}
    if render_device == 'nvidia':
        render_environment = {
            '__NV_PRIME_RENDER_OFFLOAD': '1',
            '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
        }
    initial_pose = LaunchConfiguration('initial_pose').perform(context)
    if str(runtime_prefix).startswith(str(os.path.expanduser('~/.local/'))):
        print(f'Using user-local MuJoCo ROS runtime: {runtime_prefix}')

    robot_description = {
        'robot_description': build_robot_description(
            source_urdf,
            scene,
            headless=headless,
            initial_pose=initial_pose,
            render_fps=render_fps,
            render_vsync=render_vsync,
            window_width=int(window_width),
            window_height=int(window_height),
            ui_font_scale=ui_font_scale,
        )
    }

    actions = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[robot_description, {'use_sim_time': True}],
        ),
        Node(
            package='mujoco_ros2_control',
            executable='ros2_control_node',
            emulate_tty=True,
            output='both',
            additional_env=render_environment,
            parameters=[
                {'use_sim_time': True},
                ParameterFile(controllers),
            ],
            remappings=[('~/robot_description', '/robot_description')],
            on_exit=Shutdown(),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager',
                '/controller_manager',
                '--param-file',
                controllers,
            ],
            output='both',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_trajectory_controller',
                '--controller-manager',
                '/controller_manager',
                '--param-file',
                controllers,
            ],
            output='both',
        ),
    ]
    if not headless:
        ui_arguments = [
            '--width',
            window_width,
            '--height',
            window_height,
        ]
        if show_ui:
            # ICIR used the control panel on the left and left the right panel
            # closed. The custom ROS 2 diagnostics are rendered directly by
            # the backend at bottom-left, so enabling MuJoCo's built-in F2
            # overlay here would draw two sets of text on top of each other.
            ui_arguments.append('--left')
            if show_right_ui:
                ui_arguments.extend(['--right', '--joint'])
            if show_profiler:
                ui_arguments.append('--profiler')
            if show_sensor:
                ui_arguments.append('--sensor')
        actions.append(
            Node(
                package='capstone_mujoco',
                executable='mujoco_ui_setup',
                arguments=ui_arguments,
                output='screen',
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description='Run MuJoCo without its visualization window.',
        ),
        DeclareLaunchArgument(
            'initial_pose',
            default_value='home',
            description='Initial MuJoCo keyframe: home or scan.',
        ),
        DeclareLaunchArgument(
            'show_ui',
            default_value='true',
            description='Show the ICIR-style MuJoCo left control panel.',
        ),
        DeclareLaunchArgument(
            'show_right_ui',
            default_value='false',
            description='Also show the right joint panel (closed by default).',
        ),
        DeclareLaunchArgument(
            'show_profiler',
            default_value='false',
            description='Show the MuJoCo solver profiler at startup.',
        ),
        DeclareLaunchArgument(
            'show_sensor',
            default_value='false',
            description='Show MuJoCo joint and tool sensor plots at startup.',
        ),
        DeclareLaunchArgument(
            'window_width',
            default_value='1100',
            description='MuJoCo viewer width in pixels.',
        ),
        DeclareLaunchArgument(
            'window_height',
            default_value='620',
            description='MuJoCo viewer height in pixels.',
        ),
        DeclareLaunchArgument(
            'render_fps',
            default_value='60',
            description='Viewer FPS cap independent of display refresh rate.',
        ),
        DeclareLaunchArgument(
            'render_vsync',
            default_value='false',
            description='Synchronize rendering to monitor VBlank.',
        ),
        DeclareLaunchArgument(
            'ui_font_scale',
            default_value='100',
            description='Native MuJoCo UI scale: 100 to 300 in 50% steps.',
        ),
        DeclareLaunchArgument(
            'render_device',
            default_value='default',
            description=(
                "OpenGL renderer: 'nvidia' enables PRIME offload; "
                "'default' uses the current X11 renderer."
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
