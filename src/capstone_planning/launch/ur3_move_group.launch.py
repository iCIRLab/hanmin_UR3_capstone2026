#!/usr/bin/env python3
"""Launch MoveIt move_group in planning-only mode for the calibrated UR3."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from capstone_planning.model import bounded_urdf_text


def _load_yaml(path: Path):
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def _launch_setup(context):
    share = Path(get_package_share_directory('capstone_planning'))
    description = bounded_urdf_text(
        share / 'description' / 'ur3_calibrated.urdf'
    )
    semantic = (
        share / 'description' / 'ur3.srdf'
    ).read_text(encoding='utf-8')
    limits = _load_yaml(share / 'config' / 'ur3_planning_limits.yaml')
    kinematics = _load_yaml(
        share / 'config' / 'moveit_kinematics.yaml'
    )
    ompl = _load_yaml(share / 'config' / 'moveit_ompl.yaml')
    use_sim_time = (
        LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    )

    ompl_pipeline = {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': (
            'default_planner_request_adapters/AddTimeOptimalParameterization '
            'default_planner_request_adapters/FixWorkspaceBounds '
            'default_planner_request_adapters/FixStartStateBounds '
            'default_planner_request_adapters/FixStartStateCollision '
            'default_planner_request_adapters/FixStartStatePathConstraints'
        ),
        'start_state_max_bounds_error': 0.1,
    }
    ompl_pipeline.update(ompl)

    parameters = [
        {'robot_description': description},
        {'robot_description_semantic': semantic},
        {'robot_description_kinematics': kinematics},
        {'robot_description_planning': limits},
        {'ompl': ompl_pipeline},
        {'planning_pipelines': ['ompl']},
        {'default_planning_pipeline': 'ompl'},
        # MoveIt only plans in this workspace.  The shared trajectory_executor
        # is the sole owner of FollowJointTrajectory execution for both
        # MPlib and MoveIt.
        {'allow_trajectory_execution': False},
        {
            'disable_capabilities': (
                'move_group/MoveGroupCartesianPathService '
                'move_group/MoveGroupKinematicsService '
                'move_group/MoveGroupExecuteTrajectoryAction '
                'move_group/MoveGroupMoveAction '
                'move_group/MoveGroupQueryPlannersService '
                'move_group/MoveGroupStateValidationService '
                'move_group/MoveGroupGetPlanningSceneService '
                'move_group/ClearOctomapService'
            ),
        },
        {
            'moveit_controller_manager': (
                'capstone_moveit_support/'
                'PlanningOnlyControllerManager'
            ),
        },
        {
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
            'publish_robot_description_semantic': True,
            'use_sim_time': use_sim_time,
        },
    ]

    return [
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=parameters,
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_launch_setup),
    ])
