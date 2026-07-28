import math

import pytest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from capstone_planning.model import (
    BASE_LINK,
    HOME_Q,
    JOINT_ORDER,
    PLANNING_GROUP,
    SCAN_Q,
    TCP_LINK,
)
from capstone_planning.moveit_request import (
    build_motion_plan_request,
    canonicalize_joint_trajectory,
    joint_goal_constraints,
    pose_goal_constraints,
)


def test_joint_constraints_preserve_common_order():
    constraints = joint_goal_constraints(SCAN_Q)

    assert tuple(
        constraint.joint_name
        for constraint in constraints.joint_constraints
    ) == JOINT_ORDER
    assert tuple(
        constraint.position
        for constraint in constraints.joint_constraints
    ) == SCAN_Q


def test_pose_constraints_use_tcp_and_normalize_xyzw_quaternion():
    constraints = pose_goal_constraints(
        [0.2, 0.1, 0.3, 0.0, 0.0, 0.0, 2.0]
    )
    position = constraints.position_constraints[0]
    orientation = constraints.orientation_constraints[0]

    assert position.header.frame_id == BASE_LINK
    assert position.link_name == TCP_LINK
    assert orientation.header.frame_id == BASE_LINK
    assert orientation.link_name == TCP_LINK
    assert orientation.orientation.w == 1.0


def test_motion_plan_request_has_explicit_start_state():
    request = build_motion_plan_request(
        HOME_Q,
        joint_goal_constraints(SCAN_Q),
    )

    assert request.group_name == PLANNING_GROUP
    assert request.pipeline_id == 'ompl'
    assert tuple(request.start_state.joint_state.name) == JOINT_ORDER
    assert tuple(request.start_state.joint_state.position) == HOME_Q


@pytest.mark.parametrize('scaling', [0.0, -0.1, 1.1, math.inf])
def test_invalid_scaling_is_rejected(scaling):
    with pytest.raises(ValueError):
        build_motion_plan_request(
            HOME_Q,
            joint_goal_constraints(SCAN_Q),
            velocity_scaling=scaling,
        )


def test_moveit_trajectory_is_reordered_to_common_contract():
    trajectory = JointTrajectory()
    trajectory.joint_names = list(reversed(JOINT_ORDER))
    point = JointTrajectoryPoint()
    point.positions = [float(value) for value in range(6)]
    point.velocities = [float(value) for value in range(10, 16)]
    point.accelerations = [float(value) for value in range(20, 26)]
    trajectory.points = [point]

    canonical = canonicalize_joint_trajectory(trajectory)

    assert tuple(canonical.joint_names) == JOINT_ORDER
    assert tuple(canonical.points[0].positions) == tuple(reversed(range(6)))
    assert tuple(canonical.points[0].velocities) == tuple(
        reversed(range(10, 16))
    )
    assert tuple(canonical.points[0].accelerations) == tuple(
        reversed(range(20, 26))
    )
    assert tuple(trajectory.joint_names) == tuple(reversed(JOINT_ORDER))
