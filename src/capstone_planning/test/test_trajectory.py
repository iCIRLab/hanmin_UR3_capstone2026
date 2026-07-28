import numpy as np
import pytest

from capstone_planning.model import HOME_Q, JOINT_ORDER, SCAN_Q
from capstone_planning.mplib_planner import Ur3MplibPlanner
from capstone_planning.plan_result import PlanResult
from capstone_planning.trajectory import (
    TrajectoryValidationError,
    plan_result_to_joint_trajectory,
    validate_joint_trajectory_message,
    validate_plan_result,
)


@pytest.fixture(scope='module')
def verified_plan():
    with Ur3MplibPlanner() as planner:
        return planner.plan_joint_goal(HOME_Q, SCAN_Q)


def test_verified_mplib_plan_converts_to_ros_trajectory(verified_plan):
    message, summary = plan_result_to_joint_trajectory(
        verified_plan,
        HOME_Q,
    )

    assert tuple(message.joint_names) == JOINT_ORDER
    assert len(message.points) == verified_plan.point_count
    assert summary.point_count > 2
    assert summary.duration == pytest.approx(verified_plan.duration)
    validate_joint_trajectory_message(message)


@pytest.mark.parametrize(
    'field, replacement, expected',
    [
        ('position', np.zeros((2, 5)), 'shape'),
        ('velocity', np.full((2, 6), 0.6), 'velocity limit'),
        ('acceleration', np.full((2, 6), 0.6), 'acceleration limit'),
        ('time', np.array([0.0, 0.0]), 'strictly increasing'),
    ],
)
def test_invalid_plan_fields_are_rejected(field, replacement, expected):
    values = {
        'status': 'Success',
        'time': np.array([0.0, 1.0]),
        'position': np.vstack([HOME_Q, HOME_Q]),
        'velocity': np.zeros((2, 6)),
        'acceleration': np.zeros((2, 6)),
        'duration': 1.0,
    }
    values[field] = replacement
    result = PlanResult(**values)

    with pytest.raises(TrajectoryValidationError, match=expected):
        validate_plan_result(result, HOME_Q)


def test_start_state_discontinuity_is_rejected():
    result = PlanResult(
        status='Success',
        time=np.array([0.0, 1.0]),
        position=np.vstack([SCAN_Q, SCAN_Q]),
        velocity=np.zeros((2, 6)),
        acceleration=np.zeros((2, 6)),
        duration=1.0,
    )

    with pytest.raises(TrajectoryValidationError, match='current state'):
        validate_plan_result(result, HOME_Q)


def test_executor_boundary_rechecks_numeric_limits(verified_plan):
    message, _ = plan_result_to_joint_trajectory(verified_plan, HOME_Q)
    message.points[1].velocities[0] = 0.6

    with pytest.raises(TrajectoryValidationError, match='velocity limit'):
        validate_joint_trajectory_message(message)
