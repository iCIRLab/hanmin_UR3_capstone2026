import numpy as np
import pytest

from capstone_planning.model import HOME_Q, JOINT_ORDER, SCAN_Q
from capstone_planning.mplib_planner import Ur3MplibPlanner


@pytest.fixture(scope='module')
def planner():
    instance = Ur3MplibPlanner()
    yield instance
    instance.close()


def test_mplib_planner_uses_all_canonical_joints(planner):
    assert tuple(planner.planner.user_joint_names) == JOINT_ORDER
    assert planner.planner.move_group_joint_indices == list(range(6))


def test_fk_ik_round_trip_for_home_pose(planner):
    home_pose = planner.forward_kinematics(HOME_Q)
    status, result = planner.inverse_kinematics(home_pose, HOME_Q)

    assert status == 'Success'
    assert result is not None
    solved_pose = planner.forward_kinematics(result)
    assert solved_pose.position == pytest.approx(home_pose.position, abs=1e-4)
    assert abs(np.dot(
        solved_pose.quaternion_wxyz,
        home_pose.quaternion_wxyz,
    )) == pytest.approx(1.0, abs=1e-4)


@pytest.mark.parametrize(
    'start, goal',
    [
        (HOME_Q, SCAN_Q),
        (SCAN_Q, HOME_Q),
    ],
)
def test_mplib_plans_time_parameterized_verified_poses(planner, start, goal):
    result = planner.plan_joint_goal(start, goal)

    assert result.success, result.message
    assert result.point_count > 2
    assert result.time[0] == pytest.approx(0.0)
    assert np.all(np.diff(result.time) > 0.0)
    assert result.position[0] == pytest.approx(start)
    assert result.position[-1] == pytest.approx(goal)
    assert np.max(np.abs(result.velocity)) <= 0.500001
    assert np.max(np.abs(result.acceleration)) <= 0.500001


def test_mplib_plans_to_scan_tcp_pose(planner):
    scan_pose = planner.forward_kinematics(SCAN_Q)
    result = planner.plan_pose_goal(HOME_Q, scan_pose)

    assert result.success, result.message
    assert result.point_count > 2
    final_pose = planner.forward_kinematics(result.position[-1])
    assert final_pose.position == pytest.approx(scan_pose.position, abs=1e-3)
    assert abs(np.dot(
        final_pose.quaternion_wxyz,
        scan_pose.quaternion_wxyz,
    )) == pytest.approx(1.0, abs=1e-3)


def test_no_op_plan_is_explicit_and_does_not_call_empty_mplib_path(planner):
    result = planner.plan_joint_goal(HOME_Q, HOME_Q)

    assert result.success
    assert result.message == 'No movement required'
    assert result.point_count == 1
    assert result.position[0] == pytest.approx(HOME_Q)


def test_joint_goal_outside_common_limits_is_rejected(planner):
    invalid_goal = list(HOME_Q)
    invalid_goal[2] = 4.0

    with pytest.raises(ValueError, match='outside'):
        planner.plan_joint_goal(HOME_Q, invalid_goal)
