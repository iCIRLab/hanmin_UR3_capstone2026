"""MoveIt request and trajectory adapters for the common UR3 contract."""

from collections.abc import Sequence
import copy
import math

from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory

from capstone_planning.model import (
    BASE_LINK,
    JOINT_ORDER,
    PLANNING_GROUP,
    TCP_LINK,
    validate_joint_vector,
)


def joint_goal_constraints(
    positions: Sequence[float],
    tolerance: float = 1e-4,
) -> Constraints:
    """Build exact six-joint goal constraints in canonical order."""
    values = validate_joint_vector(positions, 'joint goal')
    if not math.isfinite(tolerance) or tolerance <= 0.0:
        raise ValueError('Joint goal tolerance must be finite and positive')

    constraints = Constraints()
    constraints.name = 'ur3_joint_goal'
    for name, position in zip(JOINT_ORDER, values):
        constraint = JointConstraint()
        constraint.joint_name = name
        constraint.position = position
        constraint.tolerance_above = float(tolerance)
        constraint.tolerance_below = float(tolerance)
        constraint.weight = 1.0
        constraints.joint_constraints.append(constraint)
    return constraints


def pose_goal_constraints(
    values_xyzw: Sequence[float],
    position_tolerance: float = 0.005,
    orientation_tolerance: float = 0.01,
) -> Constraints:
    """Build a TCP pose constraint in base_link using XYZW quaternion order."""
    values = tuple(float(value) for value in values_xyzw)
    if len(values) != 7 or not all(math.isfinite(value) for value in values):
        raise ValueError(
            'Pose must contain finite [x, y, z, qx, qy, qz, qw] values'
        )
    if position_tolerance <= 0.0 or orientation_tolerance <= 0.0:
        raise ValueError('Pose tolerances must be positive')
    quaternion_norm = math.sqrt(sum(value * value for value in values[3:]))
    if quaternion_norm < 1e-9:
        raise ValueError('Pose quaternion has zero norm')

    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = values[:3]
    pose.orientation.x = values[3] / quaternion_norm
    pose.orientation.y = values[4] / quaternion_norm
    pose.orientation.z = values[5] / quaternion_norm
    pose.orientation.w = values[6] / quaternion_norm

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.SPHERE
    primitive.dimensions = [float(position_tolerance)]

    position = PositionConstraint()
    position.header.frame_id = BASE_LINK
    position.link_name = TCP_LINK
    position.constraint_region.primitives = [primitive]
    position.constraint_region.primitive_poses = [pose]
    position.weight = 1.0

    orientation = OrientationConstraint()
    orientation.header.frame_id = BASE_LINK
    orientation.link_name = TCP_LINK
    orientation.orientation = pose.orientation
    orientation.absolute_x_axis_tolerance = float(orientation_tolerance)
    orientation.absolute_y_axis_tolerance = float(orientation_tolerance)
    orientation.absolute_z_axis_tolerance = float(orientation_tolerance)
    orientation.parameterization = OrientationConstraint.ROTATION_VECTOR
    orientation.weight = 1.0

    constraints = Constraints()
    constraints.name = 'ur3_tcp_pose_goal'
    constraints.position_constraints = [position]
    constraints.orientation_constraints = [orientation]
    return constraints


def build_motion_plan_request(
    current_positions: Sequence[float],
    constraints: Constraints,
    *,
    planning_time: float = 2.0,
    velocity_scaling: float = 1.0,
    acceleration_scaling: float = 1.0,
) -> MotionPlanRequest:
    """Build a plan-service request from an explicit UR3 start state."""
    current = validate_joint_vector(current_positions, 'current positions')
    for name, value in (
        ('planning_time', planning_time),
        ('velocity_scaling', velocity_scaling),
        ('acceleration_scaling', acceleration_scaling),
    ):
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
    if velocity_scaling > 1.0 or acceleration_scaling > 1.0:
        raise ValueError('MoveIt scaling factors cannot exceed 1.0')

    request = MotionPlanRequest()
    request.group_name = PLANNING_GROUP
    request.pipeline_id = 'ompl'
    request.planner_id = 'RRTConnectkConfigDefault'
    request.num_planning_attempts = 1
    request.allowed_planning_time = float(planning_time)
    request.max_velocity_scaling_factor = float(velocity_scaling)
    request.max_acceleration_scaling_factor = float(acceleration_scaling)
    request.start_state.joint_state.name = list(JOINT_ORDER)
    request.start_state.joint_state.position = list(current)
    request.start_state.is_diff = False
    request.goal_constraints = [constraints]

    return request


def canonicalize_joint_trajectory(
    trajectory: JointTrajectory,
) -> JointTrajectory:
    """Return a deep-copied trajectory reordered to the common joint contract."""
    names = tuple(trajectory.joint_names)
    if len(names) != len(set(names)) or set(names) != set(JOINT_ORDER):
        raise ValueError(
            f'MoveIt trajectory joints do not match UR3: {names}'
        )
    source_indices = [names.index(name) for name in JOINT_ORDER]
    canonical = copy.deepcopy(trajectory)
    canonical.joint_names = list(JOINT_ORDER)

    for point in canonical.points:
        for field in ('positions', 'velocities', 'accelerations', 'effort'):
            source = list(getattr(point, field))
            if not source:
                continue
            if len(source) != len(JOINT_ORDER):
                raise ValueError(
                    f'MoveIt point {field} has {len(source)} values'
                )
            setattr(
                point,
                field,
                [source[index] for index in source_indices],
            )
    return canonical
