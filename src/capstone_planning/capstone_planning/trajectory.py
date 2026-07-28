"""Safe conversion from planner output to ROS JointTrajectory messages."""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Sequence

import numpy as np
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from capstone_planning.model import (
    JOINT_ORDER,
    JointLimit,
    load_joint_limits,
    validate_joint_vector,
)
from capstone_planning.plan_result import PlanResult


class TrajectoryValidationError(ValueError):
    """Raised when a trajectory is unsafe or structurally invalid."""


@dataclass(frozen=True)
class TrajectorySummary:
    """Validated trajectory metrics."""

    point_count: int
    duration: float
    path_length: float
    maximum_velocity: float
    maximum_acceleration: float
    no_op: bool = False


def validate_plan_result(
    result: PlanResult,
    current_qpos: Sequence[float],
    limits_path: Path | None = None,
    *,
    maximum_start_error: float = 0.05,
    final_velocity_tolerance: float = 1e-3,
    numeric_tolerance: float = 1e-6,
) -> TrajectorySummary:
    """Validate a time-parameterized plan before ROS message conversion."""
    if not result.success:
        raise TrajectoryValidationError(
            result.message or f'Planner failed with status {result.status}'
        )

    current = np.asarray(
        validate_joint_vector(current_qpos, 'current_qpos'),
        dtype=float,
    )
    joint_count = len(JOINT_ORDER)
    arrays = {
        'position': np.asarray(result.position, dtype=float),
        'velocity': np.asarray(result.velocity, dtype=float),
        'acceleration': np.asarray(result.acceleration, dtype=float),
    }
    for label, array in arrays.items():
        if array.ndim != 2 or array.shape[1] != joint_count:
            raise TrajectoryValidationError(
                f'{label} must have shape (N, {joint_count}), got {array.shape}'
            )
        if not np.all(np.isfinite(array)):
            raise TrajectoryValidationError(f'{label} contains NaN or infinity')

    point_count = arrays['position'].shape[0]
    if not (
        arrays['velocity'].shape[0]
        == arrays['acceleration'].shape[0]
        == point_count
    ):
        raise TrajectoryValidationError(
            'Trajectory arrays have inconsistent waypoint counts'
        )
    if point_count == 0:
        raise TrajectoryValidationError('Trajectory contains no waypoints')

    times = np.asarray(result.time, dtype=float).reshape(-1)
    if times.size != point_count or not np.all(np.isfinite(times)):
        raise TrajectoryValidationError(
            'Trajectory time array is invalid or has the wrong length'
        )
    if times[0] < -numeric_tolerance:
        raise TrajectoryValidationError('First trajectory time is negative')
    if point_count > 1 and not np.all(np.diff(times) > 0.0):
        raise TrajectoryValidationError(
            'Trajectory times must be strictly increasing'
        )

    start_error = float(np.max(np.abs(arrays['position'][0] - current)))
    if start_error > maximum_start_error:
        raise TrajectoryValidationError(
            f'Trajectory start differs from current state by '
            f'{start_error:.6f} rad'
        )

    limits = load_joint_limits(limits_path)
    for index, name in enumerate(JOINT_ORDER):
        limit: JointLimit = limits[name]
        positions = arrays['position'][:, index]
        if np.any(positions < limit.lower - numeric_tolerance) or np.any(
            positions > limit.upper + numeric_tolerance
        ):
            raise TrajectoryValidationError(
                f'{name} exceeds position limits'
            )
        if np.max(np.abs(arrays['velocity'][:, index])) > (
            limit.velocity + numeric_tolerance
        ):
            raise TrajectoryValidationError(
                f'{name} exceeds velocity limit'
            )
        if np.max(np.abs(arrays['acceleration'][:, index])) > (
            limit.acceleration + numeric_tolerance
        ):
            raise TrajectoryValidationError(
                f'{name} exceeds acceleration limit'
            )

    final_velocity = float(np.max(np.abs(arrays['velocity'][-1])))
    if final_velocity > final_velocity_tolerance:
        raise TrajectoryValidationError(
            f'Final velocity {final_velocity:.6f} rad/s exceeds tolerance'
        )

    duration = float(times[-1])
    if abs(float(result.duration) - duration) > max(
        numeric_tolerance,
        0.01 * max(duration, 1.0),
    ):
        raise TrajectoryValidationError(
            f'Result duration {result.duration:.6f}s does not match final '
            f'time {duration:.6f}s'
        )

    deltas = np.diff(arrays['position'], axis=0)
    path_length = (
        float(np.sum(np.linalg.norm(deltas, axis=1)))
        if deltas.size
        else 0.0
    )
    return TrajectorySummary(
        point_count=point_count,
        duration=duration,
        path_length=path_length,
        maximum_velocity=float(np.max(np.abs(arrays['velocity']))),
        maximum_acceleration=float(np.max(np.abs(arrays['acceleration']))),
        no_op=point_count == 1 and path_length <= numeric_tolerance,
    )


def plan_result_to_joint_trajectory(
    result: PlanResult,
    current_qpos: Sequence[float],
    limits_path: Path | None = None,
    **validation_options,
) -> tuple[JointTrajectory, TrajectorySummary]:
    """Validate and convert one PlanResult to a ROS JointTrajectory."""
    summary = validate_plan_result(
        result,
        current_qpos,
        limits_path,
        **validation_options,
    )
    message = JointTrajectory()
    message.joint_names = list(JOINT_ORDER)

    for index, timestamp in enumerate(result.time):
        point = JointTrajectoryPoint()
        point.positions = [
            float(value) for value in result.position[index]
        ]
        point.velocities = [
            float(value) for value in result.velocity[index]
        ]
        point.accelerations = [
            float(value) for value in result.acceleration[index]
        ]
        point.time_from_start = seconds_to_duration(float(timestamp))
        message.points.append(point)
    return message, summary


def seconds_to_duration(seconds: float) -> Duration:
    """Convert finite, non-negative seconds to a normalized ROS Duration."""
    value = float(seconds)
    if not math.isfinite(value) or value < 0.0:
        raise TrajectoryValidationError(
            f'Invalid trajectory timestamp: {seconds}'
        )
    whole_seconds = int(math.floor(value))
    nanoseconds = int(round((value - whole_seconds) * 1_000_000_000))
    if nanoseconds == 1_000_000_000:
        whole_seconds += 1
        nanoseconds = 0
    return Duration(sec=whole_seconds, nanosec=nanoseconds)


def validate_joint_trajectory_message(
    trajectory: JointTrajectory,
    limits_path: Path | None = None,
) -> None:
    """Perform structural validation at the executor trust boundary."""
    if tuple(trajectory.joint_names) != JOINT_ORDER:
        raise TrajectoryValidationError(
            'JointTrajectory names or order do not match the UR3 contract'
        )
    if len(trajectory.points) < 2:
        raise TrajectoryValidationError(
            'Executable JointTrajectory requires at least two points'
        )

    limits = load_joint_limits(limits_path)
    previous_time = -1.0
    for index, point in enumerate(trajectory.points):
        for label, values in (
            ('positions', point.positions),
            ('velocities', point.velocities),
            ('accelerations', point.accelerations),
        ):
            if len(values) != len(JOINT_ORDER):
                raise TrajectoryValidationError(
                    f'Point {index} {label} has {len(values)} values'
                )
            if not all(math.isfinite(float(value)) for value in values):
                raise TrajectoryValidationError(
                    f'Point {index} {label} contains NaN or infinity'
                )
        timestamp = (
            float(point.time_from_start.sec)
            + float(point.time_from_start.nanosec) / 1_000_000_000.0
        )
        if timestamp <= previous_time:
            raise TrajectoryValidationError(
                'JointTrajectory timestamps are not strictly increasing'
            )
        previous_time = timestamp

        for joint_index, name in enumerate(JOINT_ORDER):
            limit = limits[name]
            position = float(point.positions[joint_index])
            velocity = abs(float(point.velocities[joint_index]))
            acceleration = abs(float(point.accelerations[joint_index]))
            if position < limit.lower or position > limit.upper:
                raise TrajectoryValidationError(
                    f'Point {index} {name} exceeds position limits'
                )
            if velocity > limit.velocity + 1e-6:
                raise TrajectoryValidationError(
                    f'Point {index} {name} exceeds velocity limit'
                )
            if acceleration > limit.acceleration + 1e-6:
                raise TrajectoryValidationError(
                    f'Point {index} {name} exceeds acceleration limit'
                )
