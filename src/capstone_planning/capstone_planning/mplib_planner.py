"""Calibrated UR3 planning wrapper for the installed MPlib 0.2.1 API."""

from dataclasses import dataclass
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import RLock
from typing import Sequence

import mplib
import numpy as np

from capstone_planning.model import (
    JOINT_ORDER,
    TCP_LINK,
    description_paths,
    load_joint_limits,
    ordered_values,
    prepare_mplib_urdf,
    validate_joint_vector,
)
from capstone_planning.plan_result import PlanResult


TIME_PARAMETERIZATION_LIMIT_SCALE = 0.95


@dataclass(frozen=True)
class CartesianPose:
    """Cartesian pose using MPlib's w, x, y, z quaternion convention."""

    position: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]

    def to_mplib(self) -> mplib.Pose:
        """Convert to an MPlib Pose."""
        return mplib.Pose(
            p=np.asarray(self.position, dtype=float),
            q=np.asarray(self.quaternion_wxyz, dtype=float),
        )


class Ur3MplibPlanner:
    """Own one MPlib planner and its generated resource-resolvable URDF."""

    def __init__(
        self,
        urdf_path: Path | None = None,
        srdf_path: Path | None = None,
        limits_path: Path | None = None,
        work_directory: Path | None = None,
        verbose: bool = False,
    ) -> None:
        default_urdf, default_srdf = description_paths()
        source_urdf = default_urdf if urdf_path is None else Path(urdf_path)
        semantic = default_srdf if srdf_path is None else Path(srdf_path)
        limits = load_joint_limits(limits_path)

        self._temporary_directory: TemporaryDirectory[str] | None = None
        if work_directory is None:
            self._temporary_directory = TemporaryDirectory(
                prefix='capstone_ur3_mplib_'
            )
            generated_directory = Path(self._temporary_directory.name)
        else:
            generated_directory = Path(work_directory)

        derived_urdf = prepare_mplib_urdf(
            source_urdf,
            generated_directory,
            limits=limits,
        )
        velocity_limits = np.asarray(
            ordered_values(limits, 'velocity'),
            dtype=float,
        ) * TIME_PARAMETERIZATION_LIMIT_SCALE
        acceleration_limits = np.asarray(
            ordered_values(limits, 'acceleration'),
            dtype=float,
        ) * TIME_PARAMETERIZATION_LIMIT_SCALE

        self._planner = mplib.Planner(
            derived_urdf,
            TCP_LINK,
            srdf=semantic,
            user_joint_names=JOINT_ORDER,
            joint_vel_limits=velocity_limits,
            joint_acc_limits=acceleration_limits,
            verbose=verbose,
        )
        self._limits = limits
        self._lock = RLock()

        if tuple(self._planner.user_joint_names) != JOINT_ORDER:
            raise RuntimeError(
                'MPlib active joints do not match the canonical UR3 order'
            )
        if list(self._planner.move_group_joint_indices) != list(
            range(len(JOINT_ORDER))
        ):
            raise RuntimeError('MPlib move group does not contain all UR3 joints')

    @property
    def planner(self) -> mplib.Planner:
        """Expose the underlying planner for collision-world configuration."""
        return self._planner

    def close(self) -> None:
        """Release the temporary derived model directory."""
        if self._temporary_directory is not None:
            self._temporary_directory.cleanup()
            self._temporary_directory = None

    def __enter__(self) -> 'Ur3MplibPlanner':
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def forward_kinematics(self, qpos: Sequence[float]) -> CartesianPose:
        """Compute the tool0 pose for one canonical joint state."""
        q = np.asarray(validate_joint_vector(qpos, 'qpos'), dtype=float)
        self._validate_within_limits(q, 'qpos')
        with self._lock:
            self._planner.robot.set_qpos(q, True)
            self._planner.pinocchio_model.compute_forward_kinematics(q)
            pose = self._planner.pinocchio_model.get_link_pose(
                self._planner.move_group_link_id
            )
        return CartesianPose(
            tuple(float(value) for value in pose.p),
            tuple(float(value) for value in pose.q),
        )

    def inverse_kinematics(
        self,
        goal_pose: CartesianPose,
        start_qpos: Sequence[float],
        threshold: float = 0.001,
    ) -> tuple[str, np.ndarray | None]:
        """Return the valid IK solution closest to the start configuration."""
        start = np.asarray(
            validate_joint_vector(start_qpos, 'start_qpos'),
            dtype=float,
        )
        self._validate_within_limits(start, 'start_qpos')
        with self._lock:
            try:
                status, result = self._planner.IK(
                    goal_pose.to_mplib(),
                    start,
                    threshold=float(threshold),
                    return_closest=True,
                )
            finally:
                self._planner.robot.set_qpos(start, True)
        if status != 'Success' or result is None:
            return status, None
        return status, np.asarray(result, dtype=float)

    def plan_joint_goal(
        self,
        current_qpos: Sequence[float],
        goal_qpos: Sequence[float],
        *,
        time_step: float = 0.05,
        rrt_range: float = 0.1,
        planning_time: float = 2.0,
    ) -> PlanResult:
        """Plan and time-parameterize a collision-checked joint goal."""
        current = np.asarray(
            validate_joint_vector(current_qpos, 'current_qpos'),
            dtype=float,
        )
        goal = np.asarray(
            validate_joint_vector(goal_qpos, 'goal_qpos'),
            dtype=float,
        )
        self._validate_within_limits(current, 'current_qpos')
        self._validate_within_limits(goal, 'goal_qpos')
        if np.max(np.abs(goal - current)) <= 1e-6:
            return self._no_op_result(current)

        with self._lock:
            collision_failure = self._start_collision_failure(current)
            if collision_failure is not None:
                return collision_failure
            try:
                return self._plan_qpos_with_validation(
                    current,
                    goal,
                    time_step=float(time_step),
                    rrt_range=float(rrt_range),
                    planning_time=float(planning_time),
                )
            except (RuntimeError, ValueError) as error:
                return PlanResult.failed(f'MPlib plan_qpos failed: {error}')
            finally:
                self._planner.robot.set_qpos(current, True)

    def plan_pose_goal(
        self,
        current_qpos: Sequence[float],
        goal_pose: CartesianPose,
        *,
        time_step: float = 0.05,
        rrt_range: float = 0.1,
        planning_time: float = 2.0,
    ) -> PlanResult:
        """Plan and time-parameterize a collision-checked tool0 pose goal."""
        current = np.asarray(
            validate_joint_vector(current_qpos, 'current_qpos'),
            dtype=float,
        )
        self._validate_within_limits(current, 'current_qpos')
        with self._lock:
            collision_failure = self._start_collision_failure(current)
            if collision_failure is not None:
                return collision_failure
            try:
                raw = self._planner.plan_pose(
                    goal_pose.to_mplib(),
                    current,
                    time_step=float(time_step),
                    rrt_range=float(rrt_range),
                    planning_time=float(planning_time),
                )
                result = PlanResult.from_mplib(raw, len(JOINT_ORDER))
                collision = self._trajectory_collision(result)
                if collision is not None:
                    return PlanResult.failed(
                        'MPlib pose trajectory failed final collision '
                        f'validation: {collision}'
                    )
                return result
            except (RuntimeError, ValueError) as error:
                return PlanResult.failed(f'MPlib plan_pose failed: {error}')
            finally:
                self._planner.robot.set_qpos(current, True)

    def _validate_within_limits(
        self,
        qpos: np.ndarray,
        label: str,
    ) -> None:
        for index, name in enumerate(JOINT_ORDER):
            limit = self._limits[name]
            value = float(qpos[index])
            if value < limit.lower or value > limit.upper:
                raise ValueError(
                    f'{label} {name}={value:.6f} is outside '
                    f'[{limit.lower:.6f}, {limit.upper:.6f}]'
                )

    def _start_collision_failure(
        self,
        current: np.ndarray,
    ) -> PlanResult | None:
        self_collisions = self._planner.check_for_self_collision(current)
        environment_collisions = self._planner.check_for_env_collision(current)
        collisions = [*self_collisions, *environment_collisions]
        if not collisions:
            return None
        details = '; '.join(str(collision) for collision in collisions)
        return PlanResult.failed(f'Start state is in collision: {details}')

    def _plan_qpos_with_validation(
        self,
        current: np.ndarray,
        goal: np.ndarray,
        *,
        time_step: float,
        rrt_range: float,
        planning_time: float,
        maximum_attempts: int = 3,
    ) -> PlanResult:
        """Reject spline samples that clip an obstacle and re-run RRT."""
        last_failure = 'no planning attempt was made'
        for _ in range(maximum_attempts):
            raw = self._planner.plan_qpos(
                [goal],
                current,
                time_step=time_step,
                rrt_range=rrt_range,
                planning_time=planning_time,
            )
            result = PlanResult.from_mplib(raw, len(JOINT_ORDER))
            if not result.success:
                last_failure = result.message
                continue
            collision = self._trajectory_collision(result)
            if collision is None:
                return result
            last_failure = (
                'time-parameterized trajectory clips a collision object: '
                f'{collision}'
            )
        return PlanResult.failed(
            f'MPlib failed after {maximum_attempts} validated attempts: '
            f'{last_failure}'
        )

    def _trajectory_collision(self, result: PlanResult) -> str | None:
        if not result.success:
            return None
        for index, position in enumerate(result.position):
            collisions = [
                *self._planner.check_for_self_collision(position),
                *self._planner.check_for_env_collision(position),
            ]
            if collisions:
                return f'point {index}: {collisions[0]}'
        return None

    @staticmethod
    def _no_op_result(current: np.ndarray) -> PlanResult:
        zeros = np.zeros((1, len(JOINT_ORDER)), dtype=float)
        return PlanResult(
            status='Success',
            time=np.asarray([0.0], dtype=float),
            position=np.asarray([current], dtype=float),
            velocity=zeros,
            acceleration=zeros.copy(),
            duration=0.0,
            message='No movement required',
        )
