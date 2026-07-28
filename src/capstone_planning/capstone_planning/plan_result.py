"""Planner-independent motion-plan result types."""

from dataclasses import dataclass
from typing import Mapping

import numpy as np


@dataclass(frozen=True)
class PlanResult:
    """Time-parameterized joint trajectory returned by a planner."""

    status: str
    time: np.ndarray
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    duration: float
    message: str = ''

    @property
    def success(self) -> bool:
        """Return whether the planner returned a usable trajectory."""
        return self.status == 'Success'

    @property
    def point_count(self) -> int:
        """Return the number of trajectory waypoints."""
        return int(self.position.shape[0]) if self.position.ndim == 2 else 0

    @classmethod
    def failed(cls, message: str) -> 'PlanResult':
        """Construct a failed result with consistently shaped empty arrays."""
        empty_vector = np.empty((0,), dtype=float)
        empty_trajectory = np.empty((0, 0), dtype=float)
        return cls(
            status='Failed',
            time=empty_vector,
            position=empty_trajectory,
            velocity=empty_trajectory.copy(),
            acceleration=empty_trajectory.copy(),
            duration=0.0,
            message=str(message),
        )

    @classmethod
    def from_mplib(
        cls,
        raw: Mapping[str, object],
        joint_count: int,
    ) -> 'PlanResult':
        """Normalize an MPlib 0.2.1 result dictionary."""
        status = str(raw.get('status', 'Missing status'))
        if status != 'Success':
            return cls.failed(status)

        time = np.asarray(raw.get('time', []), dtype=float).reshape(-1)
        position = _trajectory_array(raw.get('position'), joint_count, 'position')
        velocity = _trajectory_array(raw.get('velocity'), joint_count, 'velocity')
        acceleration = _trajectory_array(
            raw.get('acceleration'),
            joint_count,
            'acceleration',
        )
        duration = float(raw.get('duration', time[-1] if time.size else 0.0))

        point_count = position.shape[0]
        if not (
            time.size
            == point_count
            == velocity.shape[0]
            == acceleration.shape[0]
        ):
            raise ValueError(
                'MPlib trajectory fields have inconsistent waypoint counts'
            )

        return cls(
            status='Success',
            time=time,
            position=position,
            velocity=velocity,
            acceleration=acceleration,
            duration=duration,
        )


def _trajectory_array(
    value: object,
    joint_count: int,
    label: str,
) -> np.ndarray:
    array = np.asarray([] if value is None else value, dtype=float)
    if array.size == 0:
        return np.empty((0, joint_count), dtype=float)
    if array.ndim != 2 or array.shape[1] != joint_count:
        raise ValueError(
            f'{label} must have shape (N, {joint_count}), got {array.shape}'
        )
    return array
