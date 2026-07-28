"""Name-based UR3 joint-state validation independent of ROS message order."""

from dataclasses import dataclass
from typing import Sequence
import math
import time

from capstone_planning.model import JOINT_ORDER


@dataclass(frozen=True)
class JointStateSnapshot:
    """One canonical, time-stamped six-joint state."""

    positions: tuple[float, ...]
    received_monotonic: float

    def age(self, now: float | None = None) -> float:
        """Return state age in seconds using a monotonic clock."""
        current = time.monotonic() if now is None else float(now)
        return max(0.0, current - self.received_monotonic)


def reorder_joint_positions(
    names: Sequence[str],
    positions: Sequence[float],
) -> tuple[float, ...]:
    """Return UR3 positions in canonical order or raise on invalid input."""
    if len(names) != len(positions):
        raise ValueError(
            f'JointState name/position lengths differ: '
            f'{len(names)} != {len(positions)}'
        )
    if len(set(names)) != len(names):
        raise ValueError('JointState contains duplicate joint names')

    by_name = dict(zip(names, positions))
    missing = [name for name in JOINT_ORDER if name not in by_name]
    if missing:
        raise ValueError(f'JointState is missing UR3 joints: {missing}')

    ordered = tuple(float(by_name[name]) for name in JOINT_ORDER)
    if not all(math.isfinite(value) for value in ordered):
        raise ValueError('JointState contains NaN or infinity')
    return ordered


class JointStateBuffer:
    """Store and validate the newest canonical joint state."""

    def __init__(self) -> None:
        self._snapshot: JointStateSnapshot | None = None

    def update(
        self,
        names: Sequence[str],
        positions: Sequence[float],
        received_monotonic: float | None = None,
    ) -> JointStateSnapshot:
        """Validate and store a new state."""
        received = (
            time.monotonic()
            if received_monotonic is None
            else float(received_monotonic)
        )
        snapshot = JointStateSnapshot(
            reorder_joint_positions(names, positions),
            received,
        )
        self._snapshot = snapshot
        return snapshot

    def get(
        self,
        max_age: float,
        now: float | None = None,
    ) -> JointStateSnapshot:
        """Return a fresh state or raise if none is available."""
        if self._snapshot is None:
            raise RuntimeError('No valid UR3 JointState has been received')
        if self._snapshot.age(now) > float(max_age):
            raise RuntimeError(
                f'UR3 JointState is stale: '
                f'{self._snapshot.age(now):.3f}s > {float(max_age):.3f}s'
            )
        return self._snapshot
