import math

import pytest

from capstone_planning.joint_state import (
    JointStateBuffer,
    reorder_joint_positions,
)
from capstone_planning.model import JOINT_ORDER


def test_joint_state_is_reordered_by_name():
    names = list(reversed(JOINT_ORDER))
    values = list(range(len(names)))

    assert reorder_joint_positions(names, values) == tuple(reversed(values))


@pytest.mark.parametrize(
    'names, positions, expected',
    [
        (JOINT_ORDER[:-1], [0.0] * 5, 'missing'),
        (JOINT_ORDER, [0.0] * 5, 'lengths differ'),
        (
            [JOINT_ORDER[0], *JOINT_ORDER[:-1]],
            [0.0] * 6,
            'duplicate',
        ),
        (JOINT_ORDER, [0.0] * 5 + [math.nan], 'NaN'),
    ],
)
def test_invalid_joint_state_is_rejected(names, positions, expected):
    with pytest.raises(ValueError, match=expected):
        reorder_joint_positions(names, positions)


def test_joint_state_buffer_rejects_missing_and_stale_state():
    buffer = JointStateBuffer()
    with pytest.raises(RuntimeError, match='No valid'):
        buffer.get(max_age=0.5, now=10.0)

    buffer.update(JOINT_ORDER, [0.0] * 6, received_monotonic=10.0)
    assert buffer.get(max_age=0.5, now=10.4).positions == (0.0,) * 6
    with pytest.raises(RuntimeError, match='stale'):
        buffer.get(max_age=0.5, now=10.6)
