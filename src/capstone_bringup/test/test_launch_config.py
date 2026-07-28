import pytest

from capstone_bringup.launch_config import (
    parse_launch_bool,
    validate_arm_launch_options,
)


def test_safe_default_is_accepted():
    assert validate_arm_launch_options(
        'mujoco',
        'mplib',
        plan_only=True,
        execute=False,
        confirm_real_hardware=False,
    ) == ('mujoco', 'mplib')


def test_interactive_dual_planner_is_accepted():
    assert validate_arm_launch_options(
        'mujoco',
        'interactive',
        plan_only=True,
        execute=False,
        confirm_real_hardware=False,
    ) == ('mujoco', 'interactive')


def test_real_backend_requires_explicit_confirmation():
    with pytest.raises(ValueError, match='confirm_real_hardware'):
        validate_arm_launch_options(
            'real',
            'moveit',
            plan_only=True,
            execute=False,
            confirm_real_hardware=False,
        )


def test_execution_requires_both_gates():
    with pytest.raises(ValueError, match='conflicts'):
        validate_arm_launch_options(
            'mujoco',
            'mplib',
            plan_only=True,
            execute=True,
            confirm_real_hardware=False,
        )
    with pytest.raises(ValueError, match='second execution gate'):
        validate_arm_launch_options(
            'mujoco',
            'mplib',
            plan_only=False,
            execute=False,
            confirm_real_hardware=False,
        )


@pytest.mark.parametrize('value', ['TRUE', 'false', ' True '])
def test_launch_boolean_parser(value):
    assert isinstance(parse_launch_bool(value, 'test'), bool)


def test_invalid_boolean_is_rejected():
    with pytest.raises(ValueError):
        parse_launch_bool('yes', 'test')
