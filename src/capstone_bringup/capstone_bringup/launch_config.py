"""Validation helpers for the unified UR3 arm launch."""


def parse_launch_bool(value: str, label: str) -> bool:
    """Parse an explicit ROS launch boolean."""
    normalized = str(value).strip().lower()
    if normalized not in {'true', 'false'}:
        raise ValueError(f'{label} must be true or false, got {value!r}')
    return normalized == 'true'


def validate_arm_launch_options(
    backend: str,
    planner: str,
    *,
    plan_only: bool,
    execute: bool,
    confirm_real_hardware: bool,
) -> tuple[str, str]:
    """Validate backend/planner selection and return normalized values."""
    normalized_backend = str(backend).strip().lower()
    normalized_planner = str(planner).strip().lower()
    if normalized_backend not in {'mujoco', 'real'}:
        raise ValueError('backend must be mujoco or real')
    if normalized_planner not in {'none', 'mplib', 'moveit', 'interactive'}:
        raise ValueError(
            'planner must be none, mplib, moveit, or interactive'
        )
    if plan_only and execute:
        raise ValueError('execute=true conflicts with plan_only=true')
    if not plan_only and not execute:
        raise ValueError(
            'plan_only=false requires the second execution gate execute=true'
        )
    if normalized_planner == 'none' and execute:
        raise ValueError('execute=true requires mplib or moveit')
    if normalized_backend == 'real' and not confirm_real_hardware:
        raise ValueError(
            'Real UR3 launch requires confirm_real_hardware=true after '
            'completing the safety checklist'
        )
    return normalized_backend, normalized_planner
