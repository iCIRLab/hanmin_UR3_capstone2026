"""Locate the system or user-local MuJoCo ros2_control installation."""

import os
from pathlib import Path
import sys

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
)


PACKAGE_NAME = 'mujoco_ros2_control'
LOCAL_INSTALL = Path.home() / '.local' / 'ros-humble-mujoco' / 'opt' / 'ros' / 'humble'


def _prepend_paths(variable: str, paths: list[Path]) -> None:
    """Prepend existing paths once while preserving the current environment."""
    current = [
        value
        for value in os.environ.get(variable, '').split(os.pathsep)
        if value
    ]
    additions = [str(path) for path in paths if path.is_dir()]
    os.environ[variable] = os.pathsep.join(
        list(dict.fromkeys([*additions, *current]))
    )


def _append_paths(variable: str, paths: list[Path]) -> None:
    """Append dependency prefixes without shadowing a workspace override."""
    current = [
        value
        for value in os.environ.get(variable, '').split(os.pathsep)
        if value
    ]
    additions = [str(path) for path in paths if path.is_dir()]
    os.environ[variable] = os.pathsep.join(
        list(dict.fromkeys([*current, *additions]))
    )


def configure_local_dependencies(prefix: Path = LOCAL_INSTALL) -> Path:
    """Expose local MuJoCo libraries while retaining the workspace core package."""
    resolved = Path(prefix).expanduser().resolve()
    _append_paths('AMENT_PREFIX_PATH', [resolved])
    _prepend_paths(
        'LD_LIBRARY_PATH',
        [resolved / 'lib', resolved / 'opt' / 'mujoco_vendor' / 'lib'],
    )
    return resolved


def configure_local_runtime(prefix: Path = LOCAL_INSTALL) -> Path:
    """Add an extracted user-local installation to this launch environment."""
    resolved = Path(prefix).expanduser().resolve()
    marker = (
        resolved
        / 'share'
        / 'ament_index'
        / 'resource_index'
        / 'packages'
        / PACKAGE_NAME
    )
    executable = resolved / 'lib' / PACKAGE_NAME / 'ros2_control_node'
    if not marker.is_file() or not executable.is_file():
        raise RuntimeError(
            f'{PACKAGE_NAME} is not installed in /opt/ros/humble or '
            f'{resolved}. Install ros-humble-mujoco-ros2-control.'
        )

    python_site = (
        resolved
        / 'local'
        / 'lib'
        / f'python{sys.version_info.major}.{sys.version_info.minor}'
        / 'dist-packages'
    )
    _prepend_paths('AMENT_PREFIX_PATH', [resolved])
    _prepend_paths(
        'LD_LIBRARY_PATH',
        [resolved / 'lib', resolved / 'opt' / 'mujoco_vendor' / 'lib'],
    )
    _prepend_paths('PATH', [resolved / 'lib' / PACKAGE_NAME])
    _prepend_paths('PYTHONPATH', [python_site])
    return resolved


def ensure_mujoco_runtime() -> Path:
    """Return the usable package prefix, configuring the known local fallback."""
    try:
        package_prefix = Path(get_package_prefix(PACKAGE_NAME))
        # A source-built workspace override still uses the user-local vendor,
        # msgs and plugin packages. Append that dependency prefix so it never
        # shadows the workspace's patched mujoco_ros2_control executable.
        if package_prefix.resolve() != LOCAL_INSTALL.resolve():
            configure_local_dependencies()
        return package_prefix
    except PackageNotFoundError:
        local_prefix = configure_local_runtime()

    try:
        return Path(get_package_prefix(PACKAGE_NAME))
    except PackageNotFoundError as error:
        raise RuntimeError(
            f'Configured {local_prefix}, but ROS still cannot discover '
            f'{PACKAGE_NAME}.'
        ) from error
