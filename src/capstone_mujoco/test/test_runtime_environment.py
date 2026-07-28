"""Tests for the user-local MuJoCo runtime fallback."""

import os
from pathlib import Path

import pytest

from capstone_mujoco.runtime_environment import (
    configure_local_dependencies,
    configure_local_runtime,
)


def _make_runtime(root: Path) -> Path:
    marker = (
        root
        / 'share'
        / 'ament_index'
        / 'resource_index'
        / 'packages'
        / 'mujoco_ros2_control'
    )
    executable = root / 'lib' / 'mujoco_ros2_control' / 'ros2_control_node'
    vendor = root / 'opt' / 'mujoco_vendor' / 'lib'
    python_site = root / 'local' / 'lib' / 'python3.10' / 'dist-packages'
    for directory in (
        marker.parent,
        executable.parent,
        vendor,
        python_site,
    ):
        directory.mkdir(parents=True, exist_ok=True)
    marker.touch()
    executable.touch()
    return root


def test_configure_local_runtime_prepends_required_paths(
    tmp_path,
    monkeypatch,
):
    prefix = _make_runtime(tmp_path / 'runtime')
    for variable in (
        'AMENT_PREFIX_PATH',
        'LD_LIBRARY_PATH',
        'PATH',
        'PYTHONPATH',
    ):
        monkeypatch.setenv(variable, '/existing')

    assert configure_local_runtime(prefix) == prefix.resolve()
    assert os.environ['AMENT_PREFIX_PATH'].split(os.pathsep)[0] == str(prefix)
    assert str(prefix / 'lib') in os.environ['LD_LIBRARY_PATH'].split(os.pathsep)
    assert (
        str(prefix / 'opt' / 'mujoco_vendor' / 'lib')
        in os.environ['LD_LIBRARY_PATH'].split(os.pathsep)
    )
    assert (
        os.environ['PATH'].split(os.pathsep)[0]
        == str(prefix / 'lib' / 'mujoco_ros2_control')
    )


def test_configure_local_runtime_does_not_duplicate_paths(
    tmp_path,
    monkeypatch,
):
    prefix = _make_runtime(tmp_path / 'runtime')
    monkeypatch.setenv('AMENT_PREFIX_PATH', str(prefix))

    configure_local_runtime(prefix)
    configure_local_runtime(prefix)

    assert os.environ['AMENT_PREFIX_PATH'].split(os.pathsep).count(
        str(prefix)
    ) == 1


def test_configure_local_runtime_rejects_incomplete_install(tmp_path):
    with pytest.raises(RuntimeError, match='not installed'):
        configure_local_runtime(tmp_path / 'missing')


def test_configure_local_dependencies_does_not_shadow_workspace(
    tmp_path,
    monkeypatch,
):
    prefix = _make_runtime(tmp_path / 'runtime')
    monkeypatch.setenv('AMENT_PREFIX_PATH', '/workspace/install')
    monkeypatch.setenv('LD_LIBRARY_PATH', '/system/lib')

    assert configure_local_dependencies(prefix) == prefix.resolve()
    assert os.environ['AMENT_PREFIX_PATH'].split(os.pathsep) == [
        '/workspace/install',
        str(prefix),
    ]
    assert os.environ['LD_LIBRARY_PATH'].split(os.pathsep)[0] == str(
        prefix / 'lib'
    )
