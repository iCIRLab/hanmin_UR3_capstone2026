import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from capstone_planning.model import HOME_Q, LOW_PICK_Q
from capstone_planning.mplib_planner import Ur3MplibPlanner
from capstone_planning.workcell import (
    WORKCELL_BOXES,
    add_mplib_workcell,
    moveit_planning_scene,
)


def test_moveit_scene_uses_the_common_box_geometry():
    scene = moveit_planning_scene()

    assert scene.is_diff is True
    assert scene.robot_state.is_diff is True
    assert len(scene.world.collision_objects) == len(WORKCELL_BOXES)
    for expected, message in zip(
        WORKCELL_BOXES,
        scene.world.collision_objects,
    ):
        assert message.header.frame_id == 'world'
        assert message.id == expected.name
        assert tuple(message.primitives[0].dimensions) == pytest.approx(
            expected.collision_size
        )
        pose = message.primitive_poses[0]
        assert (
            pose.position.x,
            pose.position.y,
            pose.position.z,
        ) == expected.center


def test_mujoco_boxes_match_the_common_planning_geometry():
    workspace_src = Path(__file__).resolve().parents[2]
    root = ET.parse(
        workspace_src / 'capstone_mujoco' / 'description' / 'ur3_scene.xml'
    ).getroot()
    mujoco_names = {
        'pickup_table_top': 'pickup_station_top',
    }

    for obstacle in WORKCELL_BOXES:
        geom = root.find(f".//geom[@name='{mujoco_names[obstacle.name]}']")
        assert geom is not None
        center = tuple(float(value) for value in geom.attrib['pos'].split())
        half_size = tuple(
            float(value) for value in geom.attrib['size'].split()
        )
        assert center == pytest.approx(obstacle.center)
        assert tuple(2.0 * value for value in half_size) == pytest.approx(
            obstacle.size
        )


def test_mplib_named_goal_path_clears_the_edited_table():
    planner = Ur3MplibPlanner()
    try:
        add_mplib_workcell(planner.planner)
        start = np.asarray(HOME_Q)
        goal = np.asarray(LOW_PICK_Q)

        assert not planner.planner.check_for_env_collision(start)
        assert not planner.planner.check_for_env_collision(goal)
        result = planner.plan_joint_goal(
            start,
            goal,
            rrt_range=0.08,
            planning_time=5.0,
        )
        assert result.success, result.message
        assert all(
            not planner.planner.check_for_env_collision(qpos)
            for qpos in result.position
        )
    finally:
        planner.close()
