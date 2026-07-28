from pathlib import Path
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pinocchio as pin
import yaml

from capstone_mujoco.robot_description import build_robot_description
from capstone_planning.model import (
    HOME_Q,
    JOINT_ORDER,
    LOW_PICK_Q,
    SCAN_Q,
    load_joint_limits,
    prepare_mplib_urdf,
)
from capstone_planning.mplib_planner import Ur3MplibPlanner
from capstone_planning.workcell import add_mplib_workcell


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE_ROOT.parents[1]
PLANNING = WORKSPACE / 'src' / 'capstone_planning'
URDF = PLANNING / 'description' / 'ur3_calibrated.urdf'
SCENE = PACKAGE_ROOT / 'description' / 'ur3_scene.xml'
CONTROLLERS = PACKAGE_ROOT / 'config' / 'controllers.yaml'
LIMITS = PLANNING / 'config' / 'ur3_planning_limits.yaml'


def test_mujoco_scene_loads_and_has_matching_position_actuators():
    model = mujoco.MjModel.from_xml_path(str(SCENE))

    joint_names = tuple(
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index)
        for index in range(model.njnt)
    )
    actuator_names = tuple(
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)
        for index in range(model.nu)
    )

    assert joint_names == JOINT_ORDER
    assert actuator_names == JOINT_ORDER
    assert model.nq == 6
    assert model.nu == 6
    assert model.nkey == 2


def test_home_keyframe_is_consistent():
    model = mujoco.MjModel.from_xml_path(str(SCENE))

    assert mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, 0) == 'home'
    assert tuple(model.key_qpos[0]) == HOME_Q
    assert tuple(model.key_ctrl[0]) == HOME_Q


def test_scan_keyframe_is_consistent():
    model = mujoco.MjModel.from_xml_path(str(SCENE))

    assert mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, 1) == 'scan'
    assert tuple(model.key_qpos[1]) == SCAN_Q
    assert tuple(model.key_ctrl[1]) == SCAN_Q


def test_scene_uses_official_ur3_visual_and_collision_meshes():
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    collision_names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index)
        for index in range(model.ngeom)
        if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index)
    }
    expected = {
        'base_link_inertia_collision',
        'shoulder_link_collision',
        'upper_arm_link_collision',
        'forearm_link_collision',
        'wrist_1_link_collision',
        'wrist_2_link_collision',
        'wrist_3_link_collision',
    }

    assert expected <= collision_names
    assert model.nmesh == 24
    for name in expected:
        geom_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_GEOM,
            name,
        )
        assert model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_MESH
        assert model.geom_contype[geom_id] == 1
        assert model.geom_conaffinity[geom_id] == 2


def test_scene_contains_workcell_and_diagnostic_sensors():
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    geom_names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index)
        for index in range(model.ngeom)
    }
    sensor_names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, index)
        for index in range(model.nsensor)
    }

    assert {
        'floor',
        'pickup_station_top',
    } <= geom_names
    assert 'route_obstacle_geom' not in geom_names
    assert 'static_target_can_geom' not in geom_names
    assert model.nsensor == 14
    assert {
        'shoulder_pan_position',
        'wrist_3_velocity',
        'tool0_position',
        'tool0_linear_velocity',
    } <= sensor_names


def test_home_is_stable_under_mujoco_dynamics():
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)

    for _ in range(1000):
        mujoco.mj_step(model, data)

    assert np.max(np.abs(data.qpos - np.asarray(HOME_Q))) < 2.0e-5
    assert data.ncon == 0


def test_scan_is_stable_under_mujoco_dynamics():
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 1)

    for _ in range(1000):
        mujoco.mj_step(model, data)

    # Gravity produces a small steady-state offset with the same position
    # actuators used by ros2_control. Keep it within the trajectory executor's
    # accepted final-position tolerance.
    assert np.max(np.abs(data.qpos - np.asarray(SCAN_Q))) < 2.0e-2
    assert np.max(np.abs(data.qvel)) < 2.0e-6
    assert data.ncon == 0


def test_planned_path_clears_the_edited_mujoco_table():
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)

    def contact_count(path):
        count = 0
        for qpos in path:
            data.qpos[:] = qpos
            data.qvel[:] = 0.0
            mujoco.mj_forward(model, data)
            count += int(data.ncon > 0)
        return count

    planner = Ur3MplibPlanner()
    try:
        add_mplib_workcell(planner.planner)
        result = planner.plan_joint_goal(
            HOME_Q,
            LOW_PICK_Q,
            planning_time=5.0,
        )
        assert result.success, result.message
        assert contact_count(result.position) == 0
    finally:
        planner.close()


def test_pinocchio_and_mujoco_tool0_kinematics_match():
    mujoco_model = mujoco.MjModel.from_xml_path(str(SCENE))
    mujoco_data = mujoco.MjData(mujoco_model)
    tool_site = mujoco.mj_name2id(
        mujoco_model,
        mujoco.mjtObj.mjOBJ_SITE,
        'tool0',
    )
    limits = load_joint_limits(LIMITS)

    with TemporaryDirectory() as directory:
        bounded_urdf = prepare_mplib_urdf(
            URDF,
            Path(directory),
            limits=limits,
        )
        pin_model = pin.buildModelFromUrdf(str(bounded_urdf))
        pin_data = pin_model.createData()
        tool_frame = pin_model.getFrameId('tool0')

        samples = (
            np.zeros(6),
            np.asarray(HOME_Q),
            np.asarray(SCAN_Q),
            np.asarray((0.2, -1.2, 0.4, -1.7, 0.3, -0.4)),
        )
        for qpos in samples:
            pin.forwardKinematics(pin_model, pin_data, qpos)
            pin.updateFramePlacements(pin_model, pin_data)
            pin_pose = pin_data.oMf[tool_frame]

            mujoco_data.qpos[:] = qpos
            mujoco.mj_forward(mujoco_model, mujoco_data)
            mujoco_position = mujoco_data.site_xpos[tool_site]
            mujoco_rotation = mujoco_data.site_xmat[tool_site].reshape(3, 3)

            assert np.linalg.norm(
                pin_pose.translation - mujoco_position
            ) < 1.0e-6
            assert np.linalg.norm(
                pin_pose.rotation - mujoco_rotation
            ) < 2.0e-6


def test_generated_ros2_control_description_uses_same_contract():
    description = build_robot_description(URDF, SCENE, headless=True)
    root = ET.fromstring(description)
    control = root.find('ros2_control')

    assert control is not None
    assert control.findtext('hardware/plugin') == (
        'mujoco_ros2_control/MujocoSystemInterface'
    )
    parameters = {
        element.attrib['name']: element.text
        for element in control.findall('hardware/param')
    }
    assert parameters['mujoco_model'] == str(SCENE.resolve())
    assert parameters['headless'] == 'true'
    assert parameters['initial_keyframe'] == 'home'
    assert parameters['render_fps'] == '60.0'
    assert parameters['render_vsync'] == 'false'
    assert parameters['window_width'] == '1100'
    assert parameters['window_height'] == '620'
    assert parameters['ui_font_scale'] == '100'

    joints = control.findall('joint')
    assert tuple(joint.attrib['name'] for joint in joints) == JOINT_ORDER
    for joint, home_position in zip(joints, HOME_Q):
        assert joint.find('command_interface').attrib['name'] == 'position'
        assert float(
            joint.find("state_interface[@name='position']/param").text
        ) == home_position


def test_generated_ros2_control_description_can_start_at_scan():
    description = build_robot_description(
        URDF,
        SCENE,
        headless=False,
        initial_pose='scan',
    )
    root = ET.fromstring(description)
    control = root.find('ros2_control')

    assert control is not None
    parameters = {
        element.attrib['name']: element.text
        for element in control.findall('hardware/param')
    }
    assert parameters['headless'] == 'false'
    assert parameters['initial_keyframe'] == 'scan'

    joints = control.findall('joint')
    for joint, scan_position in zip(joints, SCAN_Q):
        assert float(
            joint.find("state_interface[@name='position']/param").text
        ) == scan_position


def test_generated_ros2_control_description_rejects_unknown_pose():
    try:
        build_robot_description(
            URDF,
            SCENE,
            initial_pose='not-a-pose',
        )
    except ValueError as error:
        assert 'initial_pose must be one of' in str(error)
    else:
        raise AssertionError('Unknown initial pose was accepted')


def test_controller_joint_order_matches_common_model():
    content = yaml.safe_load(CONTROLLERS.read_text(encoding='utf-8'))
    parameters = content['joint_trajectory_controller']['ros__parameters']

    assert tuple(parameters['joints']) == JOINT_ORDER
    assert parameters['command_interfaces'] == ['position']
    assert parameters['state_interfaces'] == ['position', 'velocity']
    assert parameters['allow_partial_joints_goal'] is False
