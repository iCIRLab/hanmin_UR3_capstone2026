from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml

from capstone_planning.model import (
    bounded_urdf_text,
    JOINT_ORDER,
    load_joint_limits,
    prepare_mplib_urdf,
)


PACKAGE_DIR = Path(__file__).resolve().parents[1]
WORKSPACE_DIR = Path(__file__).resolve().parents[3]
DESCRIPTION_DIR = PACKAGE_DIR / 'description'
URDF_PATH = DESCRIPTION_DIR / 'ur3_calibrated.urdf'
SRDF_PATH = DESCRIPTION_DIR / 'ur3.srdf'
CALIBRATION_PATH = WORKSPACE_DIR / 'ur3_calibration.yaml'
UR_DESCRIPTION_SHARE = Path(get_package_share_directory('ur_description'))

JOINT_TO_CALIBRATION_SECTION = {
    'shoulder_pan_joint': 'shoulder',
    'shoulder_lift_joint': 'upper_arm',
    'elbow_joint': 'forearm',
    'wrist_1_joint': 'wrist_1',
    'wrist_2_joint': 'wrist_2',
    'wrist_3_joint': 'wrist_3',
}


def _parse(path: Path) -> ET.Element:
    assert path.is_file(), f'Missing generated model: {path}'
    return ET.parse(path).getroot()


def _numbers(value: str) -> list[float]:
    return [float(item) for item in value.split()]


def test_generated_files_are_concrete_xml():
    urdf_root = _parse(URDF_PATH)
    srdf_root = _parse(SRDF_PATH)

    assert urdf_root.tag == 'robot'
    assert srdf_root.tag == 'robot'
    assert urdf_root.attrib['name'] == 'ur3'
    assert srdf_root.attrib['name'] == 'ur3'
    assert not urdf_root.findall('.//{http://wiki.ros.org/xacro}*')
    assert not srdf_root.findall('.//{http://wiki.ros.org/xacro}*')


def test_planning_urdf_has_expected_active_joint_order():
    root = _parse(URDF_PATH)
    active_joint_types = {'revolute', 'continuous', 'prismatic'}
    active_joint_names = [
        joint.attrib['name']
        for joint in root.findall('joint')
        if joint.attrib['type'] in active_joint_types
    ]

    assert tuple(active_joint_names) == JOINT_ORDER
    assert root.find("./link[@name='base_link']") is not None
    assert root.find("./link[@name='tool0']") is not None
    assert root.find('ros2_control') is None

    base_joint = root.find("./joint[@name='base_joint']/origin")
    assert base_joint is not None
    assert _numbers(base_joint.attrib['xyz']) == pytest.approx(
        [0.0, 0.0, 0.715]
    )


def test_urdf_joint_origins_match_this_robot_calibration():
    root = _parse(URDF_PATH)
    calibration = yaml.safe_load(CALIBRATION_PATH.read_text())['kinematics']

    for joint_name, section_name in JOINT_TO_CALIBRATION_SECTION.items():
        joint = root.find(f"./joint[@name='{joint_name}']")
        assert joint is not None
        origin = joint.find('origin')
        assert origin is not None

        section = calibration[section_name]
        expected_xyz = [section['x'], section['y'], section['z']]
        expected_rpy = [section['roll'], section['pitch'], section['yaw']]

        assert _numbers(origin.attrib['xyz']) == pytest.approx(expected_xyz)
        assert _numbers(origin.attrib['rpy']) == pytest.approx(expected_rpy)


def test_all_collision_mesh_files_exist():
    root = _parse(URDF_PATH)
    meshes = root.findall('.//collision/geometry/mesh')

    assert len(meshes) == 7
    for mesh in meshes:
        uri = mesh.attrib['filename']
        prefix = 'package://ur_description/'
        assert uri.startswith(prefix)
        resolved_path = UR_DESCRIPTION_SHARE / uri.removeprefix(prefix)
        assert resolved_path.is_file(), f'Missing collision mesh: {resolved_path}'


def test_srdf_references_only_existing_urdf_links():
    urdf_root = _parse(URDF_PATH)
    srdf_root = _parse(SRDF_PATH)
    urdf_links = {link.attrib['name'] for link in urdf_root.findall('link')}

    chain = srdf_root.find("./group[@name='ur3_manipulator']/chain")
    assert chain is not None
    assert chain.attrib == {'base_link': 'base_link', 'tip_link': 'tool0'}

    disabled_pairs = srdf_root.findall('disable_collisions')
    assert disabled_pairs
    for pair in disabled_pairs:
        assert pair.attrib['link1'] in urdf_links
        assert pair.attrib['link2'] in urdf_links


def test_common_planning_limits_cover_all_active_joints():
    limits_path = PACKAGE_DIR / 'config' / 'ur3_planning_limits.yaml'
    limits = load_joint_limits(limits_path)

    assert tuple(limits) == JOINT_ORDER
    for limit in limits.values():
        assert limit.lower < limit.upper
        assert limit.velocity == pytest.approx(0.5)
        assert limit.acceleration == pytest.approx(0.5)


def test_moveit_model_text_has_no_continuous_planning_joint():
    root = ET.fromstring(bounded_urdf_text(URDF_PATH))
    active_joints = [
        joint
        for joint in root.findall('joint')
        if joint.attrib['name'] in JOINT_ORDER
    ]

    assert tuple(joint.attrib['name'] for joint in active_joints) == JOINT_ORDER
    assert all(joint.attrib['type'] == 'revolute' for joint in active_joints)


def test_mplib_derived_model_is_bounded_and_resolves_meshes(tmp_path):
    limits_path = PACKAGE_DIR / 'config' / 'ur3_planning_limits.yaml'
    limits = load_joint_limits(limits_path)
    derived = prepare_mplib_urdf(
        URDF_PATH,
        tmp_path,
        UR_DESCRIPTION_SHARE,
        limits,
    )

    root = _parse(derived)
    active_joints = [
        joint
        for joint in root.findall('joint')
        if joint.attrib['type'] in {'revolute', 'continuous', 'prismatic'}
    ]
    assert tuple(joint.attrib['name'] for joint in active_joints) == JOINT_ORDER
    assert all(joint.attrib['type'] == 'revolute' for joint in active_joints)

    for joint in active_joints:
        limit = joint.find('limit')
        expected = limits[joint.attrib['name']]
        assert float(limit.attrib['lower']) == pytest.approx(expected.lower)
        assert float(limit.attrib['upper']) == pytest.approx(expected.upper)

    for mesh in root.findall('.//mesh'):
        mesh_path = derived.parent / mesh.attrib['filename']
        assert mesh_path.is_file(), f'Missing MPlib mesh: {mesh_path}'
