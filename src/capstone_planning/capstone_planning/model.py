"""Shared UR3 model metadata and MPlib model preparation."""

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Tuple
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import yaml


JOINT_ORDER: Tuple[str, ...] = (
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
)

PLANNING_GROUP = 'ur3_manipulator'
BASE_LINK = 'base_link'
TCP_LINK = 'tool0'

HOME_Q: Tuple[float, ...] = (
    0.0,
    -1.5707963267948966,
    0.0,
    -1.5707963267948966,
    0.0,
    0.0,
)
SCAN_Q: Tuple[float, ...] = (
    0.0,
    -1.5707963267948966,
    1.5707963267948966,
    -2.356194490192345,
    -1.5707963267948966,
    0.0,
)

# A collision-free low working pose relative to the mounted UR3 base.
# With base_link at world z=0.715 m, tool0 is approximately
# [0.420, 0.120, 1.015] m in world (0.305 m above the edited table surface).
# It was solved from the calibrated UR3 model while preserving scan orientation.
LOW_PICK_Q: Tuple[float, ...] = (
    0.018501164137118664,
    -1.1286313322509012,
    1.483204466131257,
    -2.7106767891954213,
    -1.5854806514595434,
    0.011467514178686855,
)

NAMED_GOALS: Dict[str, Tuple[float, ...]] = {
    'home': HOME_Q,
    'scan': SCAN_Q,
    'low_pick': LOW_PICK_Q,
}


@dataclass(frozen=True)
class JointLimit:
    """Position, velocity, and acceleration limits for one joint."""

    lower: float
    upper: float
    velocity: float
    acceleration: float


def package_share() -> Path:
    """Return the installed capstone_planning share directory."""
    return Path(get_package_share_directory('capstone_planning'))


def description_paths(share: Path | None = None) -> tuple[Path, Path]:
    """Return the concrete calibrated URDF and SRDF paths."""
    root = package_share() if share is None else Path(share)
    description = root / 'description'
    return description / 'ur3_calibrated.urdf', description / 'ur3.srdf'


def planning_limits_path(share: Path | None = None) -> Path:
    """Return the shared planning-limit YAML path."""
    root = package_share() if share is None else Path(share)
    return root / 'config' / 'ur3_planning_limits.yaml'


def load_joint_limits(path: Path | None = None) -> Dict[str, JointLimit]:
    """Load and validate the common six-joint planning limits."""
    limits_path = planning_limits_path() if path is None else Path(path)
    content = yaml.safe_load(limits_path.read_text(encoding='utf-8'))
    raw_limits = content.get('joint_limits', {})

    if tuple(raw_limits) != JOINT_ORDER:
        raise ValueError(
            'Planning-limit joint order does not match JOINT_ORDER: '
            f'{tuple(raw_limits)}'
        )

    limits: Dict[str, JointLimit] = {}
    for name in JOINT_ORDER:
        raw = raw_limits[name]
        required_flags = (
            'has_position_limits',
            'has_velocity_limits',
            'has_acceleration_limits',
        )
        if not all(raw.get(flag) is True for flag in required_flags):
            raise ValueError(f'{name} must define all planning limit types')

        limit = JointLimit(
            lower=float(raw['min_position']),
            upper=float(raw['max_position']),
            velocity=float(raw['max_velocity']),
            acceleration=float(raw['max_acceleration']),
        )
        if not limit.lower < limit.upper:
            raise ValueError(f'Invalid position range for {name}')
        if limit.velocity <= 0.0 or limit.acceleration <= 0.0:
            raise ValueError(f'Non-positive motion limit for {name}')
        limits[name] = limit
    return limits


def ordered_values(
    limits: Dict[str, JointLimit],
    attribute: str,
) -> tuple[float, ...]:
    """Read one JointLimit attribute in canonical joint order."""
    return tuple(float(getattr(limits[name], attribute)) for name in JOINT_ORDER)


def bounded_urdf_text(
    source_urdf: Path,
    limits: Dict[str, JointLimit] | None = None,
) -> str:
    """Return the URDF with all six planning joints explicitly bounded."""
    source = Path(source_urdf)
    joint_limits = load_joint_limits() if limits is None else limits
    if not source.is_file():
        raise FileNotFoundError(source)

    tree = ET.parse(source)
    root = tree.getroot()
    active_joints = {
        joint.attrib['name']: joint
        for joint in root.findall('joint')
        if joint.attrib.get('type') in {'revolute', 'continuous', 'prismatic'}
    }
    if tuple(active_joints) != JOINT_ORDER:
        raise ValueError(
            f'URDF active-joint order does not match JOINT_ORDER: '
            f'{tuple(active_joints)}'
        )

    for name in JOINT_ORDER:
        joint = active_joints[name]
        joint.set('type', 'revolute')
        limit_element = joint.find('limit')
        if limit_element is None:
            raise ValueError(f'Missing URDF limit element for {name}')
        limit = joint_limits[name]
        limit_element.set('lower', str(limit.lower))
        limit_element.set('upper', str(limit.upper))
        limit_element.set('velocity', str(limit.velocity))

    return ET.tostring(root, encoding='unicode')


def prepare_mplib_urdf(
    source_urdf: Path,
    output_directory: Path,
    ur_description_share: Path | None = None,
    limits: Dict[str, JointLimit] | None = None,
) -> Path:
    """
    Create the bounded, resource-resolvable URDF required by MPlib 0.2.1.

    MPlib resolves mesh paths relative to the input URDF and rejects continuous
    revolute joints in its OMPL planner. A generated working directory therefore
    contains a link to ``ur_description`` and a derived URDF whose package URIs
    are relative and whose active joints use the common bounded limits.
    """
    source = Path(source_urdf)
    output = Path(output_directory)
    description_share = (
        Path(get_package_share_directory('ur_description'))
        if ur_description_share is None
        else Path(ur_description_share)
    )
    joint_limits = load_joint_limits() if limits is None else limits

    if not source.is_file():
        raise FileNotFoundError(source)
    if not description_share.is_dir():
        raise FileNotFoundError(description_share)

    output.mkdir(parents=True, exist_ok=True)
    resource_link = output / 'ur_description'
    if resource_link.is_symlink():
        if resource_link.resolve() != description_share.resolve():
            resource_link.unlink()
    elif resource_link.exists():
        raise FileExistsError(
            f'MPlib resource path exists and is not a symlink: {resource_link}'
        )
    if not resource_link.exists():
        resource_link.symlink_to(description_share, target_is_directory=True)

    tree = ET.parse(source)
    root = tree.getroot()
    active_joints = {
        joint.attrib['name']: joint
        for joint in root.findall('joint')
        if joint.attrib.get('type') in {'revolute', 'continuous', 'prismatic'}
    }
    if tuple(active_joints) != JOINT_ORDER:
        raise ValueError(
            f'URDF active-joint order does not match JOINT_ORDER: '
            f'{tuple(active_joints)}'
        )

    for name in JOINT_ORDER:
        joint = active_joints[name]
        joint.set('type', 'revolute')
        limit_element = joint.find('limit')
        if limit_element is None:
            raise ValueError(f'Missing URDF limit element for {name}')
        limit = joint_limits[name]
        limit_element.set('lower', str(limit.lower))
        limit_element.set('upper', str(limit.upper))
        limit_element.set('velocity', str(limit.velocity))

    for mesh in root.findall('.//mesh'):
        filename = mesh.attrib.get('filename', '')
        prefix = 'package://'
        if filename.startswith(prefix):
            mesh.set('filename', filename.removeprefix(prefix))

    derived_path = output / 'ur3_mplib.urdf'
    tree.write(derived_path, encoding='utf-8', xml_declaration=True)
    return derived_path


def validate_joint_vector(values: Iterable[float], label: str) -> tuple[float, ...]:
    """Return a finite six-joint tuple or raise a descriptive ValueError."""
    vector = tuple(float(value) for value in values)
    if len(vector) != len(JOINT_ORDER):
        raise ValueError(
            f'{label} must contain {len(JOINT_ORDER)} values, got {len(vector)}'
        )
    if any(value != value or abs(value) == float('inf') for value in vector):
        raise ValueError(f'{label} contains NaN or infinity')
    return vector
