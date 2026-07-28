"""Build the MuJoCo ros2_control robot description from the calibrated URDF."""

from pathlib import Path
import xml.etree.ElementTree as ET

from capstone_planning.model import HOME_Q, JOINT_ORDER, SCAN_Q


INITIAL_POSES = {
    'home': HOME_Q,
    'scan': SCAN_Q,
}


def build_robot_description(
    source_urdf: Path,
    mujoco_model: Path,
    *,
    headless: bool = True,
    initial_pose: str = 'home',
    render_fps: float = 60.0,
    render_vsync: bool = False,
    window_width: int = 1100,
    window_height: int = 620,
    ui_font_scale: int = 100,
) -> str:
    """Append a MuJoCo hardware interface to the calibrated planning URDF."""
    source = Path(source_urdf)
    model = Path(mujoco_model)
    if not source.is_file():
        raise FileNotFoundError(source)
    if not model.is_file():
        raise FileNotFoundError(model)
    if initial_pose not in INITIAL_POSES:
        raise ValueError(
            f'initial_pose must be one of {tuple(INITIAL_POSES)}, '
            f'got {initial_pose!r}'
        )
    initial_positions = INITIAL_POSES[initial_pose]

    root = ET.parse(source).getroot()
    if root.tag != 'robot':
        raise ValueError(f'Expected a robot root element, got {root.tag!r}')
    if root.find('ros2_control') is not None:
        raise ValueError('Source URDF already contains a ros2_control element')

    control = ET.SubElement(
        root,
        'ros2_control',
        {'name': 'MujocoSystem', 'type': 'system'},
    )
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = (
        'mujoco_ros2_control/MujocoSystemInterface'
    )
    parameters = {
        'mujoco_model': str(model.resolve()),
        'sim_speed_factor': '1.0',
        'initial_keyframe': initial_pose,
        'headless': str(bool(headless)).lower(),
        'render_fps': str(float(render_fps)),
        'render_vsync': str(bool(render_vsync)).lower(),
        'window_width': str(int(window_width)),
        'window_height': str(int(window_height)),
        'ui_font_scale': str(int(ui_font_scale)),
    }
    for name, value in parameters.items():
        ET.SubElement(hardware, 'param', {'name': name}).text = value

    for joint_name, initial_position in zip(JOINT_ORDER, initial_positions):
        joint = ET.SubElement(control, 'joint', {'name': joint_name})
        ET.SubElement(joint, 'command_interface', {'name': 'position'})
        position_state = ET.SubElement(
            joint,
            'state_interface',
            {'name': 'position'},
        )
        ET.SubElement(
            position_state,
            'param',
            {'name': 'initial_value'},
        ).text = str(initial_position)
        ET.SubElement(joint, 'state_interface', {'name': 'velocity'})
        ET.SubElement(joint, 'state_interface', {'name': 'effort'})

    return ET.tostring(root, encoding='unicode')
