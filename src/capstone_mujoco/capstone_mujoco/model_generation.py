"""Generate the calibrated UR3 MJCF with the official conversion utility."""

import argparse
from pathlib import Path
import shutil
import subprocess
import sys
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from capstone_mujoco.runtime_environment import ensure_mujoco_runtime


LINK_MESHES = {
    'base_link_inertia': 'base',
    'shoulder_link': 'shoulder',
    'upper_arm_link': 'upperarm',
    'forearm_link': 'forearm',
    'wrist_1_link': 'wrist1',
    'wrist_2_link': 'wrist2',
    'wrist_3_link': 'wrist3',
}


def _target_for_link(root: ET.Element, link_name: str) -> ET.Element:
    if link_name == 'base_link_inertia':
        target = root.find('worldbody')
    else:
        target = root.find(f".//body[@name='{link_name}']")
    if target is None:
        raise ValueError(f'MJCF body for {link_name!r} was not generated')
    return target


def replace_generated_collision_meshes(
    generated_model: Path,
    collision_mesh_directory: Path,
) -> ET.ElementTree:
    """Replace visual-mesh collision copies with official UR collision STLs."""
    tree = ET.parse(generated_model)
    root = tree.getroot()
    assets = root.find('asset')
    if assets is None:
        raise ValueError('Generated MJCF does not contain an asset section')

    for link_name, mesh_stem in LINK_MESHES.items():
        source_mesh = collision_mesh_directory / f'{mesh_stem}.stl'
        if not source_mesh.is_file():
            raise FileNotFoundError(source_mesh)
        mesh_name = f'collision_{mesh_stem}'
        ET.SubElement(
            assets,
            'mesh',
            {
                'name': mesh_name,
                'file': f'collision/{mesh_stem}.stl',
            },
        )

        target = _target_for_link(root, link_name)
        generated_collisions = [
            geom
            for geom in target.findall('geom')
            if geom.attrib.get('class') == 'collision'
        ]
        if not generated_collisions:
            raise ValueError(
                f'No generated collision geometry found for {link_name}'
            )
        placement = {
            key: value
            for key, value in generated_collisions[0].attrib.items()
            if key in {'pos', 'quat'} and value
        }
        for geom in generated_collisions:
            target.remove(geom)
        ET.SubElement(
            target,
            'geom',
            {
                'name': f'{link_name}_collision',
                'mesh': mesh_name,
                'class': 'collision',
                **placement,
            },
        )
    for element in root.iter():
        for attribute, value in tuple(element.attrib.items()):
            if not value:
                del element.attrib[attribute]
    return tree


def _copy_referenced_assets(
    tree: ET.ElementTree,
    generated_assets: Path,
    collision_mesh_directory: Path,
    destination_assets: Path,
) -> None:
    if destination_assets.exists():
        shutil.rmtree(destination_assets)

    mesh_files = {
        mesh.attrib['file']
        for mesh in tree.getroot().findall('./asset/mesh')
        if not mesh.attrib['file'].startswith('collision/')
    }
    for relative_name in sorted(mesh_files):
        source = generated_assets / relative_name
        destination = destination_assets / relative_name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        material = source.parent / 'material.mtl'
        if material.is_file():
            shutil.copy2(material, destination.parent / material.name)

    collision_destination = destination_assets / 'collision'
    collision_destination.mkdir(parents=True, exist_ok=True)
    for mesh_stem in LINK_MESHES.values():
        shutil.copy2(
            collision_mesh_directory / f'{mesh_stem}.stl',
            collision_destination / f'{mesh_stem}.stl',
        )


def generate_model(
    source_urdf: Path,
    inputs_file: Path,
    scene_template: Path,
    collision_mesh_directory: Path,
    output_directory: Path,
) -> tuple[Path, Path]:
    """Run the official converter and install only runtime MJCF assets."""
    ensure_mujoco_runtime()
    converter = (
        Path(get_package_share_directory('mujoco_ros2_control'))
        / 'scripts'
        / 'make_mjcf_from_robot_description.py'
    )
    output = Path(output_directory).resolve()
    output.mkdir(parents=True, exist_ok=True)

    with TemporaryDirectory(prefix='capstone_ur3_mjcf_') as temporary:
        generated = Path(temporary)
        subprocess.run(
            [
                sys.executable,
                str(converter),
                '--urdf',
                str(Path(source_urdf).resolve()),
                '--mujoco_inputs',
                str(Path(inputs_file).resolve()),
                '--output',
                str(generated),
                '--save_only',
                '--scene',
                str(Path(scene_template).resolve()),
            ],
            check=True,
            env=None,
        )
        generated_model = generated / 'mujoco_description_formatted.xml'
        tree = replace_generated_collision_meshes(
            generated_model,
            Path(collision_mesh_directory),
        )
        ET.indent(tree, space='  ')

        model_destination = output / 'ur3_model.xml'
        tree.write(model_destination, encoding='unicode')
        _copy_referenced_assets(
            tree,
            generated / 'assets',
            Path(collision_mesh_directory),
            output / 'assets',
        )

    scene_text = Path(scene_template).read_text(encoding='utf-8')
    scene_text = scene_text.replace(
        'mujoco_description_formatted.xml',
        'ur3_model.xml',
    )
    scene_destination = output / 'ur3_scene.xml'
    scene_destination.write_text(scene_text, encoding='utf-8')
    return model_destination, scene_destination


def main() -> None:
    """Generate package runtime assets from explicit source locations."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--inputs', type=Path, required=True)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--collision-meshes', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    arguments = parser.parse_args()
    generate_model(
        arguments.urdf,
        arguments.inputs,
        arguments.scene,
        arguments.collision_meshes,
        arguments.output,
    )


if __name__ == '__main__':
    main()
