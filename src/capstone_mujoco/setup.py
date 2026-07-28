import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'capstone_mujoco'


def install_tree(source_directory):
    """Return setuptools data-file entries for a recursive package tree."""
    entries = []
    for directory, _, filenames in os.walk(source_directory):
        files = [
            os.path.join(directory, filename)
            for filename in filenames
        ]
        if files:
            entries.append(
                (
                    os.path.join('share', package_name, directory),
                    files,
                )
            )
    return entries


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ] + install_tree('config') + install_tree('description')
    + install_tree('licenses'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryoo',
    maintainer_email='ryoo@todo.todo',
    description='MuJoCo ros2_control backend for the calibrated capstone UR3.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'generate_ur3_mjcf = capstone_mujoco.model_generation:main',
            'mujoco_ui_setup = capstone_mujoco.mujoco_ui_setup:main',
        ],
    },
)
