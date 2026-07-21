import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'ur3_control'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
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
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryoo',
    maintainer_email='ryoo@todo.todo',
    description='Keyboard fixed-pose controller for a real UR3.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ur3_pose_control = ur3_control.ur3_pose_control:main',
        ],
    },
)
