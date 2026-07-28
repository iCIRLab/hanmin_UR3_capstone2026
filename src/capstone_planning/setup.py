import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'capstone_planning'

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
            os.path.join('share', package_name, 'description'),
            glob('description/*'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='ryoo',
    maintainer_email='ryoo@todo.todo',
    description='Calibrated UR3 model and MPlib planning integration.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mplib_plan = capstone_planning.mplib_plan_node:main',
            'moveit_plan = capstone_planning.moveit_plan_node:main',
            'interactive_plan = '
            'capstone_planning.interactive_plan_node:main',
        ],
    },
)
