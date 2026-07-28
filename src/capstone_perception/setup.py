import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'capstone_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryoo',
    maintainer_email='ryoo@todo.todo',
    description='Compressed image viewer for the verified RealSense D435i stream.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'compressed_camera_viewer = '
            'capstone_perception.compressed_camera_viewer_node:main',
        ],
    },
)
