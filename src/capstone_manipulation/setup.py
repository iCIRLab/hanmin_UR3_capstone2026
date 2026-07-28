from setuptools import find_packages, setup

package_name = 'capstone_manipulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryoo',
    maintainer_email='ryoo@todo.todo',
    description='Keyboard Home and scan-pose controller for the real UR3.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'trajectory_executor = '
            'capstone_manipulation.trajectory_executor:main',
            'ur3_pose_control = capstone_manipulation.ur3_pose_control:main',
        ],
    },
)
