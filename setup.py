from setuptools import setup
import os
from glob import glob

package_name = 'helios_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neo',
    maintainer_email='mael.godard@ensta-bretagne.org',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gnss_infos = helios_ros2.gnss_infos:main',
            'mission_publisher = helios_ros2.mission_manager:main',
            'boat_simulator = helios_ros2.boat_simulator:main',
            'line_follow = helios_ros2.line_follow:main',
            'command = helios_ros2.command:main',
            'motors = helios_ros2.comm_arduino:main',
        ],
    },
)
