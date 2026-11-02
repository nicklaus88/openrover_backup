import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'person_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robopi',
    maintainer_email='vaishnavr27@gmail.com',
    description=(
        'Monocular+depth person detection/tracking/following for the '
        'RoverMaster omnidirectional base, in the spirit of '
        'koide3/monocular_person_following.'
    ),
    license='BSD',
    entry_points={
        'console_scripts': [
            'pose_detector_node = person_follower.pose_detector_node:main',
            'person_tracker_node = person_follower.person_tracker_node:main',
            'person_follower_node = person_follower.person_follower_node:main',
            'robot_controller_node = person_follower.robot_controller_node:main',
            'goal_publisher_node = person_follower.goal_publisher_node:main',
            'visualization_node = person_follower.visualization_node:main',
        ],
    },
)
