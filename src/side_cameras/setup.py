import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'side_cameras'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robopi',
    maintainer_email='vaishnavr27@gmail.com',
    description=(
        'Left/right BlueRobotics USB camera bringup: one node per camera, '
        'publishing raw and JPEG-compressed images.'
    ),
    license='BSD',
    entry_points={
        'console_scripts': [
            'side_camera_node = side_cameras.side_camera_node:main',
        ],
    },
)
