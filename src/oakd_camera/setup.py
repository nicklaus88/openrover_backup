import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'oakd_camera'

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
        'OAK-D Lite bringup: depthai_ros_driver plus an RGB relay that '
        'publishes raw and JPEG-compressed images on two topics.'
    ),
    license='BSD',
    entry_points={
        'console_scripts': [
            'oakd_rgb_publisher_node = '
            'oakd_camera.oakd_rgb_publisher_node:main',
        ],
    },
)
