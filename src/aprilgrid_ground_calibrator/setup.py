import os
from glob import glob

from setuptools import setup

package_name = 'aprilgrid_ground_calibrator'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vaishnavr27',
    maintainer_email='vaishnavr27@gmail.com',
    description=(
        'Calibrates camera height above the ground and camera downward '
        'tilt angle using a planar AprilGrid target, via a single joint '
        'PnP pose estimate over all detected tag corners.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'calibrator_node = aprilgrid_ground_calibrator.calibrator_node:main',
        ],
    },
)
