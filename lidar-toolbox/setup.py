from setuptools import setup
import os
from glob import glob
from setuptools import find_packages

package_name = 'lidar_toolbox'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adnana',
    maintainer_email='adnanabdullah@ufl.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'proximity = lidar_toolbox.proximity:main',
            'scan_transformer = lidar_toolbox.scan_transformer:main',
            'scan_player = lidar_toolbox.scan_player:main',
            'odom_tf_broadcaster = lidar_toolbox.odom_tf_broadcaster:main',
            'odom_calibrate = lidar_toolbox.odom_calibrate:main',
        ],
    },
)
