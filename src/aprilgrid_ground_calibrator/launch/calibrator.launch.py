"""Launch file for aprilgrid_ground_calibrator.

Loads config/calibrator.yaml by default. Override with:

    ros2 launch aprilgrid_ground_calibrator calibrator.launch.py \
        config_file:=/path/to/my_calibrator.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('aprilgrid_ground_calibrator')
    default_config = os.path.join(pkg_share, 'config', 'calibrator.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the calibrator YAML parameter file.',
    )

    calibrator_node = Node(
        package='aprilgrid_ground_calibrator',
        executable='calibrator_node',
        name='aprilgrid_ground_calibrator',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([config_file_arg, calibrator_node])
