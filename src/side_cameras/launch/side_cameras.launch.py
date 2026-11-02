"""Both side cameras on their own -- no perception, no lidar.

    ros2 launch side_cameras side_cameras.launch.py

Handy for checking the left/right assignment or re-aiming the mounts
without starting the whole stack. perception_oakd.py starts the same two
nodes as part of the full bringup.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

CONFIG = os.path.join(
    get_package_share_directory('side_cameras'), 'config', 'side_cameras.yaml'
)


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='side_cameras',
            executable='side_camera_node',
            name=name,
            output='screen',
            parameters=[CONFIG],
        )
        for name in ('left_camera', 'right_camera')
    ])
