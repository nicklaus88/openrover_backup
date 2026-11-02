"""Build a new map: fused odometry + slam_toolbox.

    ros2 launch rover_bringup robot.launch.py        # first
    ros2 launch rover_bringup mapping.launch.py      # drive the space slowly

Save the map into the package's maps folder (rebuild to install it):
    ros2 run nav2_map_server map_saver_cli -f ~/rover_ws/src/rover_bringup/maps/<name>

Never run alongside localization.launch.py -- both publish map -> odom.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

SHARE = get_package_share_directory("rover_bringup")
SLAM_LAUNCH = os.path.join(
    get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(SHARE, "launch", "odometry.launch.py"))),
        # slam_toolbox is a lifecycle node on Jazzy; its own launch file
        # configures and activates it. use_sim_time defaults to TRUE there.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(SLAM_LAUNCH),
            launch_arguments={
                "slam_params_file": os.path.join(
                    SHARE, "config", "mapper_params_online_async.yaml"),
                "use_sim_time": "false",
            }.items(),
        ),
    ])
