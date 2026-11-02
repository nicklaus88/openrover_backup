"""Fused odometry: owns the odom -> base_footprint transform.

Included by localization.launch.py and mapping.launch.py; needs
robot.launch.py running (for /scan, /rover/base/odometry and the
base_footprint -> laser transform).

    /scan --scan_filter--> /scan_rf2o --rf2o--> /odom_rf2o --+
                                                              +--> EKF --> odom -> base_footprint
                               /rover/base/odometry (twist) --+       /odometry/filtered

rf2o (lidar scan matching) catches wheel slip; the base's commanded
velocity carries the estimate through featureless corridors where scan
matching stalls. The scan filter drops returns off the robot's own arm.
rf2o's own TF is off -- the EKF is the only publisher of that edge.
See config/ekf_fused_odom.yaml and docs/localization.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

CONFIG = os.path.join(get_package_share_directory("rover_bringup"), "config")


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            name="rf2o_scan_filter",
            output="screen",
            parameters=[os.path.join(CONFIG, "scan_filter.yaml")],
            remappings=[("scan", "/scan"), ("scan_filtered", "/scan_rf2o")],
        ),
        Node(
            package="rf2o_laser_odometry",
            executable="rf2o_laser_odometry_node",
            name="rf2o_laser_odometry",
            output="screen",
            parameters=[{
                "laser_scan_topic": "/scan_rf2o",
                "odom_topic": "/odom_rf2o",
                "publish_tf": False,
                "base_frame_id": "base_footprint",
                "odom_frame_id": "odom",
                "init_pose_from_topic": "",
                "freq": 7.0,  # at or below the lidar's scan rate
            }],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[os.path.join(CONFIG, "ekf_fused_odom.yaml")],
        ),
    ])
