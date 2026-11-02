"""
Mapping stack: lidar -> TF -> slam_toolbox.

Brings up every piece slam_toolbox needs in one shot, because starting them by
hand meant a container restart silently dropped whichever one you forgot:

    map  -> odom             slam_toolbox (this file)
    odom -> base_footprint   odom_tf_broadcaster (rover publishes Odometry
                             messages but never broadcasts TF)
    base_footprint -> laser  robot_state_publisher, from
                             description/open_rover.urdf.xacro
    /scan                    sllidar_node

Miss any link and slam_toolbox starts, drops every scan, and never publishes
/map -- with no error louder than silence. To check whether it is really
mapping, don't trust the log (the Karto "Registering sensor" line is stdout
buffered and often never appears under `ros2 launch`); check that the `map`
frame exists instead:

    ros2 run tf2_ros tf2_echo map odom

Usage:
    ros2 launch launch/mapping.py
    ros2 launch launch/mapping.py use_lidar:=false      # sllidar already running

The lidar mount is no longer a launch argument -- edit
description/open_rover.urdf.xacro, which localization.py reads too.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

HOME = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

CONFIG = os.path.join(HOME, "config", "mapper_params_online_async.yaml")

# Same robot description localization.py uses. A map MUST be built with the
# same lidar pose it is later localized in, so these two files cannot be
# allowed to drift apart -- which is exactly what happened while both
# carried their own hardcoded laser_x default.
XACRO = os.path.join(HOME, "description", "open_rover.urdf.xacro")


def generate_launch_description():
    use_lidar = LaunchConfiguration("use_lidar")

    args = [
        DeclareLaunchArgument("use_lidar", default_value="true",
                              description="Start sllidar_node (false if already running)"),
        # The laser_x/y/z/yaw arguments are gone -- the mount now comes
        # from the URDF below. Their defaults were placeholders labelled
        # UNVERIFIED, and the measured lidar offset is 0.127 m forward of
        # base_footprint, not the 0.2 m they claimed. Every map in maps/
        # was rastered through the old value; see the LIDAR MOUNT note in
        # localization.py for what that costs.
    ]

    nodes = [
        Node(
            package="sllidar_ros2",
            executable="sllidar_node",
            name="sllidar_node",
            condition=IfCondition(use_lidar),
            parameters=[{
                "serial_port": "/dev/ttyUSB0",
                "serial_baudrate": 115200,   # A1; the C1 and later use 460800
                "frame_id": "laser",
                "angle_compensate": True,
                "scan_mode": "Sensitivity",
            }],
        ),
        # base_footprint -> base_link -> laser, and every other frame,
        # from the measured URDF. Replaces the static transform that used
        # to publish base_footprint -> laser directly; the two cannot both
        # run without giving laser two parents.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": ParameterValue(
                    Command(["xacro ", XACRO]), value_type=str),
            }],
        ),
        Node(
            package="lidar_toolbox",
            executable="odom_tf_broadcaster",
            name="odom_tf_broadcaster",
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            parameters=[CONFIG],
        ),
    ]

    return LaunchDescription(args + nodes)
