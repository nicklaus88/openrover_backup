"""Robot bringup: every driver and the full TF tree. No localization.

    ros2 launch launch/robot_start.py

Starts the robot itself and nothing that reasons about where it is:

    rover base              /rover/base/odometry, /rover/base/imu,
                            velocity in/out (package rover, executable base)
    teleop bridge           camera_detection/operation: drive commands from
                            /tmp/omni-control.sock onto the base
    OAK-D Lite              /oak/rgb/image_raw, /oak/stereo/... (depthai)
    side cameras            /capture/camera_left/img, /capture/camera_right/img
    RPLIDAR A1              /scan
    robot_state_publisher   base_footprint -> base_link -> laser, imu_link,
                            wheels, side cameras ... from
                            description/open_rover.urdf.xacro
    camera_mount_tf         base_footprint -> oak-d-base-frame, which roots
                            the depthai driver's own camera frames

Localization is a separate file, run on top of this one:

    ros2 launch launch/localization.py

That split is deliberate. Only one node may hold each serial port (lidar,
flight controller) and only one may publish each TF edge; with every
driver and every frame owned here, localization.py and anything else
you add can come and go without duplicating them.

The drivers themselves come from launch/perception_oakd.py, included
rather than copied, so there is still one definition of each. Its
switches are all available here, e.g.:

    ros2 launch launch/robot_start.py use_side_cameras:=false
    ros2 launch launch/robot_start.py use_lidar:=false
    ros2 launch launch/robot_start.py use_base:=false       # base already up

CAMERA MOUNT
------------
Nothing else in the bringup attaches the depthai frame tree to the robot:
the driver roots it at oak-d-base-frame and stops. Without this link,
anything looking up the camera in base_footprint fails -- person_follower
detects people but never produces a track. The values default to the
2026-08-25 ground calibration (the same defaults person_follower.launch.py
carries; see its comments for the derivation). camera_x and camera_y are
NOT calibrated -- measure them.

This file owns that edge now, so start person_follower with

    ros2 launch person_follower person_follower.launch.py use_camera_tf:=false

or the edge has two publishers. With identical values that is harmless;
the moment one side is retuned, tf2 alternates between them.

Do not run launch/mapping.py alongside this: it starts its own lidar and
robot_state_publisher.

Checks once it is up:

    ros2 topic hz /scan                     # ~7.5 Hz
    ros2 topic hz /rover/base/odometry      # ~50 Hz
    ros2 run tf2_ros tf2_echo base_footprint laser
    ros2 run tf2_ros tf2_echo base_footprint oak-d-base-frame
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))

# Base, teleop, OAK-D, side cameras, lidar and robot_state_publisher.
# Every one of its launch arguments passes straight through from the
# command line, since an included file shares the caller's configuration.
DRIVERS = os.path.join(HERE, "perception_oakd.py")


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "use_camera_tf", default_value="true",
            description="Publish base_footprint -> oak-d-base-frame. Set "
                        "false if something else already does"),
        DeclareLaunchArgument(
            "camera_x", default_value="0.0",
            description="Camera mount, metres forward of base_footprint "
                        "-- NOT calibrated, measure it"),
        DeclareLaunchArgument(
            "camera_y", default_value="0.0",
            description="Camera mount, metres left of base_footprint "
                        "-- NOT calibrated, measure it"),
        DeclareLaunchArgument(
            "camera_z", default_value="0.272065",
            description="Camera mount height, metres (calibrated 2026-08-25)"),
        DeclareLaunchArgument(
            "camera_yaw", default_value="0.0",
            description="Camera mount yaw, radians (0 = facing forward)"),
        DeclareLaunchArgument(
            "camera_pitch", default_value="-0.125906",
            description="Camera mount pitch, radians, REP-103 sign "
                        "(this mount is nose UP 7.21 deg)"),
        DeclareLaunchArgument(
            "camera_roll", default_value="-0.030283",
            description="Camera mount roll, radians (calibrated 2026-08-25)"),
    ]

    drivers = IncludeLaunchDescription(PythonLaunchDescriptionSource(DRIVERS))

    camera_mount_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_mount_tf",
        condition=IfCondition(LaunchConfiguration("use_camera_tf")),
        # Positional form: x y z yaw pitch roll parent child.
        arguments=[
            LaunchConfiguration("camera_x"),
            LaunchConfiguration("camera_y"),
            LaunchConfiguration("camera_z"),
            LaunchConfiguration("camera_yaw"),
            LaunchConfiguration("camera_pitch"),
            LaunchConfiguration("camera_roll"),
            "base_footprint", "oak-d-base-frame",
        ],
    )

    return LaunchDescription(args + [drivers, camera_mount_tf])
