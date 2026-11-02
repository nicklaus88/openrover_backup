"""Robot bringup: every driver and the full TF tree. No odometry fusion, no map.

    ros2 launch rover_bringup robot.launch.py
    ros2 launch rover_bringup robot.launch.py use_side_cameras:=false

Starts:
    rover base             /rover/base/odometry, /rover/base/imu,
                           /rover/base/velocity/set (Betaflight board, MSP)
    OAK-D Lite             /oak/rgb/image_raw, /oak/stereo/... (depthai)
    side cameras           /capture/camera_left/img, /capture/camera_right/img
    RPLIDAR A1             /scan (~7.5 Hz)
    robot_state_publisher  base_footprint -> laser, imu_link, wheels, ...
                           from description/open_rover.urdf.xacro
    camera_mount_tf        base_footprint -> oak-d-base-frame

This file owns every serial port and every robot TF edge. Run
localization.launch.py OR mapping.launch.py on top of it, never both.

Checks:
    ros2 topic hz /scan                     # ~7.5 Hz
    ros2 topic hz /rover/base/odometry      # ~50 Hz
    ros2 run tf2_ros tf2_echo base_footprint oak-d-base-frame
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SHARE = get_package_share_directory("rover_bringup")
XACRO = os.path.join(SHARE, "description", "open_rover.urdf.xacro")
OAKD_PARAMS = os.path.join(SHARE, "config", "oakd.yaml")
SIDE_PARAMS = os.path.join(
    get_package_share_directory("side_cameras"), "config", "side_cameras.yaml")
DEPTHAI_LAUNCH = os.path.join(
    get_package_share_directory("depthai_ros_driver"), "launch", "camera.launch.py")

# OAK-D mount relative to base_footprint (x y z yaw pitch roll; m, rad).
# z, pitch, roll: ground calibration of 2026-08-25 (nose up 7.21 deg).
# x, y: NOT calibrated yet -- measure and fill in.
CAMERA_MOUNT = ["0.0", "0.0", "0.272065", "0.0", "-0.125906", "-0.030283"]

# /dev/ttyUSB0 is the lidar's CP2102 bridge; the base is /dev/ttyACM0.
LIDAR_PORT = "/dev/ttyUSB0"


def generate_launch_description():
    use_side_cameras = IfCondition(LaunchConfiguration("use_side_cameras"))

    oakd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(DEPTHAI_LAUNCH),
        launch_arguments={
            "name": "oak",
            "params_file": OAKD_PARAMS,
            "camera_model": "OAK-D-LITE",
            "parent_frame": "oak-d-base-frame",
            "rectify_rgb": "true",
        }.items(),
    )

    base = Node(
        package="rover",
        executable="base",
        namespace="rover",
        parameters=[{"vid": "x0483", "pid": "x5740"}],
    )

    side_cameras = [
        Node(
            package="side_cameras",
            executable="side_camera_node",
            name=f"{side}_camera",  # must match the key in side_cameras.yaml
            output="screen",
            parameters=[SIDE_PARAMS],
            condition=use_side_cameras,
        )
        for side in ("left", "right")
    ]

    lidar = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        output="screen",
        parameters=[{
            "channel_type": "serial",
            "serial_port": LIDAR_PORT,
            "serial_baudrate": 115200,  # A1; C1 and later use 460800
            "frame_id": "laser",
            "inverted": False,
            "angle_compensate": True,
            "scan_mode": "Sensitivity",
        }],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(
                Command(["xacro ", XACRO]), value_type=str),
        }],
    )

    camera_mount_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_mount_tf",
        arguments=CAMERA_MOUNT + ["base_footprint", "oak-d-base-frame"],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_side_cameras", default_value="true",
            description="Start the left/right BlueRobotics cameras"),
        oakd,
        base,
        *side_cameras,
        lidar,
        robot_state_publisher,
        camera_mount_tf,
    ])
