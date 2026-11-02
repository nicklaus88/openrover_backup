"""Bring up the OAK-D Lite and fan its RGB stream onto two topics.

Starts two things:

  1. depthai_ros_driver's camera_node, configured by config/oakd_lite.yaml
     (RGBD pipeline, no NN, no IMU/IR -- the Lite has neither).
  2. oakd_rgb_publisher_node, which republishes the driver's colour frames
     as a raw sensor_msgs/Image and a JPEG sensor_msgs/CompressedImage.

    ros2 launch oakd_camera oakd_lite.launch.py

Set start_driver:=false to attach the relay to a camera that some other
bringup (e.g. person_follower's bringup_with_camera.launch.py) already
started.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    depthai_share = get_package_share_directory('depthai_ros_driver')
    oakd_share = get_package_share_directory('oakd_camera')

    default_params = os.path.join(oakd_share, 'config', 'oakd_lite.yaml')

    name = LaunchConfiguration('name')
    params_file = LaunchConfiguration('params_file')
    start_driver = LaunchConfiguration('start_driver')

    declare_args = [
        DeclareLaunchArgument(
            'name', default_value='oak',
            description='Driver node name; also the topic prefix (/oak/...).'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='YAML with both the driver and relay parameters.'),
        DeclareLaunchArgument(
            'start_driver', default_value='true',
            description='Set false to only run the relay node.'),
        DeclareLaunchArgument(
            'parent_frame', default_value='oak-d-base-frame',
            description='TF frame the camera is published relative to.'),
    ]

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [depthai_share, '/launch/camera.launch.py']),
        condition=IfCondition(start_driver),
        launch_arguments={
            'name': name,
            'params_file': params_file,
            'parent_frame': LaunchConfiguration('parent_frame'),
            # Drives which mesh/extrinsics depthai_descriptions loads. The
            # driver's own default is OAK-D-PRO, which is the wrong body
            # and the wrong baseline for this camera.
            'camera_model': 'OAK-D-LITE',
            'rectify_rgb': 'true',
        }.items(),
    )

    relay = Node(
        package='oakd_camera',
        executable='oakd_rgb_publisher_node',
        name='oakd_rgb_publisher_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription(declare_args + [driver, relay])
