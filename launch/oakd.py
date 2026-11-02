"""OAK-D Lite camera only: the depthai driver plus the RGB relay.

Publishes the rover's camera contract:

    /capture/camera_0/img              sensor_msgs/Image
    /capture/camera_0/img/compressed   sensor_msgs/CompressedImage

That first topic is what launch/perception.py already remaps
camera_detection/perception's "img" input to, so this is a drop-in
replacement for the spinnaker_camera "capture" node (whose submodule is
empty in this checkout).

    ros2 launch launch/oakd.py

Use launch/perception_oakd.py instead if you want the whole perception
pipeline, not just the camera.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

PARAMS = os.path.join(
    get_package_share_directory("oakd_camera"), "config", "oakd_lite.yaml"
)

DEPTHAI_LAUNCH = os.path.join(
    get_package_share_directory("depthai_ros_driver"), "launch", "camera.launch.py"
)

# The depthai driver is a C++ composable node -- it has to come from its
# own launch file, it is not a plain Node() we can list here.
driver = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(DEPTHAI_LAUNCH),
    launch_arguments={
        "name": "oak",
        "params_file": PARAMS,
        "camera_model": "OAK-D-LITE",
        "parent_frame": "oak-d-base-frame",
        "rectify_rgb": "true",
    }.items(),
)

nodes = [
    Node(
        package="oakd_camera",
        executable="oakd_rgb_publisher_node",
        name="oakd_rgb_publisher_node",
        output="screen",
        parameters=[PARAMS],
    ),
]


def generate_launch_description():
    return LaunchDescription([driver] + nodes)
