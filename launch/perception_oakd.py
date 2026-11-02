"""Rover sensor bringup, sourced from the OAK-D Lite instead of Spinnaker.

Started life as launch/perception.py with the spinnaker_camera "capture"
node swapped for the depthai driver. It no longer runs any detection:
the camera_detection/perception YOLO node has been removed, so what is
left is the base driver, every sensor, and the teleop bridge. Publishing
the streams is the whole job -- consumers (person_follower, rviz2, a
detector you start yourself) subscribe to them.

perception.py's own node list is left untouched so the original still
works if the Spinnaker submodule is ever checked out.

The oakd_camera relay that used to bridge the driver onto
/capture/camera_0/img is still here but off by default -- see
use_rgb_relay below.

One command gives you every eye the rover has:

    /oak/rgb/image_raw           forward   OAK-D Lite   (depthai driver)
    /capture/camera_left/img     left      BlueRobotics (side_cameras)
    /capture/camera_right/img    right     BlueRobotics (side_cameras)
    /scan                        RPLIDAR A1              (sllidar_ros2)

Each of the three is behind its own switch, because they overlap with
other launch files -- launch/mapping.py starts its own sllidar_node, and
running two against one serial port leaves both failing to connect:

    ros2 launch launch/perception_oakd.py
    ros2 launch launch/perception_oakd.py use_lidar:=false          # mapping.py has it
    ros2 launch launch/perception_oakd.py use_side_cameras:=false   # forward camera only

If the two side views come out swapped, override the device paths (see
config/side_cameras.yaml for why they are by-path symlinks):

    ros2 launch launch/perception_oakd.py \
        left_camera_device:=/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1.3:1.0-video-index0 \
        right_camera_device:=/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1.1:1.0-video-index0
"""

import os
from pathlib import Path
from shutil import rmtree

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# resolve(), not a bare parent chain: `ros2 launch launch/perception_oakd.py`
# sets __file__ to the RELATIVE path typed, and XACRO below is handed to a
# subprocess. Same reasoning as localization.py's abspath().
HOME = Path(__file__).resolve().parent.parent
VAR = HOME / "var"

# The robot's frames, measured. Shared with localization.py and mapping.py
# so all three publish an identical tree -- see laser_tf below.
XACRO = str(HOME / "description" / "open_rover.urdf.xacro")

# perception.py does a bare rmtree(VAR) here, which raises if var/ is
# missing (e.g. a fresh clone). Same intent, without that failure mode.
rmtree(VAR, ignore_errors=True)
VAR.mkdir(parents=True, exist_ok=True)

PARAMS = os.path.join(
    get_package_share_directory("oakd_camera"), "config", "oakd_lite.yaml"
)

SIDE_PARAMS = os.path.join(
    get_package_share_directory("side_cameras"), "config", "side_cameras.yaml"
)

DEPTHAI_LAUNCH = os.path.join(
    get_package_share_directory("depthai_ros_driver"), "launch", "camera.launch.py"
)

camera = IncludeLaunchDescription(
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
    # use_base:=false when rover.py or perception.py already owns the
    # base: two base nodes fight over the flight controller's serial port.
    # Teleop goes with it, since it only drives that base.
    Node(
        package="rover",
        executable="base",
        namespace="rover",
        condition=IfCondition(LaunchConfiguration("use_base")),
        parameters=[
            {"vid": "x0483"},
            {"pid": "x5740"},
        ],
    ),
    # Replaces: Node(package="spinnaker_camera", executable="capture",
    #                namespace="capture")
    #
    # OFF by default now. perception below reads the depthai driver's
    # /oak/rgb/image_raw directly, so this relay's only remaining job is
    # the /capture/... compatibility pair -- and it is not free: it is a
    # Python node doing a cv_bridge convert plus a JPEG encode on every
    # frame at i_fps (30), which is exactly the kind of hop that shows up
    # as camera lag on the Jetson. Turn it back on if something still
    # wants /capture/camera_0/img:
    #
    #     ros2 launch launch/perception_oakd.py use_rgb_relay:=true
    Node(
        package="oakd_camera",
        executable="oakd_rgb_publisher_node",
        name="oakd_rgb_publisher_node",
        output="screen",
        parameters=[PARAMS, {"input_topic": LaunchConfiguration("camera_topic")}],
        condition=IfCondition(LaunchConfiguration("use_rgb_relay")),
    ),
    # camera_detection/perception -- the YOLO node -- used to sit here,
    # subscribing to the camera and shipping yolo11n.pt detections out
    # over /tmp/omni-control.sock. Removed deliberately: nothing in this
    # file consumes the camera any more, it is pure sensor bringup.
    #
    # Note "operation" below does NOT depend on it. Both were independent
    # clients of that same socket -- perception wrote detections to it,
    # operation reads drive commands from it -- so teleop is unaffected.
    #
    # To run detection again, run it on its own against whichever image
    # topic you want:
    #
    #     ros2 run camera_detection perception --ros-args \
    #         -r img:=/oak/rgb/image_raw
    #
    # (it must be started from ROS2-WorkSpace: it resolves assets/ and
    # var/ relative to the process working directory.)
    Node(
        package="camera_detection",
        executable="operation",
        condition=IfCondition(LaunchConfiguration("use_base")),
    ),
]


def side_cameras(context, *_args, **_kwargs):
    """The two BlueRobotics side cameras, one node each.

    Built in an OpaqueFunction rather than listed above so the device
    arguments can stay *overrides*: everything comes from
    side_cameras.yaml, and a device path is only injected on top when the
    caller actually passed one. Declaring the by-path defaults here
    instead would mean editing `device:` in the yaml had no effect when
    launched from this file -- the launch value would always win.
    """
    enabled = IfCondition(LaunchConfiguration("use_side_cameras"))
    out = []
    for side in ("left", "right"):
        params = [SIDE_PARAMS]
        device = LaunchConfiguration(f"{side}_camera_device").perform(context)
        if device:
            params.append({"device": device})
        out.append(
            Node(
                package="side_cameras",
                executable="side_camera_node",
                # Has to match the key in side_cameras.yaml.
                name=f"{side}_camera",
                output="screen",
                parameters=params,
                condition=enabled,
            )
        )
    return out


# The RPLIDAR A1. Same parameters as launch/mapping.py -- keep the two in
# step if you retune one. Note the A1 runs at 115200; the C1 and later
# models use 460800 and will just time out at this baudrate.
lidar = Node(
    package="sllidar_ros2",
    executable="sllidar_node",
    name="sllidar_node",
    output="screen",
    condition=IfCondition(LaunchConfiguration("use_lidar")),
    parameters=[{
        "channel_type": "serial",
        "serial_port": LaunchConfiguration("lidar_serial_port"),
        "serial_baudrate": LaunchConfiguration("lidar_serial_baudrate"),
        "frame_id": LaunchConfiguration("lidar_frame_id"),
        "inverted": False,
        "angle_compensate": True,
        "scan_mode": "Sensitivity",
    }],
)

# base_footprint -> laser. The lidar above stamps its scans with
# lidar_frame_id, and that frame is only usable if something publishes a
# transform for it: robot_controller_node does one
# lookup_transform(base_footprint, scan.header.frame_id) to learn the
# lidar's yaw mount offset, and without this it logs
#
#     lidar tf lookup (laser -> base_footprint) failed, skipping
#     obstacle check this frame
#
# on repeat -- meaning obstacle avoidance is off while the rover drives.
#
# This used to be a static_transform_publisher parenting laser directly to
# base_footprint, with laser_x/y/z/yaw arguments defaulting to 0.2 m
# forward. localization.py and mapping.py had already moved to the URDF,
# which reaches laser via base_footprint -> base_link -> laser at the
# measured 0.127 m. Running this file next to either of them therefore gave
# laser TWO parents: tf2 flips between them on every update, AMCL and rf2o
# see the lidar jump ~73 mm, and on odom_source:=fused the EKF fuses that
# jitter as motion.
#
# Now it publishes the same URDF, so the trees are identical. Two
# robot_state_publishers with identical content are harmless, but it is
# still one redundant node -- pass use_laser_tf:=false alongside
# mapping.py. (localization.py no longer publishes any frames; it relies
# on this one, via robot_start.py.) The distinct node name keeps the two
# from colliding if you forget.
#
# lidar_frame_id must stay "laser" while this is on: that is the frame
# the URDF defines, and any other name leaves the scan with no transform.
laser_tf = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="robot_state_publisher_perception",
    output="screen",
    condition=IfCondition(LaunchConfiguration("use_laser_tf")),
    parameters=[{
        "robot_description": ParameterValue(
            Command(["xacro ", XACRO]), value_type=str),
    }],
)

args = [
    DeclareLaunchArgument(
        # What the relay below republishes. The 256x256
        # /oak/rgb/preview/image_raw would be cheaper, but its camera_info
        # is wrong and it is disabled in oakd_lite.yaml -- read the note
        # there before switching to it.
        "camera_topic", default_value="/oak/rgb/image_raw",
        description="RGB stream the relay republishes"),
    DeclareLaunchArgument(
        "use_base", default_value="true",
        description="Start the rover base node and teleop bridge. Set "
                    "false when rover.py / perception.py already has it"),
    DeclareLaunchArgument(
        "use_laser_tf", default_value="true",
        description="Publish the robot's frames (incl. base_footprint -> "
                    "laser) from description/open_rover.urdf.xacro. Set "
                    "false when mapping.py is running -- it publishes the "
                    "same tree"),
    # laser_x/y/z/yaw are gone: the mount comes from the URDF now. See
    # laser_tf above.
    DeclareLaunchArgument(
        "use_rgb_relay", default_value="false",
        description="Also republish the RGB stream as /capture/camera_0/img "
                    "(+ /compressed) -- only needed by consumers still on "
                    "the old capture topic names"),
    DeclareLaunchArgument(
        "use_side_cameras", default_value="true",
        description="Start the left/right BlueRobotics cameras"),
    DeclareLaunchArgument(
        "left_camera_device", default_value="",
        description="Override the left camera's device path "
                    "(empty = use config/side_cameras.yaml)"),
    DeclareLaunchArgument(
        "right_camera_device", default_value="",
        description="Override the right camera's device path "
                    "(empty = use config/side_cameras.yaml)"),
    DeclareLaunchArgument(
        "use_lidar", default_value="true",
        description="Start sllidar_node (false if mapping.py already has it)"),
    DeclareLaunchArgument(
        # /dev/ttyUSB0 is the lidar's CP2102 bridge. It is the only ttyUSB
        # on this robot (the flight controller is ttyACM0), so the number
        # is stable in practice; if that ever changes, the by-id path
        # /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
        # names the same device unambiguously.
        "lidar_serial_port", default_value="/dev/ttyUSB0",
        description="Serial port the RPLIDAR A1 is on"),
    DeclareLaunchArgument(
        "lidar_serial_baudrate", default_value="115200",
        description="A1 = 115200; C1 and later = 460800"),
    DeclareLaunchArgument(
        "lidar_frame_id", default_value="laser",
        description="TF frame the scan is published in"),
]


def generate_launch_description():
    return LaunchDescription(
        args + [camera] + nodes
        + [OpaqueFunction(function=side_cameras), lidar, laser_tf]
    )
