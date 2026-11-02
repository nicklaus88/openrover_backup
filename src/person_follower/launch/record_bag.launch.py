"""Record a rosbag2 of everything needed to reconstruct/describe a
person-following run afterwards -- raw sensor data (forward camera +
camera_info, the two side cameras, scan, odom, tf) plus every stage of
the pipeline's own output (detections -> tracks -> target
state/imprint-candidate progress -> the robot controller's full decision
reasoning -> the velocity commanded AND the velocity the base reported
back). The idea being: enough
recorded here to later turn a run into a language description (what was
detected and where -- bounding boxes are in detections/tracks --, what
state the follower was in, why the robot did what it did at each moment,
how it actually moved) without needing to re-derive any of it from raw
video.

This wraps `ros2 bag record` (via ExecuteProcess) rather than being a
custom recording node -- rosbag2 already does exactly this job, reliably,
with no reason to reimplement it.

Included by default from person_follower.launch.py (record_bag:=false to
skip it there) -- can also be run standalone alongside it in a separate
terminal instead (record_bag:=false on the main launch, run this here).
Either way, recording starts immediately; topics that don't exist yet are
picked up automatically once they start publishing, no restart needed.

Usage:
  ros2 launch person_follower record_bag.launch.py
  ros2 launch person_follower record_bag.launch.py bag_dir:=~/bags record_debug_visuals:=true

Default bag_dir is inside the ROS2-WorkSpace bind mount (persists on the
host across container rebuilds/recreates) -- NOT under $HOME, which lives
only in the container's writable layer and is silently wiped whenever the
container gets recreated (this bit torch/ultralytics and the discovery
server config earlier; don't let it happen to recorded data too).

Topic defaults here are this rover's, matching person_follower.launch.py.
They only apply when this file is run STANDALONE: included from
person_follower.launch.py it is passed that launch's resolved topics
instead, so the two cannot drift apart.

Note the recorded RobotDecision now carries the strafe axis too
(base_vy/final_vy, and obstacle_direction reinterpreted as a strafe
bias) -- see RobotDecision.msg. Bags recorded from the differential-drive
robot predate those fields.
"""
import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    default_bag_dir = os.path.expanduser('~/rover_ws/bags')
    default_bag_name = 'run_' + datetime.now().strftime('%Y%m%d_%H%M%S')

    bag_dir = LaunchConfiguration('bag_dir')
    bag_name = LaunchConfiguration('bag_name')
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    tf_topic = LaunchConfiguration('tf_topic')
    tf_static_topic = LaunchConfiguration('tf_static_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    vel_feedback_topic = LaunchConfiguration('vel_feedback_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    left_image_topic = LaunchConfiguration('left_image_topic')
    right_image_topic = LaunchConfiguration('right_image_topic')
    record_debug_visuals = LaunchConfiguration('record_debug_visuals')

    declare_args = [
        DeclareLaunchArgument('bag_dir', default_value=default_bag_dir),
        DeclareLaunchArgument('bag_name', default_value=default_bag_name),
        # Same defaults as person_follower.launch.py -- the *compressed*
        # image topic, matching what the pipeline actually consumes.
        DeclareLaunchArgument(
            'image_topic', default_value='/oak/rgb/image_raw/compressed'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/oak/rgb/camera_info'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('tf_topic', default_value='/tf'),
        DeclareLaunchArgument('tf_static_topic', default_value='/tf_static'),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='/rover/base/velocity/set'),
        # BaseDriver.cpp's acknowledgment that it consumed a command
        # (base/velocity/get, same geometry_msgs/Twist). NOT a measured
        # velocity and NOT a modified one -- it republishes the Twist it
        # received, verbatim, from inside the 10 ms debounce timer. What
        # it adds over cmd_vel_topic is TIMING and DROPS: it is stamped
        # when the driver acted, and commands arriving faster than 10 ms
        # collapse into one, so /set without a matching /get is a command
        # the base never acted on. Clamping (motion() clamps its own
        # local copies) and halting (halt() skips the motors but /get
        # still publishes) are invisible here -- odom_topic is the only
        # record of those. set = asked for, get = accepted and when,
        # odom = what the wheels did.
        DeclareLaunchArgument(
            'vel_feedback_topic', default_value='/rover/base/velocity/get'),
        # The rover's own wheel/IMU odometry. Recorded for the same
        # reason the TurtleBot4's /tb4/odom was: it is the only record of
        # what the base actually did, as opposed to what it was told to
        # do. Worth more on this base, where a commanded strafe can be
        # refused outright by the sideways clearance guard.
        DeclareLaunchArgument(
            'odom_topic', default_value='/rover/base/odometry'),
        # The two BlueRobotics side cameras (side_cameras package). The
        # *compressed* topics, for the same reason the forward camera is
        # recorded compressed: these are 1280x720 at 30 Hz per side, so
        # raw would be ~2.7 MB per frame PAIR against ~150 KB JPEG. They
        # see what the forward 250x250 preview cannot -- a person leaving
        # frame sideways, and which side they left on.
        DeclareLaunchArgument(
            'left_image_topic',
            default_value='/capture/camera_left/img/compressed'),
        DeclareLaunchArgument(
            'right_image_topic',
            default_value='/capture/camera_right/img/compressed'),
        # There is no on/off argument for these: not running the
        # side_cameras package IS the off switch. Naming a topic nothing
        # publishes costs nothing -- rosbag2 records no messages on it,
        # and picks it up if it starts mid-run.
        # Off by default: derived from the other topics (annotated_image
        # is tracks+target re-rendered onto image_topic; markers are
        # tracks re-rendered as RViz markers) so they're redundant for a
        # "reconstruct what happened" bag and just add size. Turn on if
        # you specifically want the rendered debug view saved too.
        DeclareLaunchArgument('record_debug_visuals', default_value='false'),
    ]

    output_uri = PythonExpression(["'", bag_dir, "/'", " + '", bag_name, "'"])

    topics = [
        # raw sensor data
        image_topic,
        camera_info_topic,
        scan_topic,
        tf_topic,
        tf_static_topic,
        odom_topic,
        # The side views go in THIS bag, not a sibling one, so a run
        # stays a single `ros2 bag play`-able unit with everything
        # already on one clock -- no joining two bags by timestamp.
        left_image_topic,
        right_image_topic,
        # person_follower pipeline, in order: per-frame keypoint
        # detections (bounding boxes + keypoints) -> id-tracked positions
        # (bounding boxes again, now with a persistent id + 3D position)
        # -> the follower state machine's chosen target + imprint/re-id
        # decision progress -> the controller's full reasoning for
        # turning that into motion -> what actually got driven.
        '/pose_detector_node/detections',
        '/person_tracker_node/tracks',
        '/person_follower_node/target',
        '/robot_controller_node/decision',
        cmd_vel_topic,
        vel_feedback_topic,
    ]

    record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', output_uri, '-s', 'mcap'] + topics,
        output='screen',
    )

    record_debug = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o',
             PythonExpression(["'", output_uri, "' + '_debug_visuals'"]),
             '-s', 'mcap',
             '/visualization_node/annotated_image', '/visualization_node/markers'],
        output='screen',
        condition=IfCondition(record_debug_visuals),
    )

    return LaunchDescription(declare_args + [record, record_debug])
