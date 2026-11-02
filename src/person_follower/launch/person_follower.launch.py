"""Bring up the full person_follower pipeline on the RoverMaster rover.

  camera (external) --image/depth/camera_info-->
    pose_detector_node --detections--> person_tracker_node --tracks-->
      person_follower_node --target--> robot_controller_node --> base velocity
                                   |--> goal_publisher_node --> PoseStamped goal (opt-in)
                                   |--> visualization_node --> annotated image / rviz markers
                                   \\--> record_bag.launch.py --> rosbag2 (opt-out, on by default)

PORTED FROM THE TURTLEBOT4. Every default below now points at this
rover's topics and frames rather than the /tb4/... namespace:

    image / camera_info   /oak/rgb/...          depthai_ros_driver (name:=oak)
    depth                 /oak/stereo/image_raw aligned to rgb
    scan                  /scan                 RPLIDAR A1, 360deg
    velocity command      /rover/base/velocity/set   rover/BaseDriver.cpp
    tf                    /tf, /tf_static       global, not namespaced
    base frame            base_footprint        there is NO base_link here

The controller was also rewritten for an omnidirectional base -- it
strafes rather than turning to correct lateral error, and its velocity
outputs are NORMALIZED THROTTLE in [-1, 1] rather than m/s and rad/s.
See robot_controller_node.py's module docstring and the UNITS banner in
config/person_follower.yaml.

CAMERA MOUNT TRANSFORM. This launch publishes base_footprint ->
oak-d-base-frame itself (camera_mount_tf below), because nothing else in
the workspace does: launch/perception_oakd.py starts the depthai driver
with parent_frame:=oak-d-base-frame, which roots the camera's own frame
tree but never attaches it to the robot. Without that link
person_tracker_node's tf2 lookup fails and the pipeline produces
detections but never a single track. The offsets are launch arguments --
MEASURE THEM ON THE ROBOT, the defaults are a guess. See camera_x/y/z
and camera_yaw/pitch/roll below.

That transform is separate from, and does not substitute for, the
camera_height / camera_pitch_deg calibration in
config/person_follower.yaml: this one places the camera in the tf tree,
those two define the ground plane that ground_plane mode ray-casts onto.
Both need to be right, and only the second can be solved optically (see
aprilgrid_ground_calibrator).

record_bag (default true) is included directly rather than run separately
-- `ros2 launch` sends every child process the same shutdown signal
together, so Ctrl+C-ing this launch stops the recording at the same
moment it stops the pipeline, no separate step needed. Pass
record_bag:=false to skip it (e.g. quick manual testing where you don't
want a bag written every run). It's fed the *actual* resolved topics
below, not a second independent copy of the defaults, so it can't
silently drift out of sync with what the pipeline is really doing.

robot_controller_node and goal_publisher_node are alternatives, not both
at once -- see use_goal_publisher below. robot_controller_node drives the
base directly (with its own lidar obstacle avoidance);
goal_publisher_node instead publishes where to go as a PoseStamped, for a
separate local planner to actually execute.

enable_driving:=false runs NEITHER of those -- just detection, tracking,
the target-selection state machine, and visualization. Nothing ever
publishes a velocity or a goal, so the robot physically cannot move. Use
this for the first run on this rover, and for tuning detection or
calibration, without any risk. (Rest of the pipeline, including
recording, still runs normally.)

State machine (see person_follower_node.py / config/person_follower.yaml
for the full explanation):
  SEARCH -> TRACKING -> (on track loss) LOST -> REACQUIRE -> TRACKING
                                                      \\-> (timeout) SEARCH

image_topic should be the *raw* image_transport base topic (e.g.
"/oak/rgb/image_raw", the default), not the "/compressed" one -- when
use_compressed_image is true (the default), the pipeline remaps to
"<image_topic>/compressed" and decodes sensor_msgs/CompressedImage
itself. Depth is never compressed (see pose_detector_node.py).

camera_info_topic must describe the SAME stream as image_topic. Mixing a
preview stream's images with the full stream's intrinsics (or the
reverse) silently produces wildly wrong 3D positions -- this caused a
real bug on the TurtleBot4: ~24m distance estimates for someone standing
a couple of meters away.
"""
import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('person_follower')
    default_params_file = os.path.join(pkg_share, 'config', 'person_follower.yaml')
    # Inside the ros2_ws bind mount so recorded bags persist on the host
    # across container rebuilds/recreates -- NOT under $HOME, which lives
    # only in the container's writable layer (see record_bag.launch.py).
    # ROS2-WorkSpace is bind-mounted at this path (see compose.yaml), so
    # bags persist on the host across container rebuilds -- NOT under
    # $HOME, which lives only in the container's writable layer.
    default_bag_dir = '/workspace/ROS2-WorkSpace/bags'
    default_bag_name = 'run_' + datetime.now().strftime('%Y%m%d_%H%M%S')

    image_topic = LaunchConfiguration('image_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    enable_driving = LaunchConfiguration('enable_driving')
    use_goal_publisher = LaunchConfiguration('use_goal_publisher')
    goal_topic = LaunchConfiguration('goal_topic')
    tf_topic = LaunchConfiguration('tf_topic')
    tf_static_topic = LaunchConfiguration('tf_static_topic')
    use_compressed_image = LaunchConfiguration('use_compressed_image')
    params_file = LaunchConfiguration('params_file')
    use_visualization = LaunchConfiguration('use_visualization')
    record_bag = LaunchConfiguration('record_bag')
    bag_dir = LaunchConfiguration('bag_dir')
    bag_name = LaunchConfiguration('bag_name')
    record_debug_visuals = LaunchConfiguration('record_debug_visuals')

    declare_args = [
        # The rover's OAK-D Lite via depthai_ros_driver (name:=oak), on
        # the full-size 640x360 ISP output.
        #
        # DO NOT move this to /oak/rgb/preview/image_raw without reading
        # the note in oakd_camera/config/oakd_lite.yaml first. The 256x256
        # preview is tempting -- a third of the bytes, resized on-device --
        # but depthai publishes BROKEN intrinsics for it when the preview
        # is not aspect-ratio-preserving: measured 2026-08-26, it reports
        # fx=fy=201.32 for a stream whose true fy is 357.9, because it
        # scales both by the width ratio. Ground-plane ranging divides by
        # fy, so every distance came out 1.78x short, everything read as
        # inside `distance` (1.6 m), and the rover drove backwards.
        DeclareLaunchArgument(
            'image_topic', default_value='/oak/rgb/preview/image_raw'),
        # Aligned to the rgb frame (i_align_depth: true in
        # oakd_lite.yaml). Only used when pose_detector_node's
        # position_source is "depth"; ignored in "ground_plane" mode.
        DeclareLaunchArgument('depth_topic', default_value='/oak/stereo/image_raw'),
        # Must describe the SAME stream as image_topic -- change the two
        # as a pair. Mixing a preview stream's images with the full
        # stream's intrinsics (or the reverse) silently produces wildly
        # wrong 3D positions: it caused a real bug once, ~24m distance
        # estimates for someone standing a couple of metres away.
        #
        # pose_detector_node's _intrinsics() rescales K by the actual
        # frame size, independently in x and y, so a size mismatch is
        # survivable. Note what that does NOT protect against: a
        # camera_info whose K is wrong for its own declared size. Then
        # sx=sy=1.0, no rescale, no warning -- which is exactly how the
        # preview stream drove the rover backwards. See image_topic above.
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/oak/rgb/preview/camera_info'),
        # rover/BaseDriver.cpp's velocity input, namespaced 'rover' by
        # launch/rover.py. NOTE this expects NORMALIZED throttle in
        # [-1, 1], not m/s -- robot_controller_node publishes that; a
        # stray teleop sending real m/s here will move very differently
        # than expected.
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='/rover/base/velocity/set'),
        # robot_controller_node's obstacle avoidance (see its module
        # docstring) -- ignored by goal_publisher_node.
        # The RPLIDAR A1, raw. Deliberately NOT /scan_rf2o: that filtered
        # topic exists only to feed rf2o_laser_odometry. The controller
        # does its own minimum-range rejection instead (scan_min_range in
        # config/person_follower.yaml -- it matters, this rover's lidar
        # sees its own arm).
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        # false: run detection/tracking/state-machine/visualization only
        # -- no node ever publishes a velocity or a goal, so the robot
        # can't move. Worth using for the first run on this rover.
        # See module docstring.
        DeclareLaunchArgument('enable_driving', default_value='true'),
        # false (default): robot_controller_node drives the base directly.
        # true: goal_publisher_node instead, publishing a PoseStamped
        # standoff-point goal on goal_topic for your own obstacle-avoiding
        # local planner to consume -- point goal_topic at whatever it
        # actually subscribes to. (Ignored if enable_driving:=false.)
        DeclareLaunchArgument('use_goal_publisher', default_value='false'),
        DeclareLaunchArgument('goal_topic', default_value='/person_follower/goal'),
        # person_tracker_node's tf2 lookup (camera frame ->
        # base_footprint) needs these. Both /tf and /tf_static have to be
        # remapped together if they are remapped at all: static
        # transforms (like the camera mount below) only ever publish
        # once, on /tf_static, so missing that remap silently breaks
        # lookups even if /tf itself is remapped correctly.
        # Global, un-namespaced. The TurtleBot4 put its whole graph under
        # /tb4 and so needed both of these remapped; this rover publishes
        # tf globally, so these are identity remaps kept only so the
        # arguments still exist for anyone running a namespaced setup.
        DeclareLaunchArgument('tf_topic', default_value='/tf'),
        DeclareLaunchArgument('tf_static_topic', default_value='/tf_static'),
        DeclareLaunchArgument('use_compressed_image', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params_file),
        DeclareLaunchArgument('use_visualization', default_value='true'),
        # See record_bag.launch.py -- records the whole run (camera, tf,
        # scan, odom, and every pipeline stage) automatically, starting
        # and stopping together with this launch. record_bag:=false to
        # skip it.
        DeclareLaunchArgument('record_bag', default_value='true'),
        DeclareLaunchArgument('bag_dir', default_value=default_bag_dir),
        DeclareLaunchArgument('bag_name', default_value=default_bag_name),
        DeclareLaunchArgument('record_debug_visuals', default_value='false'),

        # ---- camera mount transform ---------------------------------
        # base_footprint -> oak-d-base-frame. Nothing else in this
        # workspace publishes this link: launch/perception_oakd.py starts
        # the depthai driver with parent_frame:=oak-d-base-frame, which
        # builds the camera's internal frame tree (oak_rgb_camera_optical_frame
        # and friends hang off it) but never attaches that tree to the
        # robot. Without this, person_tracker_node's lookup from the
        # camera optical frame into base_footprint throws every frame and
        # the pipeline detects people but produces no tracks at all --
        # which looks like a detection failure and is not one.
        #
        # z / pitch / roll below come from the 2026-08-25 calibration on
        # this rover (ROS2-WorkSpace/camera_ground_calibration.yaml, the
        # same run that produced pose_detector_node's camera_height and
        # camera_pitch_deg). They transfer directly: base_footprint sits
        # on the ground by definition and is level, so "height above the
        # ground" IS camera_z, and "tilt from horizontal" IS the mount
        # pitch. Converted to radians, and note that run reported
        # status: FAILED -- see the config file for what that means.
        #
        #   height 0.272065 m      -> camera_z     0.272065
        #   tilt  -7.2139 deg      -> camera_pitch -0.125906  (nose UP)
        #   roll  -1.7351 deg      -> camera_roll  -0.030283
        #
        # camera_x AND camera_y ARE STILL A GUESS -- MEASURE THEM. The
        # ground calibration solves height/tilt/roll only; it has no way
        # to see how far forward or sideways the camera sits on the
        # robot, so those two remain a tape-measure job.
        #
        # Sign conventions differ between the two consumers, deliberately
        # not "fixed" here: this is tf (REP-103), where positive pitch is
        # nose DOWN, so the calibration's -7.2139 deg carries over with
        # its sign intact. pose_detector_node's camera_pitch_deg uses the
        # opposite convention (positive = tilted DOWN toward the floor)
        # and is in degrees. Both end up describing a camera tilted UP by
        # 7.2 deg. They describe the same mount but are consumed by
        # different code, and neither is derived from the other.
        #
        # One approximation worth knowing: the calibration measured the
        # height of the camera's OPTICAL centre, while this transform
        # positions oak-d-base-frame, which the depthai URDF then offsets
        # by a centimetre or two to reach the rgb optical frame. So
        # camera_z is high by that offset. It does not affect
        # ground_plane distances at all (those read camera_height from
        # the config, not tf); it only slightly shifts the reported
        # height of a tracked person, which nothing here uses.
        DeclareLaunchArgument(
            'camera_x', default_value='0.0',
            description='Camera mount offset, metres forward of '
                        'base_footprint -- NOT calibrated, measure it'),
        DeclareLaunchArgument(
            'camera_y', default_value='0.0',
            description='Camera mount offset, metres left of '
                        'base_footprint -- NOT calibrated, measure it'),
        DeclareLaunchArgument(
            'camera_z', default_value='0.272065',
            description='Camera mount height above base_footprint, metres '
                        '(calibrated 2026-08-25)'),
        DeclareLaunchArgument(
            'camera_yaw', default_value='0.0',
            description='Camera mount yaw, radians (0 = facing robot forward)'),
        DeclareLaunchArgument(
            'camera_pitch', default_value='-0.125906',
            description='Camera mount pitch, radians, REP-103 sign '
                        '(positive = nose down; this mount is nose UP 7.21 deg)'),
        DeclareLaunchArgument(
            'camera_roll', default_value='-0.030283',
            description='Camera mount roll, radians (calibrated 2026-08-25)'),
        # Set false if something else in your bringup already publishes
        # base_footprint -> oak-d-base-frame. Two publishers of the same
        # static transform is not harmless -- tf2 will take whichever
        # arrived last, intermittently.
        DeclareLaunchArgument('use_camera_tf', default_value='true'),
    ]

    # "<image_topic>/compressed" when use_compressed_image=true, else
    # image_topic unchanged -- keeps the remap in sync with the
    # use_compressed_image parameter passed to each node below.
    image_remap_topic = PythonExpression([
        "'", image_topic, "/compressed'",
        " if '", use_compressed_image, "' == 'true' else '", image_topic, "'",
    ])

    # enable_driving gates whether either control node runs at all;
    # use_goal_publisher then picks which one, same as before.
    run_robot_controller = PythonExpression([
        "'", enable_driving, "' == 'true' and '", use_goal_publisher, "' == 'false'"])
    run_goal_publisher = PythonExpression([
        "'", enable_driving, "' == 'true' and '", use_goal_publisher, "' == 'true'"])

    camera_mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_mount_tf',
        condition=IfCondition(LaunchConfiguration('use_camera_tf')),
        # Positional form: x y z yaw pitch roll parent child.
        arguments=[
            LaunchConfiguration('camera_x'),
            LaunchConfiguration('camera_y'),
            LaunchConfiguration('camera_z'),
            LaunchConfiguration('camera_yaw'),
            LaunchConfiguration('camera_pitch'),
            LaunchConfiguration('camera_roll'),
            'base_footprint', 'oak-d-base-frame',
        ],
        remappings=[
            ('/tf', tf_topic),
            ('/tf_static', tf_static_topic),
        ],
    )

    pose_detector_node = Node(
        package='person_follower',
        executable='pose_detector_node',
        name='pose_detector_node',
        output='screen',
        parameters=[params_file, {'use_compressed_image': use_compressed_image}],
        remappings=[
            ('~/image', image_remap_topic),
            ('~/depth', depth_topic),
            ('~/camera_info', camera_info_topic),
        ],
    )

    person_tracker_node = Node(
        package='person_follower',
        executable='person_tracker_node',
        name='person_tracker_node',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('~/detections', '/pose_detector_node/detections'),
            ('/tf', tf_topic),
            ('/tf_static', tf_static_topic),
        ],
    )

    person_follower_node = Node(
        package='person_follower',
        executable='person_follower_node',
        name='person_follower_node',
        output='screen',
        parameters=[params_file, {'use_compressed_image': use_compressed_image}],
        remappings=[
            ('~/image', image_remap_topic),
            ('~/tracks', '/person_tracker_node/tracks'),
        ],
    )

    robot_controller_node = Node(
        package='person_follower',
        executable='robot_controller_node',
        name='robot_controller_node',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(run_robot_controller),
        remappings=[
            ('~/target', '/person_follower_node/target'),
            ('~/tracks', '/person_tracker_node/tracks'),
            ('~/scan', scan_topic),
            ('/rover/base/velocity/set', cmd_vel_topic),
            # needed for the LIDAR mounting-yaw tf lookup, see
            # robot_controller_node.py's module docstring.
            ('/tf', tf_topic),
            ('/tf_static', tf_static_topic),
        ],
    )

    goal_publisher_node = Node(
        package='person_follower',
        executable='goal_publisher_node',
        name='goal_publisher_node',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(run_goal_publisher),
        remappings=[
            ('~/target', '/person_follower_node/target'),
            ('~/tracks', '/person_tracker_node/tracks'),
            ('~/goal', goal_topic),
        ],
    )

    visualization_node = Node(
        package='person_follower',
        executable='visualization_node',
        name='visualization_node',
        output='screen',
        parameters=[params_file, {'use_compressed_image': use_compressed_image}],
        condition=IfCondition(use_visualization),
        remappings=[
            ('~/image', image_remap_topic),
            ('~/tracks', '/person_tracker_node/tracks'),
            ('~/target', '/person_follower_node/target'),
        ],
    )

    record_bag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'record_bag.launch.py')),
        condition=IfCondition(record_bag),
        # Pass through what this launch actually resolved to, not
        # record_bag.launch.py's own independent defaults -- so a bag
        # always reflects the real topics this run used, even if
        # image_topic/scan_topic/etc were overridden above.
        launch_arguments={
            'bag_dir': bag_dir,
            'bag_name': bag_name,
            'image_topic': image_remap_topic,
            'camera_info_topic': camera_info_topic,
            'scan_topic': scan_topic,
            'tf_topic': tf_topic,
            'tf_static_topic': tf_static_topic,
            'cmd_vel_topic': cmd_vel_topic,
            'record_debug_visuals': record_debug_visuals,
        }.items(),
    )

    return LaunchDescription(declare_args + [
        camera_mount_tf,
        pose_detector_node,
        person_tracker_node,
        person_follower_node,
        robot_controller_node,
        goal_publisher_node,
        visualization_node,
        record_bag_launch,
    ])
