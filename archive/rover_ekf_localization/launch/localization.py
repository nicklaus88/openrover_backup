"""Localization against a map you already built: odometry + map_server + AMCL.

LOCALIZATION ONLY. This file starts no drivers and no robot frames --
that is launch/robot_start.py's job. Bring the robot up first, then
localize on top of it:

    ros2 launch launch/robot_start.py          # base, cameras, lidar, TF tree
    ros2 launch launch/localization.py         # this file

The counterpart to launch/mapping.py. mapping.py BUILDS a map with
slam_toolbox; this file LOCALIZES in one that already exists, publishing
the same map -> odom transform slam_toolbox would have:

    map -> odom              amcl (this file)
    odom -> base_footprint   this file, one of (see ODOMETRY SOURCES):
                             odom_source:=rover -> odom_tf_broadcaster
                             odom_source:=rf2o  -> rf2o_laser_odometry
                             odom_source:=fused -> ekf_filter_node
    /map                     map_server (from maps/*.pgm + .yaml)

  and from robot_start.py, which this file expects to be running:

    base_footprint -> laser  robot_state_publisher, from
                             description/open_rover.urdf.xacro
    /scan                    sllidar_node
    /rover/base/odometry     rover base node (rover and fused sources)

Miss any link and AMCL starts, drops every scan, and never publishes
map -> odom. To check it is really localizing, don't trust the log --
check that the transform exists:

    ros2 run tf2_ros tf2_echo map odom


WHY AMCL AND NOT slam_toolbox's LOCALIZATION MODE
-------------------------------------------------
slam_toolbox can only localize against a serialized posegraph
(.posegraph + .data, written by its /slam_toolbox/serialize_map
service). Everything in ros2_ws/maps/ is map_saver output instead -- a
.pgm image plus a .yaml -- and the posegraph cannot be reconstructed
from a rendered image. AMCL reads .pgm/.yaml directly, so the lab maps
work as they are. If you would rather stay on slam_toolbox, re-run
mapping.py, call serialize_map, and use mode: localization.


ODOMETRY SOURCES
----------------
AMCL never subscribes to an odometry TOPIC. It reads the odom ->
base_footprint TRANSFORM and uses it as the motion prior between scan
updates. "Adding" an odometry source therefore means changing who
broadcasts that one TF edge -- and exactly one node may, because two
publishers of the same edge make TF alternate between them and AMCL
jitter. odom_source picks the publisher:

    odom_source:=rover   lidar_toolbox/odom_tf_broadcaster, republishing
                         /rover/base/odometry as TF. This is open-loop:
                         BaseDriver.cpp derives linear velocity from the
                         COMMANDED velocity times odom_k_x/odom_k_y, not from
                         encoders, with heading from the IMU. It drifts
                         whenever the wheels slip or the rover is pushed.

    odom_source:=rf2o    rf2o_laser_odometry, which
                         estimates planar motion by matching consecutive
                         scans and broadcasts odom -> base_footprint
                         itself (publish_tf). Grounded in what the lidar
                         actually sees, so it does not care about wheel
                         slip -- but it degrades in featureless corridors
                         where consecutive scans look identical, exactly
                         where wheel odometry still works.

                         It is fed /scan_rf2o, not /scan: this lidar sees
                         the robot's own arm and rf2o, unlike AMCL, has
                         no range filter to drop those returns. See
                         config/scan_filter.yaml -- that node starts and
                         stops with rf2o.

    odom_source:=fused   THE DEFAULT. robot_localization's EKF, consuming
                         BOTH of the above: the rover's odometry for
                         velocity (twist only -- its pose error is
                         unbounded) and rf2o for pose, differentially.
                         The two fail in opposite conditions, so together
                         they cover each other:
                         scan matching catches the wheel slip, commanded
                         velocity carries the estimate through a corridor
                         where consecutive scans look identical.

                         rf2o still runs here, with publish_tf FORCED
                         FALSE -- the EKF owns odom -> base_footprint and
                         only one node may. See config/ekf_fused_odom.yaml
                         for what is fused and why the IMU is not.

amcl_params.yaml's alpha1..alpha5 are the FUSED set, matching the default
(validated on the robot 2026-09-24 with scripts/odom_diagnose.py). They
are the tightest of the three sets in that file. Switching odom_source
means switching alphas too: the rf2o set for :=rf2o, the loosest set for
:=rover -- running either on the fused set over-trusts its odometry.

If the fused estimate misbehaves, odom_source:=rf2o (with the rf2o
alphas) is the fallback. Re-check fused with scripts/odom_diagnose.py
after any change to the base, flight controller or lidar.

rf2o needs the base_footprint -> laser transform before it can start;
robot_start.py's robot_state_publisher provides it on /tf_static (latched).
Upstream rf2o looked it up ONCE, on the first scan, and on failure did
NOT stop -- it carried on with an identity lidar pose. This lidar is
yawed by pi, so that reversed every forward and sideways move, and with
robot_start.py already streaming /scan the first scan usually beats
/tf_static into the fresh TF buffer. The local patch in
src/rf2o_laser_odometry now drops scans until the transform resolves,
logging "Waiting for TF ..." and then "Laser pose from TF acquired".
If you never see the second line, the URDF is not being published.

The /odom_rf2o topic it publishes is inspection-only on
odom_source:=rf2o, where AMCL reads the TF instead. On 'fused' it stops
being decorative: it is the EKF's second input.

use_rf2o cuts across odom_source rather than extending it. odom_source
picks the publisher; use_rf2o:=false removes rf2o without promoting the
rover's odometry in its place, leaving odom -> base_footprint unpublished
so that a node outside this file can own it. On odom_source:=fused it
does not silence the EKF, it starves it: the filter keeps running on
commanded velocity alone. use_amcl:=false keeps the odometry but drops
the map and map -> odom, for a bare odometry test.

WHO PUBLISHES /rover/base/odometry
-----------------------------------
robot_start.py, via the rover base node. odom_source:=rover and
odom_source:=fused both read it. Without it, 'fused' does not fail
loudly: the EKF runs on rf2o alone and the corridor coverage it exists
for is gone. Check with

    ros2 topic hz /rover/base/odometry      # expect ~50 Hz


RUNNING NEXT TO OTHER LAUNCH FILES
----------------------------------
    mapping.py            starts its own odom_tf_broadcaster (a second
                          odom -> base_footprint publisher), lidar and
                          robot_state_publisher. Never run it with this
                          file or with robot_start.py.
    person_follower       pass use_camera_tf:=false -- robot_start.py
                          already publishes base_footprint ->
                          oak-d-base-frame.


LIDAR MOUNT
-----------
The laser_x/y/z/yaw arguments are gone. Every frame now comes from
description/open_rover.urdf.xacro via robot_state_publisher, which puts
the lidar 0.127 m forward of base_footprint -- the measured value. The
arguments they replace defaulted to 0.2 m and were labelled UNVERIFIED.

That 73 mm matters for the maps already in maps/, all of which were built
through the 0.2 m value. A wrong lever arm does not shift a map uniformly:
the error rotates with the robot, so those maps carry a heading-dependent
smear of up to ~146 mm peak to peak. AMCL still converges against them at
resolution 0.05 with sigma_hit 0.1, with a small bias and a slightly
fatter particle cloud. Re-map a space to be rid of it.


USAGE
-----
    ros2 launch launch/robot_start.py        # first, in another terminal
    ros2 launch launch/localization.py       # no arguments needed
    ros2 launch launch/localization.py map:=RoboPI_WetLab
    ros2 launch launch/localization.py odom_source:=rf2o     # scan matching only
    ros2 launch launch/localization.py odom_source:=rover    # commanded velocity only
    ros2 launch launch/localization.py set_initial_pose:=true x:=1.5 y:=-2.0 yaw:=1.57
    ros2 launch launch/localization.py rviz:=false           # headless
    ros2 launch launch/localization.py use_amcl:=false        # odometry only
    ros2 launch launch/localization.py odom_source:=rf2o use_rf2o:=false

By default rviz2 opens and AMCL WAITS for you: it publishes no
map -> odom at all until it receives a pose on /initialpose. Pick the
"2D Pose Estimate" tool in rviz2, click where the robot is on the map and
drag toward the way it faces, then drive a metre or two so the filter
converges (it only updates after 0.10 m / 0.10 rad of motion). That is the
default because AMCL refines a guess rather than searching the map, and a
wrong guess -- the old default was the map origin -- stays wrong.

If you know the pose, set_initial_pose:=true x:=... y:=... yaw:=... seeds
it at startup instead. A 2D Pose Estimate still overrides it either way.
Headless runs (rviz:=false) need one of that or an /initialpose
published from elsewhere, or AMCL never starts localizing.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# abspath() matters: `ros2 launch launch/localization.py` sets __file__ to
# the RELATIVE path you typed, so a bare dirname() chain yields
# "ros2_ws/maps/..." and map_server fails with a bare "bad file" error.
# mapping.py already takes the abspath route for the same reason.
HOME = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

CONFIG = os.path.join(HOME, "config", "amcl_params.yaml")

RVIZ = os.path.join(HOME, "config", "localization.rviz")

# Range filter for rf2o's input scan. See the file itself for why rf2o
# needs one and AMCL does not.
SCAN_FILTER = os.path.join(HOME, "config", "scan_filter.yaml")

# Deriving the path from __file__ rather than hardcoding it means the same
# string resolves correctly on the Jetson host AND inside the container,
# where this workspace is mounted as /workspace/ROS2-WorkSpace.
#
# This used to point at ../ros2_ws/maps, which does not exist -- the maps
# are in this workspace. map_server failed with a bare "bad file" error.
MAPS = os.path.join(HOME, "maps")

# EKF that fuses the two odometry sources. See the file itself.
EKF = os.path.join(HOME, "config", "ekf_fused_odom.yaml")


def generate_launch_description():
    map_name = LaunchConfiguration("map")
    odom_source = LaunchConfiguration("odom_source")

    map_yaml = PathJoinSubstitution([MAPS, [map_name, ".yaml"]])

    # Two gates, not one: odom_source picks WHICH odometry, and use_rf2o
    # can veto it outright without promoting the rover's in its place.
    # Shared by rf2o and the scan filter that feeds it, so the two can
    # never end up half-started.
    #
    # 'fused' needs rf2o running too -- it is one of the EKF's two inputs.
    # The difference is who broadcasts TF, handled by rf2o_publishes_tf.
    rf2o_enabled = IfCondition(AndSubstitution(
        LaunchConfiguration("use_rf2o"),
        PythonExpression(
            ["'", odom_source, "' in ('rf2o', 'fused')"]),
    ))

    # rf2o owns odom -> base_footprint on odom_source:=rf2o, but on 'fused'
    # the EKF owns it and rf2o must stay off that edge. Exactly one writer.
    rf2o_publishes_tf = ParameterValue(
        PythonExpression(["'", odom_source, "' == 'rf2o'"]), value_type=bool)

    args = [
        DeclareLaunchArgument(
            "map", default_value="RoboPI_DryLab_corridor",
            description=f"Map basename (no .yaml) in {MAPS}"),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="Open rviz2 with config/localization.rviz "
                        "(needed for the 2D Pose Estimate; false = headless)"),
        # AMCL's starting guess. It is NOT a global search: the particle
        # filter only refines from here, so a wrong pose stays wrong until
        # you correct it in rviz2.
        DeclareLaunchArgument(
            "x", default_value="0.0",
            description="Initial pose x in the map frame"),
        DeclareLaunchArgument(
            "y", default_value="0.0",
            description="Initial pose y in the map frame"),
        DeclareLaunchArgument(
            "yaw", default_value="0.0",
            description="Initial pose yaw in the map frame, radians"),
        # See ODOMETRY SOURCES above. fused is the default: rf2o's
        # measured motion plus commanded velocity to carry it through
        # featureless stretches. rf2o alone is the fallback if the EKF
        # misbehaves; rover alone is the last resort.
        DeclareLaunchArgument(
            "odom_source", default_value="fused",
            choices=["rover", "rf2o", "fused"],
            description="Who broadcasts odom -> base_footprint: the rover's "
                        "own open-loop odometry, RF2O scan matching, or an "
                        "EKF fusing both"),
        # The A1 delivers ~7.5 scans/second in Sensitivity mode. RF2O
        # cannot go faster than the scans it consumes -- ask it to and it
        # just logs a warning every cycle -- so this tracks the sensor
        # rather than upstream's optimistic 20.0.
        DeclareLaunchArgument(
            "rf2o_freq", default_value="7.0",
            description="RF2O update rate in Hz; keep at or below the "
                        "lidar's actual scan rate"),
        # A plain off switch. Distinct
        # from odom_source:=rover, which SWAPS in the wheel odometry:
        # use_rf2o:=false leaves odom -> base_footprint unpublished
        # entirely, for when something outside this file already owns it.
        DeclareLaunchArgument(
            "use_rf2o", default_value="true",
            description="Start rf2o_laser_odometry when odom_source is "
                        "rf2o or fused (false on rf2o = no odometry TF at "
                        "all; on fused = EKF runs on commanded velocity "
                        "alone)"),
        # Skips map_server, amcl AND the lifecycle manager -- all three go
        # together, since the manager exists only to activate the other
        # two. What is left is odometry alone, for a bare odometry test.
        DeclareLaunchArgument(
            "use_amcl", default_value="true",
            description="Start map_server + amcl + lifecycle_manager "
                        "(false = no map, no map -> odom)"),
        # AMCL's particle filter has to start SOMEWHERE. With this true it
        # starts at x/y/yaw; with it false it holds off until you publish
        # /initialpose, which in practice means rviz2's "2D Pose Estimate"
        # tool. False is the honest choice when you do not actually know
        # where the robot is -- a wrong initial guess is worse than none,
        # because AMCL refines from it rather than searching globally.
        #
        # Default false: localizing always starts with a 2D Pose Estimate.
        DeclareLaunchArgument(
            "set_initial_pose", default_value="false",
            description="Seed AMCL with x/y/yaw at startup "
                        "(false = wait for a 2D Pose Estimate in rviz2)"),
    ]

    nodes = [
        # odom -> base_footprint, source 1 of 3. Mutually exclusive with
        # the rf2o and EKF nodes below -- see ODOMETRY SOURCES.
        Node(
            package="lidar_toolbox",
            executable="odom_tf_broadcaster",
            name="odom_tf_broadcaster",
            condition=LaunchConfigurationEquals("odom_source", "rover"),
        ),
        # Strips the robot's own arm out of /scan and republishes as
        # /scan_rf2o, purely so rf2o below has a clean input. AMCL still
        # reads raw /scan -- its laser_min_range already handles the same
        # beams, and leaving it on the raw topic keeps its behaviour
        # identical to how the maps were validated.
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            name="rf2o_scan_filter",
            output="screen",
            condition=rf2o_enabled,
            parameters=[SCAN_FILTER],
            remappings=[("scan", "/scan"), ("scan_filtered", "/scan_rf2o")],
        ),
        # odom -> base_footprint, source 2 of 3: planar scan matching over
        # consecutive scan frames. On 'fused' it runs but stays off TF.
        Node(
            package="rf2o_laser_odometry",
            executable="rf2o_laser_odometry_node",
            name="rf2o_laser_odometry",
            output="screen",
            condition=rf2o_enabled,
            parameters=[{
                # NOT raw /scan. rf2o has no range filtering of its own
                # and this lidar sees the robot's own arm at ~0.18 m --
                # see config/scan_filter.yaml.
                "laser_scan_topic": "/scan_rf2o",
                # Inspection-only on odom_source:=rf2o, where AMCL reads
                # the TF instead. On 'fused' this is the EKF's odom1.
                "odom_topic": "/odom_rf2o",
                # True on odom_source:=rf2o, where this is what feeds
                # AMCL. False on 'fused', where the EKF owns the edge.
                "publish_tf": rf2o_publishes_tf,
                # Upstream defaults to base_link. Everything on this robot
                # speaks base_footprint (amcl_params.yaml, the URDF,
                # rover/base), and a mismatch is not a warning: RF2O
                # looks up base_frame_id -> laser on the first scan, throws
                # TransformException when that frame does not exist, and
                # never publishes any odometry at all.
                "base_frame_id": "base_footprint",
                "odom_frame_id": "odom",
                # Upstream defaults this to "/base_pose_ground_truth", a
                # simulator topic that does not exist here; RF2O would wait
                # for it forever. Empty means "start at the origin", which
                # is right because odom is defined by wherever the robot
                # started anyway.
                "init_pose_from_topic": "",
                "freq": LaunchConfiguration("rf2o_freq"),
            }],
        ),
        # odom -> base_footprint, source 3 of 3: an EKF fusing the two
        # above. rf2o measures translation but degenerates in a featureless
        # corridor; the rover's commanded velocity never degenerates but
        # drifts on slip. Running both covers each other's failure.
        #
        # Started only on odom_source:=fused, and when it runs neither
        # odom_tf_broadcaster nor rf2o broadcasts TF -- see
        # config/ekf_fused_odom.yaml.
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            condition=LaunchConfigurationEquals("odom_source", "fused"),
            parameters=[EKF],
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_amcl")),
            parameters=[CONFIG, {"yaml_filename": map_yaml}],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_amcl")),
            parameters=[CONFIG, {
                "set_initial_pose": LaunchConfiguration("set_initial_pose"),
                # Still passed when set_initial_pose is false; AMCL simply
                # ignores them in that case, so there is no need to make
                # the parameter list itself conditional.
                "initial_pose.x": LaunchConfiguration("x"),
                "initial_pose.y": LaunchConfiguration("y"),
                "initial_pose.z": 0.0,
                "initial_pose.yaw": LaunchConfiguration("yaw"),
            }],
        ),
        # Use THIS config, not the sllidar one. map_server publishes /map
        # once, latched TRANSIENT_LOCAL; rviz's Map display defaults to
        # VOLATILE, which never receives an already-sent latched message,
        # so the map silently never appears. localization.rviz sets the
        # display to Transient Local and the fixed frame to map.
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_localization",
            arguments=["-d", RVIZ],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
        # map_server and amcl are LIFECYCLE nodes: launching them only
        # gets them to "unconfigured". Without this manager to configure
        # and activate them they sit there silently -- no /map, no
        # map -> odom, and no error to explain why.
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_amcl")),
            parameters=[{
                "autostart": True,
                "node_names": ["map_server", "amcl"],
            }],
        ),
    ]

    return LaunchDescription(args + nodes)
