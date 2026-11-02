"""Localize in an existing map: fused odometry + map_server + AMCL.

    ros2 launch rover_bringup robot.launch.py                       # first
    ros2 launch rover_bringup localization.launch.py map:=RoboPI_WetLab
    ros2 launch rover_bringup localization.launch.py map:=PST rviz:=false \\
        set_initial_pose:=true x:=1.5 y:=-2.0 yaw:=1.57

Maps are the .yaml basenames in rover_bringup/maps.

AMCL publishes nothing until it has an initial pose. With rviz (the
default), use "2D Pose Estimate": click where the robot is and drag the
way it faces, then drive a metre or two so it converges. Headless runs
need set_initial_pose:=true with x/y/yaw.

Check it is localizing:  ros2 run tf2_ros tf2_echo map odom
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

SHARE = get_package_share_directory("rover_bringup")
AMCL_PARAMS = os.path.join(SHARE, "config", "amcl_params.yaml")
RVIZ = os.path.join(SHARE, "config", "localization.rviz")
MAPS = os.path.join(SHARE, "maps")


def generate_launch_description():
    map_yaml = PathJoinSubstitution([MAPS, [LaunchConfiguration("map"), ".yaml"]])

    odometry = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(SHARE, "launch", "odometry.launch.py")))

    return LaunchDescription([
        DeclareLaunchArgument(
            "map", default_value="RoboPI_DryLab_corridor",
            description="Map basename (no .yaml) in rover_bringup/maps"),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="Open rviz2 (needed for 2D Pose Estimate)"),
        DeclareLaunchArgument(
            "set_initial_pose", default_value="false",
            description="Seed AMCL with x/y/yaw instead of waiting for rviz"),
        DeclareLaunchArgument("x", default_value="0.0", description="Initial x, map frame (m)"),
        DeclareLaunchArgument("y", default_value="0.0", description="Initial y, map frame (m)"),
        DeclareLaunchArgument("yaw", default_value="0.0", description="Initial yaw, map frame (rad)"),

        odometry,
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[AMCL_PARAMS, {"yaml_filename": map_yaml}],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[AMCL_PARAMS, {
                "set_initial_pose": LaunchConfiguration("set_initial_pose"),
                "initial_pose.x": LaunchConfiguration("x"),
                "initial_pose.y": LaunchConfiguration("y"),
                "initial_pose.z": 0.0,
                "initial_pose.yaw": LaunchConfiguration("yaw"),
            }],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{"autostart": True, "node_names": ["map_server", "amcl"]}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_localization",
            arguments=["-d", RVIZ],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
