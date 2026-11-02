"""One-shot bringup: starts the OAK-D camera (depthai_ros_driver) with an
RGBD pipeline aligned/rectified as this package expects, then starts the
person_follower pipeline on top of it.

Convenience only. On this rover you will more often want
launch/perception_oakd.py, which brings up the same camera alongside the
rover base, the two side cameras and the lidar -- then
person_follower.launch.py on top of that. Use this file when you want the
follower and nothing else.

Camera settings come from oakd_camera's oakd_lite.yaml, the same file
launch/perception_oakd.py uses, so both paths configure the camera
identically. That matters on the Lite specifically: it has no IMU and no
IR projector, and its mono pair caps at 480P, all of which that config
handles and the driver's defaults do not.

NOTE this does NOT start the lidar, so robot_controller_node's obstacle
avoidance and its sideways clearance guard will have no scan to read.
They fail safe (no scan means no avoidance nudge and no strafe veto,
see _scan_zones returning None) -- but that means "no obstacle
avoidance at all", not "avoidance is fine". Do not drive this way.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    depthai_share = get_package_share_directory('depthai_ros_driver')
    person_follower_share = get_package_share_directory('person_follower')

    oakd_camera_share = get_package_share_directory('oakd_camera')
    default_params_file = oakd_camera_share + '/config/oakd_lite.yaml'

    camera_name = LaunchConfiguration('camera_name')
    camera_params_file = LaunchConfiguration('camera_params_file')

    declare_args = [
        DeclareLaunchArgument('camera_name', default_value='oak'),
        DeclareLaunchArgument(
            'camera_params_file', default_value=default_params_file,
            description='depthai_ros_driver settings for the OAK-D Lite'),
    ]

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [depthai_share, '/launch/camera.launch.py']),
        launch_arguments={
            'name': camera_name,
            'params_file': camera_params_file,
            'camera_model': 'OAK-D-LITE',
            'parent_frame': 'oak-d-base-frame',
            'rectify_rgb': 'true',
        }.items(),
    )

    person_follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [person_follower_share, '/launch/person_follower.launch.py']),
        launch_arguments={
            # The 512x288 (16:9) on-device preview, matching
            # person_follower.launch.py's defaults.
            #
            # These are OVERRIDES: they win over person_follower.launch.py's
            # own defaults, so anything changed there must be changed here
            # too or launching from this file silently ignores it.
            #
            # Note this gives up rectification -- the old value here was
            # /rgb/image_rect, whose camera_info has zero distortion, which
            # is exactly right for pose_detector_node (it uses K only). The
            # preview is unrectified, so expect slightly worse accuracy
            # toward the frame edges.
            'image_topic': ['/', camera_name, '/rgb/preview/image_raw'],
            'depth_topic': ['/', camera_name, '/stereo/image_raw'],
            'camera_info_topic': ['/', camera_name, '/rgb/preview/camera_info'],
            # MUST travel with the preview camera_info above. 1.0 because
            # the preview is now 16:9 and therefore uncropped, so the
            # driver's reported fx=fy=402.64 is already correct. A square
            # preview needs 1.7777778 here instead -- see camera_focal_scale
            # in person_follower.launch.py.
            'camera_focal_scale': '1.0',
        }.items(),
    )

    return LaunchDescription(declare_args + [camera_launch, person_follower_launch])
