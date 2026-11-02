# rover_ws

ROS 2 Jazzy workspace for the omniwheel rover: Jetson Orin Nano Super,
Betaflight flight controller base (MSP), RPLIDAR A1, OAK-D Lite and two
BlueRobotics side cameras. Runs natively on Ubuntu 24.04.

The Humble-in-Docker workspace this replaced is on the `humble-archive` branch.

## Build

```bash
cd ~/rover_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash        # ~/.bashrc does this in new shells
```

## Run

Start the drivers first, then localization **or** mapping on top (never both).

```bash
ros2 launch rover_bringup robot.launch.py                    # drivers + TF
ros2 launch rover_bringup localization.launch.py map:=RoboPI_WetLab
ros2 launch rover_bringup mapping.launch.py
```

- **Localization:** set the start pose with rviz "2D Pose Estimate", then
  drive 1–2 m. Headless: `rviz:=false set_initial_pose:=true x:=.. y:=.. yaw:=..`.
- **Mapping:** save with
  `ros2 run nav2_map_server map_saver_cli -f ~/rover_ws/src/rover_bringup/maps/<name>`,
  then rebuild.
- **Teleop:** `python3 scripts/rover_teleop.py`.
- **No side cameras:** `robot.launch.py use_side_cameras:=false`.

## Layout

| Path | What |
|---|---|
| `src/rover_bringup` | launch files, configs, URDF, maps (13) |
| `src/rover` | C++ base driver: velocity in, odometry + IMU out over MSP |
| `src/rf2o_laser_odometry` | lidar scan-matching odometry (local TF-wait patch) |
| `src/sllidar_ros2` | RPLIDAR driver |
| `src/side_cameras` | left/right USB cameras |
| `src/person_follower`, `src/person_follower_msgs` | person following (needs PyTorch + Ultralytics, not installed) |
| `scripts/` | `rover_teleop.py`, `odom_diagnose.py` |
| `models/` | `yolo11n-pose.pt` for person_follower |
| `docs/localization.md` | how the fused EKF odometry was calibrated (2026-09-24) |

Odometry: rf2o + the base's commanded velocity fused by robot_localization's
EKF (`odometry.launch.py`), shared by localization and mapping.

## Known gaps

- OAK-D mount x/y offset is not calibrated (0, 0 in `robot.launch.py`).
- person_follower won't run until PyTorch (JetPack 7 build) and Ultralytics
  are installed. Keep `numpy<2` and remove `opencv-python` afterwards so
  `cv_bridge` keeps the system builds.
