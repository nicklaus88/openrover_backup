# aprilgrid_ground_calibrator

Calibrates **camera height above the ground** and **camera downward tilt
angle** (plus roll) for an OAK-D camera, using a planar fiducial target
placed flat on the floor. Defaults target this rover's OAK-D Lite
(`/oak/rgb/*`); it was originally written against a TurtleBot4.

Its output feeds `person_follower`'s `camera_height` / `camera_pitch_deg`
(`config/person_follower.yaml`), which are what `position_source:
"ground_plane"` uses to turn ankle keypoints into a distance. Those two
numbers *are* the distance measurement in that mode, so calibrate before
trusting any distance the follower reports. Two target types are supported,
selected via the `fiducial_type` parameter:

* `apriltag_grid` (default): a 4x5 AprilGrid (20 x tag36h11 tags). The
  whole board is treated as ONE rigid target: all visible tag corners
  (up to 80 with 20 tags visible) are fed into a single joint OpenCV PnP
  solve, not 20 independent per-tag poses that get averaged.
* `aruco_single`: a single square ArUco marker (e.g. a large printed
  120mm `DICT_ARUCO_ORIGINAL` marker) instead of the grid. Much easier to
  print/mount large and legible from a distance, at the cost of only 4
  point correspondences instead of up to 80 (so tilt/roll -- especially
  roll -- are noisier per-frame; the 100-sample robust-median averaging
  matters more here). The shipped [config/calibrator.yaml](config/calibrator.yaml)
  is currently set up for this mode.

See the extensive comments at the top of
[`aprilgrid_ground_calibrator/calibrator_node.py`](aprilgrid_ground_calibrator/calibrator_node.py)
for the exact coordinate-frame and sign conventions used, including the
two different (and non-interchangeable) tag/marker corner-order
conventions used by the two target types.

## 1. Dependencies

### apt / rosdep (ROS2 Humble)

```bash
sudo apt update
sudo apt install -y \
    python3-opencv python3-numpy python3-yaml \
    ros-humble-cv-bridge ros-humble-tf2-ros \
    ros-humble-sensor-msgs ros-humble-std-msgs ros-humble-geometry-msgs
```

### pip (AprilTag detector)

```bash
pip3 install pupil-apriltags
```

`pupil_apriltags` is the preferred, dependency-light AprilTag detector
and is NOT available via apt/rosdep, so it must be pip-installed.

### Optional: external `apriltag_ros` detection mode

If you'd rather reuse an existing `apriltag_ros` detection pipeline
instead of detecting tags inside this node, install/build
`apriltag_msgs` and set `detection_source: "apriltag_ros"` (see
[config/calibrator.yaml](config/calibrator.yaml)). This mode is optional
and secondary to the default standalone detector.

## 2. Build

```bash
cd /workspace/ROS2-WorkSpace
colcon build --packages-select aprilgrid_ground_calibrator
source install/setup.bash
```

Bring the camera up first (`ros2 launch launch/perception_oakd.py`), then
run the calibrator against it.

## 3. Run

```bash
ros2 launch aprilgrid_ground_calibrator calibrator.launch.py
```

Override any parameter via a custom config file:

```bash
ros2 launch aprilgrid_ground_calibrator calibrator.launch.py \
    config_file:=/path/to/my_calibrator.yaml
```

or run the node directly with inline parameter overrides:

```bash
ros2 run aprilgrid_ground_calibrator calibrator_node --ros-args \
    -p image_topic:=/oak/rgb/image_raw \
    -p camera_info_topic:=/oak/rgb/camera_info \
    -p samples_required:=100
```

Place the AprilGrid flat on the floor, keep the robot and board
stationary, and make sure at least `min_tags` tags stay visible until
`samples_required` frames have been accepted.

## 4. Topics

Subscribed:

| Topic | Type |
|---|---|
| `image_topic` (default `/oak/rgb/image_raw`) | `sensor_msgs/Image` |
| `camera_info_topic` (default `/oak/rgb/camera_info`) | `sensor_msgs/CameraInfo` |
| `apriltag_ros_topic` (only if `detection_source: apriltag_ros`) | `apriltag_msgs/AprilTagDetectionArray` |

Published:

| Topic | Type |
|---|---|
| `/aprilgrid_calibration/height` | `std_msgs/Float64` |
| `/aprilgrid_calibration/tilt` | `std_msgs/Float64` (signed downward tilt) |
| `/aprilgrid_calibration/roll` | `std_msgs/Float64` |
| `/aprilgrid_calibration/reprojection_error` | `std_msgs/Float64` (RMSE, px) |
| `/aprilgrid_calibration/debug_image` | `sensor_msgs/Image` |
| `/aprilgrid_calibration/camera_pose` | `geometry_msgs/PoseStamped` (camera pose in the `ground` frame) |

Optional TF: set `publish_tf: true` to broadcast `ground -> <camera_optical_frame_id>`
on **two** topics:

* `/tf` — one transform per *accepted* frame while samples are still being
  collected, so you can watch the pose converge live in rviz. Stops
  naturally once calibration finishes (the node stops processing frames).
* `/tf_static` — published **once**, when calibration finishes, using the
  robust-median result. This is latched (transient-local), so anything
  that subscribes later (e.g. your ground-plane estimator, started after
  this node finished) still receives it, and it carries the final,
  low-noise number rather than any single noisy frame. The exact
  `R_board_camera`/`t_board_camera` used is taken from whichever accepted
  frame is closest to the robust median height/tilt/roll — the three
  scalar medians alone don't fully determine a rotation (yaw is
  intentionally never tracked, see below), so it's built from a real
  captured pose rather than a synthesized one.

No `base_link -> camera` transform is published, because the AprilGrid
only fixes the vertical relationship (height/pitch/roll) to the floor —
its horizontal position and yaw on the floor are arbitrary.

## 5. Output

On completion (`samples_required` accepted frames), the node prints a
full statistics report to the console and writes
`camera_ground_calibration.yaml` (path configurable via
`output_yaml_path`) with the final robust-median height/tilt/roll,
their standard deviations, reprojection RMSE, and the camera intrinsics
used.

## Example console output

```
[INFO] Detected tags: 18 / 20 | Used corners: 72 | PnP inliers: 70 | Reprojection RMSE: 0.312 px
[INFO] Height: 0.6187 m | Tilt: 14.84 deg | Roll: 0.27 deg | Reprojection RMSE: 0.312 px | ACCEPTED (100/100)
[INFO] ============================================================
[INFO] APRILGRID CAMERA-TO-GROUND CALIBRATION
[INFO] ============================================================
[INFO]
[INFO] Board:
[INFO]     rows               : 5
[INFO]     columns            : 4
[INFO]     tag size           : 0.0400 m
[INFO]     tag spacing        : 0.0080 m
[INFO]     tag pitch          : 0.0480 m
[INFO]
[INFO] Samples:
[INFO]     requested          : 100
[INFO]     accepted           : 100
[INFO]     rejected           : 14
[INFO]
[INFO] Tag detections:
[INFO]     median tags/frame  : 19
[INFO]     median corners     : 76
[INFO]
[INFO] Reprojection:
[INFO]     median RMSE        : 0.24 px
[INFO]
[INFO] CAMERA HEIGHT:
[INFO]     median             : 0.6184 m
[INFO]     mean               : 0.6185 m
[INFO]     std                : 0.0012 m (2 outlier(s) rejected)
[INFO]
[INFO] CAMERA DOWNWARD TILT:
[INFO]     median             : 14.82 deg
[INFO]     mean               : 14.83 deg
[INFO]     std                : 0.07 deg (1 outlier(s) rejected)
[INFO]
[INFO] CAMERA ROLL:
[INFO]     median             : 0.31 deg
[INFO]     mean               : 0.30 deg
[INFO]     std                : 0.05 deg (0 outlier(s) rejected)
[INFO]
[INFO] ============================================================
[INFO] FINAL VALUES
[INFO] ============================================================
[INFO]
[INFO] camera_height_m = 0.6184
[INFO] camera_tilt_deg = 14.82
[INFO] camera_roll_deg = 0.31
[INFO]
[INFO] CALIBRATION STATUS: GOOD
[INFO]     - all stability and reprojection checks passed
[INFO] ============================================================
[INFO] Wrote calibration result to: camera_ground_calibration.yaml
[INFO] Calibration complete. Node remains alive (publishing stops); press Ctrl+C to exit.
```

## Tag ID mapping

Assumed standard Kalibr-style row-major layout (`row = tag_id // cols`,
`col = tag_id % cols`), isolated entirely in `tag_id_to_row_col()` in
[`calibrator_node.py`](aprilgrid_ground_calibrator/calibrator_node.py)
so it's a one-function change if your physical board differs:

```
 0  1  2  3
 4  5  6  7
 8  9 10 11
12 13 14 15
16 17 18 19
```

The debug image (`/aprilgrid_calibration/debug_image`, or the OpenCV
window when `show_debug_image: true`) draws each tag's ID, assigned
row/col, and per-corner index (0-3) so this mapping and the assumed
corner winding order can be verified visually against the physical
board. If corner index 0 is not the physical bottom-left corner on your
prints, set `flip_tag_corner_order: true`.
