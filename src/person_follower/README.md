# person_follower

A ROS2 (Humble) person-detection/tracking/following stack for the
**RoverMaster omnidirectional rover** + OAK-D Lite, built in the spirit of
[koide3/monocular_person_following](https://github.com/koide3/monocular_person_following):
pose detection → tracking → a re-identification state machine → a
proportional velocity controller (design rationale — why not real
OpenPose, why depth instead of the original's ground-plane trick, etc —
below and in this package's design discussion).

```
camera --image/depth/camera_info-->
  pose_detector_node --detections--> person_tracker_node --tracks-->
    person_follower_node --target--> robot_controller_node --> /rover/base/velocity/set
                                 \--> visualization_node --> annotated image + rviz markers
```

> **Ported from a TurtleBot4.** Topics, frames and — most importantly —
> the *control law* were all changed for this base. If you are looking at
> older notes, bags or configs, see
> [Omnidirectional control](#omnidirectional-control) below for what is
> no longer true.

| | TurtleBot4 (before) | This rover (now) |
|---|---|---|
| velocity topic | `/tb4/cmd_vel` | `/rover/base/velocity/set` |
| velocity feedback | *(none — odom only)* | `/rover/base/velocity/get` (recorded, not consumed) |
| velocity units | m/s, rad/s | **normalized throttle, [-1, 1]** |
| scan | `/tb4/scan` | `/scan` |
| camera | `/tb4/oakd/rgb/preview/*` (250×250) | `/oak/rgb/*` (1280×720) |
| side cameras | *(none)* | `/capture/camera_left\|right/img` (recorded, not consumed) |
| tf | `/tb4/tf`, `/tb4/tf_static` | `/tf`, `/tf_static` |
| base frame | `base_link` | `base_footprint` (there is no `base_link`) |
| lateral error | corrected by **turning** | corrected by **strafing** |
| obstacle avoidance | nudges the turn (`va`) | nudges the strafe (`vy`) |

> **Camera mount geometry limits `ground_plane` mode.** The OAK-D sits
> 27.2 cm up and tilted **up** 7.21°, so nothing on the floor closer than
> **1.195 m** can be ranged at all. The standoff is raised to 1.6 m for
> that reason alone. `position_source: "depth"` removes the constraint
> entirely — see §4.

| Node | Original analog | Job |
|---|---|---|
| `pose_detector_node` | `tf-pose-estimation` | RGB → per-frame keypoints/bbox (YOLO-Pose) + depth back-projection |
| `person_tracker_node` | `monocular_people_tracking` (UKF) | ID association + constant-velocity Kalman filter + tf to `base_footprint` |
| `person_follower_node` | orchestrator + `ccf_person_identification` | SEARCH/TRACKING/LOST/REACQUIRE state machine, appearance re-id |
| `robot_controller_node` | `robot_controller.py` | target position → holonomic base velocity |
| `visualization_node` | `visualization.py` | annotated image + RViz markers |

## 1. Install

Run these **inside the `ros2_humble` container** (`./enter_ros.sh`).

### 1a. Pose model (Jetson-aware PyTorch)

`ultralytics` pulls a plain PyPI `torch` by default, which has no CUDA
support on Jetson's aarch64. Install the NVIDIA Jetson build of
`torch`/`torchvision` for your JetPack/L4T version *first* (check
`cat /etc/nv_tegra_release`), then:

```bash
pip3 install --user ultralytics
```

The node falls back to CPU if no CUDA-enabled torch is found — slower,
but it still runs.

### 1b. Known environment gotchas on this box

Installing `torch`/`ultralytics` here pulled in a much newer user-site
`numpy` (2.x) and `setuptools`, which collide with a couple of
ROS-provided pieces still built against older versions:

- **`colcon build --symlink-install` fails** (`setup.py develop
  --uninstall: option not recognized`) because the new `setuptools` no
  longer supports the legacy develop-uninstall path `colcon`'s
  symlink-install mode relies on. **Just use plain `colcon build`**
  (no `--symlink-install`) for this package.
- **`ament_cmake_python`'s egg-info step can fail** with
  `canonicalize_version() got an unexpected keyword argument
  'strip_trailing_zero'` — the new `setuptools` expects a newer
  `packaging` than the system one. Fix once with:
  `pip3 install --user --upgrade "packaging>=23"`.
- **`import cv_bridge` prints a scary NumPy ABI warning**
  (`A module that was compiled using NumPy 1.x cannot be run in NumPy
  2.2.6...` followed by an `AttributeError: _ARRAY_API not found`
  traceback-looking block) because `cv_bridge`'s compiled extension was
  built against the apt `numpy` (1.x) but user-site now has numpy 2.x.
  In testing this was non-fatal at import time, but if you hit real
  `AttributeError`s doing image conversion, pin numpy down:
  `pip3 install --user "numpy<2"` (recent `ultralytics` versions work
  fine with numpy 1.x too).

These are pre-existing conflicts between the apt-installed ROS
environment and pip's user-site packages, not specific to this package
— worth checking if they've drifted further by the time you read this.

## 2. Build

```bash
cd /workspace/ROS2-WorkSpace
colcon build --packages-select person_follower_msgs person_follower
source install/setup.bash
```

`person_follower_msgs` must be rebuilt too, not just `person_follower`:
`RobotDecision.msg` gained the strafe axis (`base_vy`, `final_vy`).
Rebuilding only the Python package leaves the old message definition
installed and the controller will fail at runtime setting `base_vy`.

`colcon test` doesn't reliably discover this package's pytest tests in
this environment (falls back to a no-op `setup.py test`). Run them
directly instead (from inside the package dir, so flake8 doesn't also
scan `build/`/`install/`):

```bash
cd /workspace/ROS2-WorkSpace/src/person_follower
python3 -m pytest test/
```

## 3. Run

If your OAK-D isn't already running as part of your robot's own bringup:

```bash
ros2 launch person_follower bringup_with_camera.launch.py
```

Otherwise, just the pipeline (adjust topics to match your actual camera
bringup — check with `ros2 topic list`):

```bash
ros2 launch person_follower person_follower.launch.py \
  image_topic:=/oak/rgb/image_raw \
  depth_topic:=/oak/stereo/image_raw \
  camera_info_topic:=/oak/rgb/camera_info \
  cmd_vel_topic:=/rover/base/velocity/set \
  scan_topic:=/scan
```

(these are the current defaults — override any of them if your setup
differs.) On this rover the usual full bringup is:

```bash
ros2 launch launch/perception_oakd.py          # camera + lidar + base
ros2 launch person_follower person_follower.launch.py enable_driving:=false
```

**The camera mount transform is published by this launch**, not by the
camera bringup. `launch/perception_oakd.py` starts the depthai driver
with `parent_frame:=oak-d-base-frame`, which builds the camera's internal
frame tree but never attaches it to the robot — so
`person_follower.launch.py` publishes `base_footprint → oak-d-base-frame`
itself. **The default offsets are a guess; measure them:**

```bash
ros2 launch person_follower person_follower.launch.py \
  camera_x:=0.12 camera_y:=0.0 camera_z:=0.68 camera_pitch:=0.06
```

(x/y/z in metres, angles in **radians**.) If this transform is missing or
wrong, the pipeline detects people but produces **no tracks at all** —
which looks like a detection failure and is not one. Check with:

```bash
ros2 run tf2_ros tf2_echo base_footprint oak_rgb_camera_optical_frame
```

`image_topic` should be the **raw** image_transport base topic (e.g.
`.../rgb/image_raw`), not `.../compressed`. By default
(`use_compressed_image:=true`) the pipeline remaps to
`<image_topic>/compressed` and decodes `sensor_msgs/CompressedImage`
(JPEG) itself — much lighter over the robot's WiFi link than raw frames.
Depth is never compressed (lossy JPEG is wrong for 16-bit depth values).
Pass `use_compressed_image:=false` to use raw images end-to-end instead
(e.g. if your camera bringup doesn't publish a `/compressed` topic —
check with `ros2 topic list`).

View `/visualization_node/annotated_image` (rqt_image_view or RViz with
`rviz/person_follower.rviz`) to see detections/state/track ids overlaid.

### Locking/dropping a target manually

```bash
# force-lock a specific track id (see the annotated image for ids)
ros2 service call /person_follower_node/imprint person_follower_msgs/srv/Imprint "{target_id: 3}"

# drop the current target, back to SEARCH
ros2 service call /person_follower_node/reset std_srvs/srv/Empty
```

### Using your own obstacle-avoiding local planner instead

`robot_controller_node` (the default) drives the base directly, with its own
LIDAR-based obstacle avoidance blended in while `TRACKING` (strafes around/slows
for anything within `obstacle_distance_threshold`, hard-stops forward motion
inside `obstacle_critical_distance` — see `config/person_follower.yaml`). If
you'd rather hand off to your own local planner/costmap-based obstacle
avoidance instead, pass `use_goal_publisher:=true`: `goal_publisher_node` runs
in place of `robot_controller_node` and publishes a `geometry_msgs/PoseStamped`
goal (in `base_footprint`, at the same "stand `distance` meters from the target,
facing them" point the velocity controller implicitly converges to) on
`goal_topic` — point that at whatever topic your planner subscribes to:

```bash
ros2 launch person_follower person_follower.launch.py \
  use_goal_publisher:=true \
  goal_topic:=/your_planner/goal
```

It only publishes while `person_follower_node`'s state is `TRACKING` — silent
during `SEARCH`/`LOST`/`REACQUIRE`.

### Recording a bag of a run

`record_bag.launch.py` wraps `ros2 bag record` (mcap storage) with the raw
sensor topics (forward camera, camera_info, scan, odom, tf, tf_static) plus
every stage of the pipeline's own output (detections → tracks → target state →
the controller's reasoning → the velocity commanded → the velocity the base
reported back on `/rover/base/velocity/get`) — enough to
reconstruct/describe a run afterwards without
re-deriving anything from raw video.

Those last three are deliberately all recorded, because they are three
different things: `velocity/set` is what `robot_controller_node` asked for,
`velocity/get` is `BaseDriver.cpp` acknowledging that it consumed a command
on its 10 ms cycle, and `/rover/base/odometry` is what the wheels then did.

`velocity/get` is worth being precise about: it republishes the received
`Twist` **verbatim**, so it is not a measured velocity and does not show
clamping or halts. What it adds is *timing and drops* — it is stamped when
the driver acted, and commands arriving faster than 10 ms collapse into one,
so a `/set` with no matching `/get` is a command the base never acted on.
For whether a commanded strafe actually happened, compare against odom. Run it in a separate terminal alongside
`person_follower.launch.py` (it doesn't launch the pipeline itself; topics
that don't exist yet are picked up automatically once they start publishing):

```bash
ros2 launch person_follower record_bag.launch.py
# writes to /workspace/ROS2-WorkSpace/bags/run_<timestamp>/ by default; override with:
ros2 launch person_follower record_bag.launch.py bag_dir:=~/bags bag_name:=my_run
```

The two side cameras (`side_cameras` package) are recorded into the same bag
— the compressed topics `/capture/camera_left/img/compressed` and
`/capture/camera_right/img/compressed`, so a run stays one `ros2 bag
play`-able unit with everything on one clock. Budget roughly 9 MB/s for the
pair at 720p30. There is no argument to turn them off: not running
`side_cameras` is the off switch, and naming a topic nothing publishes costs
nothing — rosbag2 records no messages on it and picks it up if it starts
mid-run.

`record_debug_visuals:=true` additionally records `visualization_node`'s
annotated image + RViz markers, into a separate `..._debug_visuals` bag (off
by default — both are derived from the topics already being recorded, so
they're redundant for reconstruction and just add size; useful if you
specifically want the rendered debug view saved too).

## 4. Tuning

Everything lives in `config/person_follower.yaml`. Notably:

- `pose_detector_node.position_source`: `"ground_plane"` (default) or
  `"depth"` -- how a detected person's keypoints become a 3D position.
  `ground_plane` ray-casts through the ankle keypoints onto the floor
  (no depth needed, runs at full RGB rate) -- this is what the original
  `monocular_person_following` did, and what this defaults to here since
  this robot's depth topic only runs ~1.2-1.6Hz and was bottlenecking
  the whole pipeline. `depth` back-projects real stereo depth at the hip
  keypoints instead -- more accurate, but only as fast as the depth
  topic. Both paths are fully implemented; switching is just this one
  parameter, no code or other node changes needed.
  - `camera_height`/`camera_pitch_deg` are **calibrated for this rover**
    (2026-08-25, `camera_ground_calibration.yaml`): 0.272 m high, tilted
    **up** 7.21°. Read the geometry block above them in
    `config/person_follower.yaml` before trusting distances — that run
    reported `status: FAILED`, and the mount constrains the mode badly
    (see below).
  - **Strongly consider `position_source: "depth"` instead.** Every
    ground_plane problem on this robot comes from the mount, not the
    camera: a 1.195 m blind zone, a standoff forced up to 1.6 m, 8%
    range error at 3 m from tilt noise alone, and an ignored 1.7° roll.
    `depth` uses the Lite's real stereo and needs none of those numbers.
    ground_plane was only chosen on the TurtleBot4 because *its* depth
    ran at 1.2–1.6 Hz. Check here first:
    ```bash
    ros2 topic hz /oak/stereo/image_raw
    ```
    If it's usable (>5 Hz), switch `position_source` to `"depth"` and put
    `distance` back to `1.0` and `imprint_max_distance` back to `2.0`.
- `person_follower_node`: `imprint_max_distance`/`imprint_max_angle_deg`/
  `imprint_hold_time` control auto-imprint in SEARCH; `reacquire_*`
  control how re-id behaves after a target is lost.
- `robot_controller_node`: `max_vx`/`max_vy`/`max_va`, the matching
  `gain_*`, and `distance` control driving behavior. **These are
  normalized throttle in [-1, 1], not m/s and rad/s** — see the UNITS
  banner at the top of that section. Do not copy values in from
  TurtleBot4 notes; `max_va: 1.5` there meant 1.5 rad/s, here it would
  mean 150% of full authority.
- `yaw_deadband_deg` (18°) is the knob that decides how much this robot
  turns versus strafes. Inside that half-angle it does not rotate at all.
  Raise it to strafe more and turn less; set it to `0.0` for continuous
  centering; set it above `max_bearing_deg` to never turn while
  `TRACKING`.
- `obstacle_side_clearance` (0.35 m) and `scan_min_range` (0.25 m) are
  **safety guards, not tuning knobs** — see
  [Omnidirectional control](#omnidirectional-control).

## Omnidirectional control

The original controller turned to face the target and only drove forward
once roughly aligned. This base can translate in any direction
regardless of which way it is pointing, so:

- **Translation is holonomic.** `_drive_toward_target` drives straight at
  the standoff point using `vx` and `vy` together, in one step, with no
  alignment gate. A target off to one side produces a diagonal move, not
  a turn-then-drive sequence.
- **Yaw only keeps the person in frame.** The camera is body-mounted with
  a ~69° horizontal FOV, so the robot still has to face roughly towards
  the target to see them — but that is the *only* remaining reason to
  rotate while following. Inside `yaw_deadband_deg` it does not rotate at
  all.
- **Obstacle avoidance strafes instead of steering.** The same
  continuous, smoothed `[-1, 1]` bias now lands on `vy` rather than `va`.

That last point fixed a real bug rather than just porting one. On the
TurtleBot4, avoidance and target-framing both wrote to `va`: turning away
from an obstacle swung the target's apparent bearing, centering
immediately turned back to cancel that swing, and the two could null each
other out entirely ("avoiding left, but the robot turns right instead to
re-center the person"). `obstacle_frame_deadband_deg` existed purely to
stop them fighting. Here they are on orthogonal axes and cannot cancel.

**Two guards exist here that had no TurtleBot4 equivalent:**

- `obstacle_side_clearance` — the forward trigger cone says nothing about
  the robot's flanks. That was fine for a robot that could only move
  along its own x axis; this one strafes as a matter of course, so its
  sides are now a real collision surface. Strafing into a side closer
  than this is refused outright. Checked against the *closest* return in
  the side zone, not the average (an average stays comfortable while one
  table leg sits centimetres from the wheels). The RPLIDAR A1 is a full
  360° scanner, so these are real measurements.
- `scan_min_range` — **this rover's lidar sees its own arm.**
  `config/scan_filter.yaml` measured it: 3.9% of valid beams return under
  0.35 m, the farthest at 0.177 m. `amcl_params.yaml` drops them with
  `laser_min_range: 0.25` and the rf2o filter with the same 0.25, but
  this node reads the *raw* `/scan`, so it needs its own floor. Without
  it those returns — which never change with motion, being bolted to the
  robot — pin forward speed at zero and refuse strafe to one side
  permanently, with nothing actually in the way.

## 5. Before driving

This pipeline has **never been run on this rover** — it was ported from a
TurtleBot4, and the control law, the units, the topics and the frames all
changed. Treat the first run as bring-up, not as a regression test.

1. **Measure `camera_x` and `camera_y`** — how far forward and sideways
   the camera sits on the robot. The 2026-08-25 ground calibration solved
   height, tilt and roll (already applied to both the config and the
   mount transform), but it cannot see those two, so they are still
   `0.0`. Everything else in §3's transform is calibrated.
2. **Run with `enable_driving:=false` first.** No node publishes a
   velocity at all in that mode, so the robot physically cannot move.
   Confirm `ros2 topic echo /person_follower_node/target` shows sane
   `state`/`target_id` transitions as you step in and out of frame, and
   that reported distances match a tape measure at 2 m *and* 4 m
   (ground-plane error grows with distance).
3. **Wheels off the ground for the first driving run.** Then check
   `ros2 topic echo /rover/base/velocity/set` against where you actually
   are:
   - stand to the robot's **left** → expect **positive `linear.y`**
     (strafe left), and `angular.z` near zero while you are within
     `yaw_deadband_deg`
   - stand **far** → positive `linear.x`; stand **closer than
     `distance`** → negative `linear.x`
   - a sign error on `linear.y` is the specific new failure mode here and
     it is not subtle: the robot strafes *away* from the person.
4. `ros2 topic echo /robot_controller_node/decision` shows the full
   reasoning behind every command — `base_v*` before the obstacle nudge,
   `final_v*` after, plus what the lidar saw. Use it rather than guessing
   when behaviour looks wrong.
5. The base has its own 2 s failsafe (`BaseDriver`'s `vel_timeout`) that
   halts the motors if it stops hearing commands. That is a backstop, not
   a stop button — keep the physical kill within reach.

## Not ported from the original

- No Unscented Kalman Filter / ground-plane height estimation — OAK-D
  depth gives 3D position directly, so this uses a much simpler
  constant-velocity KF (`kalman.py`).
- The side cameras (`/capture/camera_left/img`, `.../camera_right/img`)
  are **not** used for sideways safety. They are uncalibrated monocular
  streams with no `camera_info` and no depth, so they cannot produce a
  metric clearance distance — while the RPLIDAR A1 is a 360° scanner that
  already covers both flanks with real range data. `obstacle_side_clearance`
  uses the lidar for that reason. The cameras would be useful for
  *detecting* something the lidar's single scan plane misses (an
  overhang, a low step), which is a different job than this guard does.
  They *are* recorded into a bag (see [Recording a bag of a
  run](#recording-a-bag-of-a-run)) — being useless as a live safety input
  is not the same as being useless as a record of what happened.
- No trained CCF/online-boosting re-id classifier (`dlib`+`flann`) — re-id
  during REACQUIRE is an HSV color-histogram comparison (`reid.py`).
  Weaker than the original in cluttered scenes with similarly-dressed
  people, but dependency-free and good enough for a single-target
  following robot in a small space.
