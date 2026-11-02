# oakd_camera

OAK-D Lite camera source for the rover. Starts `depthai_ros_driver` and
republishes the colour stream on **two** topics:

| Topic | Type | Notes |
|---|---|---|
| `/capture/camera_0/img` | `sensor_msgs/Image` | `bgr8`, 1280x720, ~2.76 MB/frame |
| `/capture/camera_0/img/compressed` | `sensor_msgs/CompressedImage` | JPEG q80, ~50 KB/frame |

## Why that topic name

`launch/perception.py` remaps `camera_detection/perception`'s `img` input
to `/capture/camera_0/img`, which was meant to be published by the
`spinnaker_camera` "capture" node. That submodule is **empty** in this
checkout — `src/spinnaker_camera` has no files, because `.gitmodules`
points it at an SSH `git@github.com:uf-focus-lab/Spinnaker-Viewer.git`
URL — so nothing was feeding perception.

Publishing the same topic means the OAK-D is a drop-in replacement and
`perception.py` needs no edits.

## Running

Camera only:

```bash
cd ROS2-WorkSpace
ros2 launch launch/oakd.py
```

Full perception pipeline (rover base + camera + perception + operation):

```bash
ros2 launch launch/perception_oakd.py
```

`perception_oakd.py` is a copy of `perception.py` with the Spinnaker node
swapped for this one; the original is left untouched so it still works if
the submodule is ever checked out.

Do **not** start `depthai_ros_driver` yourself — both launch files start
it. Two drivers means `X_LINK_DEVICE_ALREADY_IN_USE`; only one process
can hold the camera.

## Build

```bash
cd ROS2-WorkSpace
colcon build --packages-select oakd_camera
source install/setup.bash
```

`make package/oakd_camera` also works, but it runs `rosdep install` first
and needs network.

## Host prerequisite: the udev rule

Already applied on this Jetson (`/etc/udev/rules.d/80-movidius.rules`).
On a fresh machine the camera fails as a non-root user with:

```
[depthai] [warning] Insufficient permissions to communicate with
                    X_LINK_UNBOOTED device ... Make sure udev rules are set
[ERROR] [oak]: Failed to boot device!
```

Fix on the **host** (the container only bind-mounts `/dev`):

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | \
    sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then replug the camera. Check with `lsusb`: `03e7:2485` is unbooted,
`03e7:f63b` means a driver has it.

## Seeing the topics

`compose.yaml` puts the whole container in Fast DDS **discovery-server**
mode (`ROS_DISCOVERY_SERVER`). If `ros2 topic list` is empty while the
launch is clearly printing "frames relayed", the local discovery server
is not running — start it with `./start_discovery_server.sh` after every
container restart.

## Configuration

`config/oakd_lite.yaml` — driver settings under `/oak:`, this node's under
`/oakd_rgb_publisher_node:`.

Three things verified against the driver binary on this camera:

- **`pipeline_gen.i_enable_imu`, not `camera.i_enable_imu`.** The Lite has
  no IMU. The `IMU enabled but not available!` check lives in
  `pipeline_generator.cpp`, which reads the `pipeline_gen` namespace;
  `camera.i_enable_imu` is accepted and then ignored.
- **`i_width`/`i_height` set the published size, not `i_resolution`.**
  `i_resolution` picks the sensor mode; the ISP output is sized by
  `i_width`/`i_height` (default 1280x720).
- **There is no `i_usb_speed` parameter** in this build, despite what some
  upstream docs show.

## Note on cv_bridge

Only cv_bridge's **decode** direction works in this container;
`cv2_to_imgmsg()` raises `KeyError(16)` because pip's opencv-python
numbers `CV_8UC3` as 64 while the compiled extension returns 16. The node
uses `imgmsg_to_cv2()` + `cv2.imencode()` and passes the raw message
through untouched, so it never hits that path.

`camera_detection/perception` uses `imgmsg_to_cv2()` (decode), so it is
unaffected.
