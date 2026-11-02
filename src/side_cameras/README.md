# side_cameras

Left and right BlueRobotics USB cameras (`H264 USB Camera`, USB ID
`32e4:9422`) as ROS 2 topics.

One node instance per camera, so either side can be restarted or retuned
without disturbing the other:

| Side  | Raw                          | Compressed                              |
|-------|------------------------------|-----------------------------------------|
| left  | `/capture/camera_left/img`   | `/capture/camera_left/img/compressed`   |
| right | `/capture/camera_right/img`  | `/capture/camera_right/img/compressed`  |

The naming follows the rover's existing `/capture/camera_0/img` contract,
which `oakd_camera` publishes for the forward-facing OAK-D.

## Running

```bash
ros2 launch side_cameras side_cameras.launch.py   # cameras only
ros2 launch launch/perception_oakd.py             # full stack
```

## Which camera is which side

Both cameras are the same model **and report the same USB serial**
(`2020032801`), so `/dev/v4l/by-id/` cannot distinguish them, and
`/dev/videoN` numbering can swap between boots. The USB port is the only
stable discriminator, so the defaults are `/dev/v4l/by-path/` symlinks:

```
port 2.2.1 -> /dev/video0..3      (only index0 captures)
port 2.2.3 -> /dev/video4..7      (only index0 captures)
```

**Which port is physically on which side is not knowable from software** --
verify it once:

```bash
ros2 run rqt_image_view rqt_image_view /capture/camera_left/img
```

If the views are swapped, exchange the two `device:` paths in
`config/side_cameras.yaml`, or override them per-launch:

```bash
ros2 launch launch/perception_oakd.py \
    left_camera_device:=/dev/v4l/by-path/platform-3610000.usb-usb-0:2.2.3:1.0-video-index0 \
    right_camera_device:=/dev/v4l/by-path/platform-3610000.usb-usb-0:2.2.1:1.0-video-index0
```

## Notes

* **MJPG, not YUYV.** Uncompressed YUYV is bandwidth-capped on this USB 2.0
  bus: asking for 720p silently yields 800x600@16fps. MJPG was measured at
  1920x1080@33fps, so the 720p default has headroom for both cameras at once.
* **`cv_bridge.cv2_to_imgmsg()` is unusable in this container** -- it raises
  `KeyError(16)` (pip's opencv-python numbers `CV_8UC3` as 64, the compiled
  cv_bridge extension expects 16). The node fills `sensor_msgs/Image` by
  hand instead; same trap is documented in `oakd_camera` and `person_follower`.
* **RELIABLE QoS by default**, matching the rest of the rover's camera
  topics -- a BEST_EFFORT publisher will not match the depth-10 RELIABLE
  subscriptions used by `camera_detection/perception`, `ros2 topic echo`
  and `rqt_image_view`.
