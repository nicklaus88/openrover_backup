"""pose_detector_node: RGB(+depth) -> PersonDetectionArray.

The ROS2 analog of tf-pose-estimation in the original
monocular_person_following stack, except real OpenPose is dead weight
here: we use Ultralytics YOLO-Pose instead, which is pip-installable and
actively maintained. This node is deliberately stateless (no IDs, no
smoothing) -- that's person_tracker_node's job.

Two interchangeable ways to turn a detected person's keypoints into a 3D
position, selected by the `position_source` parameter:

  "depth" (original design here): back-project the depth around the
      person's hip keypoints through the camera intrinsics. Accurate and
      simple, but only as fast as the depth stream -- on this robot the
      OAK-D's stereo depth topic runs at only ~1-1.5Hz (vs ~8Hz RGB),
      which bottlenecks the whole pipeline down to depth's rate.

  "ground_plane" (default, matches the original *koide3*
      monocular_person_following's approach): assumes the person's feet
      are on the ground and the camera's height/tilt above that ground
      plane are known, then ray-casts through the ankle keypoints to
      find where that ray hits the ground -- no depth needed at all, so
      it runs at full RGB rate. Needs `camera_height`/`camera_pitch_deg`
      calibrated correctly (see those params below) or distance
      estimates will be systematically off; accuracy also degrades for
      people far from the camera (shallow ray angle).

Both paths are fully implemented and switching is just a parameter flip
(no other node needs to change) -- this exists so "ground_plane" can be
used now while the depth-rate bottleneck gets sorted out, and "depth"
switched back on later without losing any code.
"""
import math

import cv_bridge
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from person_follower_msgs.msg import PersonDetection, PersonDetectionArray

# COCO-17 keypoint indices used elsewhere in this package.
L_HIP = 11
R_HIP = 12
L_ANKLE = 15
R_ANKLE = 16


class PoseDetectorNode(Node):

    def __init__(self):
        super().__init__('pose_detector_node')

        self.declare_parameter('model_path', 'yolo11n-pose.pt')
        self.declare_parameter('device', 'auto')  # 'auto' | 'cpu' | 'cuda:0'
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('keypoint_confidence_threshold', 0.3)
        # "depth" or "ground_plane" -- see module docstring.
        self.declare_parameter('position_source', 'ground_plane')
        # multiply raw depth by this to get meters (uint16 mm -> m)
        self.declare_parameter('depth_scale', 0.001)
        # pixels, half-size of the depth sampling patch
        self.declare_parameter('depth_patch_radius', 2)
        self.declare_parameter('sync_slop', 0.5)  # seconds, image/depth sync tolerance
        # ground_plane params: how high the camera is mounted above the
        # floor, and how far it's tilted downward from horizontal. Get
        # these as exactly as you can -- ground-plane distance estimates
        # are very sensitive to tilt error, especially for people far
        # away (the projection ray grazes the ground at a shallow angle).
        # Best source: once base_footprint<->camera tf is working, read
        # the real values with `ros2 run tf2_ros tf2_echo base_footprint
        # oak_rgb_camera_optical_frame` instead of guessing/hand-measuring
        # -- but note that only tells you what the STATIC MOUNT TRANSFORM
        # claims, and that transform is itself hand-entered in
        # person_follower.launch.py, so it is a restatement of the tape
        # measure rather than a check on it. For numbers that are
        # actually measured, run the aprilgrid_ground_calibrator package
        # in this workspace against a fiducial on the floor: it solves
        # height/tilt/roll optically.
        self.declare_parameter('camera_height', 0.20)
        self.declare_parameter('camera_pitch_deg', 0.0)
        # RGB image arrives as sensor_msgs/CompressedImage (JPEG) instead of
        # raw sensor_msgs/Image -- much lighter over a WiFi link to the
        # robot. Depth (when position_source=="depth") always stays raw
        # (lossy JPEG is wrong for 16-bit depth values, and
        # compressed_depth_image_transport needs its own decoder
        # cv_bridge doesn't provide).
        self.declare_parameter('use_compressed_image', True)
        # Multiplies fx and fy from camera_info. 1.0 (correct camera_info)
        # for every stream except depthai's "preview".
        #
        # depthai publishes a WRONG focal length for the preview: it
        # scales f by preview_width/ISP_width, but with
        # i_keep_preview_aspect_ratio true the ISP frame is centre-cropped
        # to square FIRST, so the real scale is preview_size/ISP_height.
        # Measured on this camera 2026-08-26, 250 square from 1280x720:
        # /oak/rgb/preview/camera_info reports fx=fy=196.603 where the
        # true value is 349.52. Out by ISP_width/ISP_height = 16/9.
        #
        # So the correction is the ISP's ASPECT RATIO, 1.7777778 here.
        # Recompute it if you change rgb.i_width/i_height in
        # rover_bringup/config/oakd.yaml.
        #
        # cx/cy are NOT touched -- the driver already transforms the
        # principal point correctly for the crop; only f is wrong.
        #
        # Set back to 1.0 if you point camera_info_topic at the full-size
        # /oak/rgb/camera_info. Getting this pairing wrong is what makes
        # the rover drive backwards: too-small fy makes ground_plane
        # ranging read everything as closer than it is, inside
        # `distance`, so the controller reverses.
        self.declare_parameter('focal_length_scale', 1.0)

        model_path = self.get_parameter('model_path').value
        device = self.get_parameter('device').value
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.keypoint_confidence_threshold = float(
            self.get_parameter('keypoint_confidence_threshold').value)
        self.position_source = self.get_parameter('position_source').value
        if self.position_source not in ('depth', 'ground_plane'):
            raise ValueError(
                f"position_source must be 'depth' or 'ground_plane', got "
                f"{self.position_source!r}")
        self.depth_scale = float(self.get_parameter('depth_scale').value)
        self.depth_patch_radius = int(self.get_parameter('depth_patch_radius').value)
        self.camera_height = float(self.get_parameter('camera_height').value)
        self.camera_pitch = math.radians(float(self.get_parameter('camera_pitch_deg').value))
        self.use_compressed_image = bool(self.get_parameter('use_compressed_image').value)
        self.focal_length_scale = float(
            self.get_parameter('focal_length_scale').value)
        if self.focal_length_scale <= 0.0:
            raise ValueError(
                f'focal_length_scale must be > 0, got '
                f'{self.focal_length_scale}')

        self.bridge = cv_bridge.CvBridge()
        self.camera_info = None
        self._warned_intrinsics_scale = False

        self.get_logger().info(f'loading pose model {model_path} (device={device})')
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "ultralytics is not installed. Run 'pip install ultralytics' "
                '(see this package README for the Jetson-specific torch install steps) '
                'before starting pose_detector_node.'
            ) from exc

        self.model = YOLO(model_path)
        self.device = None if device == 'auto' else device

        self.detections_pub = self.create_publisher(PersonDetectionArray, '~/detections', 1)

        self.camera_info_sub = self.create_subscription(
            CameraInfo, '~/camera_info', self._camera_info_callback, qos_profile_sensor_data)

        image_type = CompressedImage if self.use_compressed_image else Image
        self._image_count = 0
        self._sync_count = 0

        if self.position_source == 'depth':
            image_sub = message_filters.Subscriber(
                self, image_type, '~/image', qos_profile=qos_profile_sensor_data)
            depth_sub = message_filters.Subscriber(
                self, Image, '~/depth', qos_profile=qos_profile_sensor_data)
            sync_slop = float(self.get_parameter('sync_slop').value)
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [image_sub, depth_sub], queue_size=10, slop=sync_slop)
            self.sync.registerCallback(self._callback_depth)

            # --- debug instrumentation ----------------------------------
            # message_filters.Subscriber.registerCallback appends a
            # callback rather than replacing the one the synchronizer
            # already registered, so this doesn't interfere with syncing
            # -- it just lets us see raw per-topic arrivals independently
            # of whether a synced pair ever fires (the two most common
            # "stuck" causes are different: no messages on one of the
            # topics at all, vs. messages on both but never landing
            # within sync_slop of each other).
            self._depth_count = 0
            image_sub.registerCallback(self._on_image_received)
            depth_sub.registerCallback(self._on_depth_received)
        else:
            # No depth needed at all -- plain subscription, runs at full
            # RGB rate, nothing to time-sync.
            self.image_sub = self.create_subscription(
                image_type, '~/image', self._callback_ground_plane, qos_profile_sensor_data)

        image_topic = self.resolve_topic_name('~/image')
        camera_info_topic = self.resolve_topic_name('~/camera_info')
        info_lines = [
            f'pose_detector_node ready. position_source={self.position_source!r} '
            f'use_compressed_image={self.use_compressed_image}',
            f'  image topic:       {image_topic}',
            f'  camera_info topic: {camera_info_topic}',
        ]
        if self.focal_length_scale != 1.0:
            info_lines.append(
                f'  focal_length_scale: {self.focal_length_scale} -- fx/fy from '
                f'camera_info are being MULTIPLIED by this. Correct only if '
                f'camera_info topic above is a depthai preview stream; set it '
                f'to 1.0 for a full-size stream.')
        if self.position_source == 'depth':
            info_lines.append(f"  depth topic:       {self.resolve_topic_name('~/depth')}")
        else:
            info_lines.append(
                f'  ground plane: camera_height={self.camera_height}m '
                f'camera_pitch={math.degrees(self.camera_pitch)}deg '
                f'(WARNING: uncalibrated values here silently give wrong '
                f'distances -- see module docstring)')
        info_lines.append('waiting for data on all inputs...')
        self.get_logger().info('\n'.join(info_lines))
        self._startup_watchdog = self.create_timer(5.0, self._startup_watchdog_cb)

    def _startup_watchdog_cb(self):
        have_depth = self.position_source != 'depth' or self._depth_count
        if self._image_count and have_depth and self.camera_info is not None:
            msg = f'all inputs flowing (image={self._image_count}'
            if self.position_source == 'depth':
                msg += f', depth={self._depth_count}'
            msg += ' msgs so far) -- stopping startup watchdog.'
            self.get_logger().info(msg)
            self._startup_watchdog.cancel()
            return

        missing = []
        if not self._image_count:
            missing.append(f"image ('{self.resolve_topic_name('~/image')}')")
        if self.position_source == 'depth' and not self._depth_count:
            missing.append(f"depth ('{self.resolve_topic_name('~/depth')}')")
        if self.camera_info is None:
            missing.append(f"camera_info ('{self.resolve_topic_name('~/camera_info')}')")
        self.get_logger().warn(
            'still waiting for: ' + ', '.join(missing)
            + '. Check `ros2 topic hz <topic>` and `ros2 topic info <topic> --verbose` '
              '(QoS mismatches are silent) for each missing one.')

    def _on_image_received(self, msg):
        self._image_count += 1
        if self._image_count == 1:
            self.get_logger().info(
                f"first RGB image received on '{self.resolve_topic_name('~/image')}' "
                f'(stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d})')

    def _on_depth_received(self, msg):
        self._depth_count += 1
        if self._depth_count == 1:
            self.get_logger().info(
                f"first depth image received on '{self.resolve_topic_name('~/depth')}' "
                f'(stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d})')

    def _camera_info_callback(self, msg):
        first = self.camera_info is None
        self.camera_info = msg
        if first:
            self.get_logger().info(
                f"first camera_info received on '{self.resolve_topic_name('~/camera_info')}'")

    def _decode_image(self, image_msg):
        if self.use_compressed_image:
            return self.bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        return self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

    def _callback_ground_plane(self, image_msg):
        self._image_count += 1
        if self._image_count == 1:
            self.get_logger().info(
                f"first RGB image received on '{self.resolve_topic_name('~/image')}'")

        if self.camera_info is None:
            self.get_logger().warn(
                'no camera_info received yet, skipping frame', throttle_duration_sec=5.0)
            return

        bgr = self._decode_image(image_msg)
        fx, fy, cx, cy = self._intrinsics(bgr)

        def backproject(kx, ky, ks, bbox_x, bbox_y, bbox_w, bbox_h):
            return self._backproject_ground_plane(
                kx, ky, ks, bbox_x, bbox_y, bbox_w, bbox_h, fx, fy, cx, cy)

        self._process_frame(bgr, image_msg.header, backproject)

    def _callback_depth(self, image_msg, depth_msg):
        self._sync_count += 1
        if self._sync_count == 1:
            self.get_logger().info(
                'first synced image+depth pair matched within sync_slop -- '
                'running pose detection now.')

        if self.camera_info is None:
            self.get_logger().warn(
                'no camera_info received yet, skipping frame', throttle_duration_sec=5.0)
            return

        bgr = self._decode_image(image_msg)
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        fx, fy, cx, cy = self._intrinsics(bgr)
        depth_scale_x = depth.shape[1] / bgr.shape[1]
        depth_scale_y = depth.shape[0] / bgr.shape[0]

        def backproject(kx, ky, ks, bbox_x, bbox_y, bbox_w, bbox_h):
            return self._backproject_depth(kx, ky, ks, bbox_x, bbox_y, bbox_w, bbox_h,
                                           depth, depth_scale_x, depth_scale_y, fx, fy, cx, cy)

        self._process_frame(bgr, image_msg.header, backproject)

    def _process_frame(self, bgr, header, backproject_fn):
        results = self.model.predict(
            bgr, conf=self.confidence_threshold, device=self.device, verbose=False)[0]

        out = PersonDetectionArray()
        out.header = header

        if results.keypoints is None or results.boxes is None:
            self.detections_pub.publish(out)
            return

        boxes_xyxy = results.boxes.xyxy.cpu().numpy()
        boxes_conf = results.boxes.conf.cpu().numpy()
        kpts_xy = results.keypoints.xy.cpu().numpy()
        kpts_conf = (
            results.keypoints.conf.cpu().numpy()
            if results.keypoints.conf is not None
            else np.ones(kpts_xy.shape[:2], dtype=np.float32)
        )

        for i in range(boxes_xyxy.shape[0]):
            det = PersonDetection()
            det.score = float(boxes_conf[i])

            x1, y1, x2, y2 = boxes_xyxy[i]
            det.bbox_x = float(x1)
            det.bbox_y = float(y1)
            det.bbox_w = float(x2 - x1)
            det.bbox_h = float(y2 - y1)

            n = min(17, kpts_xy.shape[1])
            kx = [0.0] * 17
            ky = [0.0] * 17
            ks = [0.0] * 17
            for j in range(n):
                kx[j] = float(kpts_xy[i, j, 0])
                ky[j] = float(kpts_xy[i, j, 1])
                ks[j] = float(kpts_conf[i, j])
            det.keypoints_x = kx
            det.keypoints_y = ky
            det.keypoints_score = ks

            position = backproject_fn(kx, ky, ks, det.bbox_x, det.bbox_y, det.bbox_w, det.bbox_h)
            det.position.x, det.position.y, det.position.z = position

            out.detections.append(det)

        self.detections_pub.publish(out)

    def _intrinsics(self, bgr):
        """camera_info's K, rescaled to the resolution of the frame the
        keypoints were actually measured in.

        The two can disagree. depthai_ros_driver builds camera_info from
        its rgb.i_width/i_height parameters, but the ISP does not always
        honour them: setting 640x360 on this OAK-D Lite left the frames
        on the wire at 1280x720 while camera_info dropped to 640x360
        (bags run_20260825_213059 onwards -- 212608, half an hour
        earlier, still had 1280x720 in both). Nothing warns about it;
        the only symptom is arithmetic.

        It is a big deal here specifically because ground_plane ranges a
        person off the ankle pixel ROW. Half-size cy/fy against full-size
        ankle pixels put every range at ~0.55 m regardless of where the
        person really was, which is inside `distance` (1.6 m) forever, so
        robot_controller_node drove full-speed REVERSE for an entire run
        (215 backward commands, 0 forward) with a target it could see
        perfectly well.

        Scaling makes either resolution correct. The depth path already
        does the same thing for depth-vs-RGB size (depth_scale_x/y).
        """
        k = self.camera_info.k
        info_w, info_h = self.camera_info.width, self.camera_info.height
        if not info_w or not info_h:
            return k[0], k[4], k[2], k[5]
        sx = bgr.shape[1] / float(info_w)
        sy = bgr.shape[0] / float(info_h)
        if (sx != 1.0 or sy != 1.0) and not self._warned_intrinsics_scale:
            self._warned_intrinsics_scale = True
            self.get_logger().warn(
                f'camera_info is {info_w}x{info_h} but images are '
                f'{bgr.shape[1]}x{bgr.shape[0]} -- scaling intrinsics by '
                f'({sx:.3f}, {sy:.3f}). Distances would be wrong without this; '
                f'better to fix the camera driver so the two agree.')
        # focal_length_scale corrects a camera_info whose K is wrong for
        # its own declared size -- which the size-based sx/sy rescale
        # above cannot detect. See the parameter's declaration.
        return (k[0] * sx * self.focal_length_scale,
                k[4] * sy * self.focal_length_scale,
                k[2] * sx,
                k[5] * sy)

    def _backproject_depth(self, kx, ky, ks, bbox_x, bbox_y, bbox_w, bbox_h,
                           depth, depth_scale_x, depth_scale_y, fx, fy, cx, cy):
        hip_ok = (ks[L_HIP] > self.keypoint_confidence_threshold
                  and ks[R_HIP] > self.keypoint_confidence_threshold)
        if hip_ok:
            u = 0.5 * (kx[L_HIP] + kx[R_HIP])
            v = 0.5 * (ky[L_HIP] + ky[R_HIP])
        else:
            u = bbox_x + bbox_w / 2.0
            v = bbox_y + bbox_h / 2.0

        du = int(round(u * depth_scale_x))
        dv = int(round(v * depth_scale_y))
        r = self.depth_patch_radius
        h, w = depth.shape[:2]
        u0, u1 = max(0, du - r), min(w, du + r + 1)
        v0, v1 = max(0, dv - r), min(h, dv + r + 1)
        if u1 <= u0 or v1 <= v0:
            return (float('nan'), float('nan'), float('nan'))

        patch = depth[v0:v1, u0:u1].astype(np.float32).flatten()
        patch = patch[np.isfinite(patch) & (patch > 0)]
        if patch.size == 0:
            return (float('nan'), float('nan'), float('nan'))

        z = float(np.median(patch)) * self.depth_scale
        if depth.dtype != np.uint16:
            # float depth images from depthai are already in meters.
            z = float(np.median(patch))

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return (x, y, z)

    def _backproject_ground_plane(self, kx, ky, ks, bbox_x, bbox_y, bbox_w, bbox_h,
                                  fx, fy, cx, cy):
        """Ray-cast through the ankle keypoints (feet) and find where that
        ray intersects the ground plane, using known camera height/tilt.
        Returns the intersection point in the camera optical frame (same
        convention as _backproject_depth's output), so downstream code
        (person_tracker_node's tf transform to base_footprint etc.) doesn't
        need to know or care which method produced it.

        Math: in the camera optical frame (X=right, Y=down, Z=forward),
        a ray through pixel (u,v) has direction (dx, dy, 1) with
        dx=(u-cx)/fx, dy=(v-cy)/fy. Tilting the camera down by pitch
        theta from horizontal, that ray's height-drop-per-unit-forward-
        travel works out to (cos(theta)*dy + sin(theta)) [see
        module/derivation in the design doc -- this is the standard
        pinhole+ground-plane-intersection result]. Given the camera
        sits at height H above the ground, the ray hits the ground
        (world Z=0) at scale factor t = H / (cos(theta)*dy + sin(theta)),
        and the camera-frame intersection point is simply t*(dx, dy, 1).
        """
        ankle_ok = (ks[L_ANKLE] > self.keypoint_confidence_threshold
                    and ks[R_ANKLE] > self.keypoint_confidence_threshold)
        if ankle_ok:
            u = 0.5 * (kx[L_ANKLE] + kx[R_ANKLE])
            v = 0.5 * (ky[L_ANKLE] + ky[R_ANKLE])
        elif ks[L_ANKLE] > self.keypoint_confidence_threshold:
            u, v = kx[L_ANKLE], ky[L_ANKLE]
        elif ks[R_ANKLE] > self.keypoint_confidence_threshold:
            u, v = kx[R_ANKLE], ky[R_ANKLE]
        else:
            # Fall back to the bottom-center of the bounding box -- a
            # decent stand-in for "where the feet are" when ankles
            # weren't detected confidently (e.g. legs out of frame).
            u = bbox_x + bbox_w / 2.0
            v = bbox_y + bbox_h

        dx = (u - cx) / fx
        dy = (v - cy) / fy

        denom = math.cos(self.camera_pitch) * dy + math.sin(self.camera_pitch)
        if denom <= 1e-6:
            # Ray points at or above the horizon -- doesn't hit the
            # ground in front of the camera (bad keypoint, or camera
            # tilt too shallow for this pixel row).
            return (float('nan'), float('nan'), float('nan'))

        t = self.camera_height / denom
        return (t * dx, t * dy, t)


def main(args=None):
    rclpy.init(args=args)
    node = PoseDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
