"""person_tracker_node: PersonDetectionArray -> TrackedPersonArray.

The ROS2 analog of monocular_people_tracking: turns per-frame, ID-less
detections into persistent tracks. Where the original used an Unscented
Kalman Filter with a ground-plane observation model (because it had no
real depth), this uses a much simpler constant-velocity Kalman filter
(kalman.py) directly on the 3D position we already get from OAK-D depth,
plus a greedy nearest-neighbor data association.

Also publishes each track's position in the robot's base frame (via
tf2) so that
downstream nodes don't need to do their own tf lookups.
"""
import math

import rclpy
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped conversions)
import tf2_ros
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

from person_follower.kalman import ConstantVelocityKalmanFilter
from person_follower_msgs.msg import PersonDetectionArray, TrackedPerson, TrackedPersonArray


def _to_float_list(values):
    # rosidl represents fixed-size float32[N] fields as numpy arrays; a
    # bare list(values) would keep numpy.float32 elements, which fail the
    # generated message setters' isinstance(x, float) check when
    # reassigned to another message's field.
    return [float(v) for v in values]


class Track:
    def __init__(self, track_id, detection, stamp):
        self.id = track_id
        self.kf = ConstantVelocityKalmanFilter(
            initial_position=[detection.position.x, detection.position.y, detection.position.z])
        self.keypoints_x = _to_float_list(detection.keypoints_x)
        self.keypoints_y = _to_float_list(detection.keypoints_y)
        self.keypoints_score = _to_float_list(detection.keypoints_score)
        self.bbox = (detection.bbox_x, detection.bbox_y, detection.bbox_w, detection.bbox_h)
        self.score = detection.score
        self.missed_frames = 0
        self.age = 0
        self.last_stamp = stamp
        self.has_valid_position = _is_finite(detection.position)

    def bbox_center(self):
        x, y, w, h = self.bbox
        return (x + w / 2.0, y + h / 2.0)


def _is_finite(point):
    return math.isfinite(point.x) and math.isfinite(point.y) and math.isfinite(point.z)


def _stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class PersonTrackerNode(Node):

    def __init__(self):
        super().__init__('person_tracker_node')

        self.declare_parameter('max_missed_frames', 15)
        self.declare_parameter('association_max_distance_m', 1.0)
        self.declare_parameter('association_max_pixel_distance', 150.0)
        # base_footprint, not base_link: this rover's tf tree has no
        # base_link node at all (see launch/localization.py, which makes
        # the same correction to rf2o's upstream default). The
        # TrackedPerson field this fills is still called
        # position_base_link -- that is a message field name kept for
        # compatibility, not a claim about which frame it is in; it holds
        # whatever base_frame resolves to.
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('process_noise', 0.05)
        self.declare_parameter('measurement_noise', 0.05)

        self.max_missed_frames = int(self.get_parameter('max_missed_frames').value)
        self.association_max_distance_m = float(
            self.get_parameter('association_max_distance_m').value)
        self.association_max_pixel_distance = float(
            self.get_parameter('association_max_pixel_distance').value)
        self.base_frame = self.get_parameter('base_frame').value
        self.process_noise = float(self.get_parameter('process_noise').value)
        self.measurement_noise = float(self.get_parameter('measurement_noise').value)

        self.tracks = {}
        self.next_id = 0
        self.last_stamp_sec = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.tracks_pub = self.create_publisher(TrackedPersonArray, '~/tracks', 1)
        self.detections_sub = self.create_subscription(
            PersonDetectionArray, '~/detections', self._callback, 10)

        # debug instrumentation -- see pose_detector_node.py for rationale
        self._detections_count = 0
        self._startup_watchdog = self.create_timer(5.0, self._startup_watchdog_cb)

        detections_topic = self.resolve_topic_name('~/detections')
        self.get_logger().info(f'person_tracker_node ready, detections topic: {detections_topic}')

    def _startup_watchdog_cb(self):
        if self._detections_count:
            self.get_logger().info(
                f'detections flowing ({self._detections_count} msgs so far) -- '
                f'stopping startup watchdog.')
            self._startup_watchdog.cancel()
            return
        self.get_logger().warn(
            f"still waiting for detections on '{self.resolve_topic_name('~/detections')}'")

    def _callback(self, msg):
        self._detections_count += 1
        if self._detections_count == 1:
            self.get_logger().info(
                f"first detections message received "
                f"('{self.resolve_topic_name('~/detections')}', {len(msg.detections)} "
                f'person(s) in frame)')

        stamp_sec = _stamp_to_sec(msg.header.stamp)
        dt = 0.0 if self.last_stamp_sec is None else max(0.0, stamp_sec - self.last_stamp_sec)
        self.last_stamp_sec = stamp_sec

        for track in self.tracks.values():
            track.kf.predict(dt)

        matched_track_ids, unmatched_detections = self._associate(msg.detections)

        for track_id, det in matched_track_ids.items():
            track = self.tracks[track_id]
            if _is_finite(det.position):
                track.kf.update([det.position.x, det.position.y, det.position.z])
                track.has_valid_position = True
            track.keypoints_x = _to_float_list(det.keypoints_x)
            track.keypoints_y = _to_float_list(det.keypoints_y)
            track.keypoints_score = _to_float_list(det.keypoints_score)
            track.bbox = (det.bbox_x, det.bbox_y, det.bbox_w, det.bbox_h)
            track.score = det.score
            track.missed_frames = 0
            track.age += 1

        matched_ids = set(matched_track_ids.keys())
        for track_id, track in list(self.tracks.items()):
            if track_id not in matched_ids:
                track.missed_frames += 1
                track.age += 1
                if track.missed_frames > self.max_missed_frames:
                    del self.tracks[track_id]

        for det in unmatched_detections:
            track = Track(self.next_id, det, msg.header.stamp)
            self.tracks[self.next_id] = track
            self.next_id += 1

        self._publish(msg.header)

    def _associate(self, detections):
        """Greedy nearest-neighbor association, 3D distance when both sides
        have valid depth, falling back to pixel-space bbox-center distance
        otherwise (e.g. depth dropout right at the person's location)."""
        candidates = []
        for track_id, track in self.tracks.items():
            for det_idx, det in enumerate(detections):
                cost, valid = self._cost(track, det)
                if valid:
                    candidates.append((cost, track_id, det_idx))

        candidates.sort(key=lambda c: c[0])

        matched_tracks = {}
        used_detections = set()
        for cost, track_id, det_idx in candidates:
            if track_id in matched_tracks or det_idx in used_detections:
                continue
            matched_tracks[track_id] = detections[det_idx]
            used_detections.add(det_idx)

        unmatched_detections = [
            det for i, det in enumerate(detections) if i not in used_detections]
        return matched_tracks, unmatched_detections

    def _cost(self, track, det):
        # Pixel-space match is primary: a person's bbox center moves
        # smoothly frame-to-frame regardless of how noisy the 3D
        # back-projection is (especially with ground_plane, which can
        # swing many meters for a couple pixels of keypoint jitter --
        # see pose_detector_node.py). Matching on 3D position first was
        # causing constant track-id churn: the same physical person,
        # sitting at basically the same pixel, would get treated as a
        # new person because their *estimated distance* jumped.
        tx, ty = track.bbox_center()
        dx = det.bbox_x + det.bbox_w / 2.0
        dy = det.bbox_y + det.bbox_h / 2.0
        pixel_dist = math.hypot(tx - dx, ty - dy)
        if pixel_dist <= self.association_max_pixel_distance:
            return pixel_dist, True

        # Pixel match failed (large jump, e.g. after being briefly
        # occluded) -- fall back to 3D distance as a secondary check.
        if track.has_valid_position and _is_finite(det.position):
            tp = track.kf.position
            dist_3d = math.dist(
                (tp[0], tp[1], tp[2]), (det.position.x, det.position.y, det.position.z))
            return 1000.0 + dist_3d, dist_3d <= self.association_max_distance_m

        return float('inf'), False

    def _publish(self, header):
        out = TrackedPersonArray()
        out.header = header

        for track_id, track in self.tracks.items():
            tp = TrackedPerson()
            tp.id = track_id
            tp.keypoints_x = track.keypoints_x
            tp.keypoints_y = track.keypoints_y
            tp.keypoints_score = track.keypoints_score
            tp.bbox_x, tp.bbox_y, tp.bbox_w, tp.bbox_h = track.bbox
            tp.score = track.score
            tp.missed_frames = track.missed_frames
            tp.age = track.age

            pos = track.kf.position
            vel = track.kf.velocity
            tp.position.x, tp.position.y, tp.position.z = (
                float(pos[0]), float(pos[1]), float(pos[2]))
            tp.velocity.x, tp.velocity.y, tp.velocity.z = (
                float(vel[0]), float(vel[1]), float(vel[2]))

            tp.has_position_base_link = False
            if track.has_valid_position:
                base_point = self._to_base_link(header, pos)
                if base_point is not None:
                    tp.position_base_link = base_point
                    tp.has_position_base_link = True

            out.tracks.append(tp)

        self.tracks_pub.publish(out)

    def _to_base_link(self, header, camera_frame_position):
        point = PointStamped()
        point.header = header
        point.point.x, point.point.y, point.point.z = (
            float(camera_frame_position[0]),
            float(camera_frame_position[1]),
            float(camera_frame_position[2]),
        )
        try:
            transformed = self.tf_buffer.transform(
                point, self.base_frame, timeout=rclpy.duration.Duration(seconds=0.2))
            return transformed.point
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self.get_logger().warn(
                f'tf transform to {self.base_frame} failed: {exc}', throttle_duration_sec=5.0)
            return None


def main(args=None):
    rclpy.init(args=args)
    node = PersonTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
