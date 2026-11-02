"""person_follower_node: the target-selection state machine.

The ROS2 analog of monocular_person_following's orchestrator
(context.cpp + state/*.cpp) and, in a simplified form, of
ccf_person_identification. States mirror the original almost exactly:

  SEARCH     -- no target locked. A person standing close to and roughly
                centered in front of the robot for `imprint_hold_time`
                seconds is auto-imprinted (the original's
                Initial/InitialTraining states), or a track can be locked
                on demand via the ~imprint service.
  TRACKING   -- publishing the locked track's id as target_id, and
                continuously updating a rolling appearance model (HSV
                histogram, see reid.py) of the target for later re-id.
  LOST       -- the target's track id has disappeared from
                person_tracker_node's output; waiting `lost_timeout`
                before giving up and trying to re-identify.
  REACQUIRE  -- looking for a track whose appearance matches the stored
                target appearance for `reacquire_positive_count`
                consecutive frames (the original's ReID state, minus the
                trained CCF/online-boosting classifier).

~reset (topic + std_srvs/Empty service) drops the current target, same as
the original.
"""
import math
from collections import deque

import cv_bridge
import message_filters
import rclpy
import std_srvs.srv
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Empty

from person_follower import reid
from person_follower_msgs.msg import FollowTarget, TrackedPersonArray
from person_follower_msgs.srv import Imprint

STATE_SEARCH = 'SEARCH'
STATE_TRACKING = 'TRACKING'
STATE_LOST = 'LOST'
STATE_REACQUIRE = 'REACQUIRE'


def _stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class PersonFollowerNode(Node):

    def __init__(self):
        super().__init__('person_follower_node')

        self.declare_parameter('imprint_max_distance', 2.0)
        self.declare_parameter('imprint_max_angle_deg', 25.0)
        self.declare_parameter('imprint_hold_time', 1.0)
        self.declare_parameter('lost_timeout', 1.0)
        self.declare_parameter('reacquire_timeout', 20.0)
        self.declare_parameter('reacquire_positive_count', 5)
        self.declare_parameter('reacquire_similarity_thresh', 0.6)
        self.declare_parameter('appearance_history_size', 20)
        self.declare_parameter('sync_slop', 0.1)
        self.declare_parameter('use_compressed_image', True)

        self.imprint_max_distance = float(self.get_parameter('imprint_max_distance').value)
        self.imprint_max_angle = math.radians(
            float(self.get_parameter('imprint_max_angle_deg').value))
        self.imprint_hold_time = float(self.get_parameter('imprint_hold_time').value)
        self.lost_timeout = float(self.get_parameter('lost_timeout').value)
        self.reacquire_timeout = float(self.get_parameter('reacquire_timeout').value)
        self.reacquire_positive_count = int(self.get_parameter('reacquire_positive_count').value)
        self.reacquire_similarity_thresh = float(
            self.get_parameter('reacquire_similarity_thresh').value)
        history_size = int(self.get_parameter('appearance_history_size').value)
        self.use_compressed_image = bool(self.get_parameter('use_compressed_image').value)

        self.bridge = cv_bridge.CvBridge()

        self.state = STATE_SEARCH
        self.target_id = -1
        self.target_appearance = None
        self.appearance_history = deque(maxlen=history_size)

        self.candidate_id = None
        self.candidate_hold_start = None

        self.lost_since = None
        self.reacquire_since = None
        self.reacquire_counts = {}

        self.last_tracks_by_id = {}
        self.last_image = None

        self.target_pub = self.create_publisher(FollowTarget, '~/target', 1)

        image_type = CompressedImage if self.use_compressed_image else Image
        image_sub = message_filters.Subscriber(
            self, image_type, '~/image', qos_profile=qos_profile_sensor_data)
        tracks_sub = message_filters.Subscriber(self, TrackedPersonArray, '~/tracks')
        sync_slop = float(self.get_parameter('sync_slop').value)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [image_sub, tracks_sub], queue_size=10, slop=sync_slop)
        self.sync.registerCallback(self._callback)

        # debug instrumentation -- see pose_detector_node.py for rationale
        self._image_count = 0
        self._tracks_count = 0
        self._sync_count = 0
        image_sub.registerCallback(self._on_image_received)
        tracks_sub.registerCallback(self._on_tracks_received)
        self._startup_watchdog = self.create_timer(5.0, self._startup_watchdog_cb)

        self.reset_sub = self.create_subscription(Empty, '~/reset', self._reset_topic_cb, 10)
        self.reset_srv = self.create_service(
            std_srvs.srv.Empty, '~/reset', self._reset_service_cb)
        self.imprint_srv = self.create_service(Imprint, '~/imprint', self._imprint_service_cb)

        image_topic = self.resolve_topic_name('~/image')
        tracks_topic = self.resolve_topic_name('~/tracks')
        self.get_logger().info(
            f'person_follower_node ready, state=SEARCH\n'
            f'  image topic:  {image_topic}\n'
            f'  tracks topic: {tracks_topic}')

    def _startup_watchdog_cb(self):
        if self._image_count and self._tracks_count:
            self.get_logger().info(
                f'all inputs flowing (image={self._image_count}, tracks={self._tracks_count} '
                f'msgs so far) -- stopping startup watchdog.')
            self._startup_watchdog.cancel()
            return
        missing = []
        if not self._image_count:
            missing.append(f"image ('{self.resolve_topic_name('~/image')}')")
        if not self._tracks_count:
            missing.append(f"tracks ('{self.resolve_topic_name('~/tracks')}')")
        self.get_logger().warn('still waiting for: ' + ', '.join(missing))

    def _on_image_received(self, msg):
        self._image_count += 1
        if self._image_count == 1:
            self.get_logger().info(
                f"first image received on '{self.resolve_topic_name('~/image')}'")

    def _on_tracks_received(self, msg):
        self._tracks_count += 1
        if self._tracks_count == 1:
            self.get_logger().info(
                f"first tracks message received on '{self.resolve_topic_name('~/tracks')}' "
                f'({len(msg.tracks)} track(s))')

    def _callback(self, image_msg, tracks_msg):
        self._sync_count += 1
        if self._sync_count == 1:
            self.get_logger().info('first synced image+tracks pair matched within sync_slop.')
        now = _stamp_to_sec(tracks_msg.header.stamp)
        if self.use_compressed_image:
            bgr = self.bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        else:
            bgr = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        self.last_image = bgr
        self.last_tracks_by_id = {t.id: t for t in tracks_msg.tracks}

        if self.state == STATE_SEARCH:
            self._step_search(tracks_msg, now)
        elif self.state == STATE_TRACKING:
            self._step_tracking(bgr, now)
        elif self.state == STATE_LOST:
            self._step_lost(now)
        elif self.state == STATE_REACQUIRE:
            self._step_reacquire(bgr, now)

        self._publish(tracks_msg.header, now)

    def _crop(self, bgr, track):
        h, w = bgr.shape[:2]
        x0 = max(0, int(track.bbox_x))
        y0 = max(0, int(track.bbox_y))
        x1 = min(w, int(track.bbox_x + track.bbox_w))
        y1 = min(h, int(track.bbox_y + track.bbox_h))
        if x1 <= x0 or y1 <= y0:
            return None
        return bgr[y0:y1, x0:x1]

    # -- SEARCH -----------------------------------------------------------

    def _step_search(self, tracks_msg, now):
        best = None
        best_dist = None
        for t in tracks_msg.tracks:
            if not t.has_position_base_link:
                continue
            x, y = t.position_base_link.x, t.position_base_link.y
            dist = math.hypot(x, y)
            angle = abs(math.atan2(y, x))
            if dist <= self.imprint_max_distance and angle <= self.imprint_max_angle:
                if best_dist is None or dist < best_dist:
                    best, best_dist = t, dist

        if best is None:
            self.candidate_id = None
            self.candidate_hold_start = None
            return

        if self.candidate_id != best.id:
            self.candidate_id = best.id
            self.candidate_hold_start = now
            return

        if now - self.candidate_hold_start >= self.imprint_hold_time:
            self._lock_target(best.id, self.last_image, best)

    # -- TRACKING -----------------------------------------------------------

    def _step_tracking(self, bgr, now):
        track = self.last_tracks_by_id.get(self.target_id)
        if track is None:
            self.state = STATE_LOST
            self.lost_since = now
            return

        crop = self._crop(bgr, track)
        hist = reid.extract_appearance(crop)
        if hist is not None:
            self.appearance_history.append(hist)
            self.target_appearance = reid.average_histograms(list(self.appearance_history))

    # -- LOST -----------------------------------------------------------

    def _step_lost(self, now):
        if self.target_id in self.last_tracks_by_id:
            self.state = STATE_TRACKING
            return

        if now - self.lost_since >= self.lost_timeout:
            self.state = STATE_REACQUIRE
            self.reacquire_since = now
            self.reacquire_counts = {}

    # -- REACQUIRE -----------------------------------------------------------

    def _step_reacquire(self, bgr, now):
        if self.target_id in self.last_tracks_by_id:
            self.state = STATE_TRACKING
            return

        for track_id, track in self.last_tracks_by_id.items():
            crop = self._crop(bgr, track)
            hist = reid.extract_appearance(crop)
            score = reid.similarity(hist, self.target_appearance)
            if score >= self.reacquire_similarity_thresh:
                self.reacquire_counts[track_id] = self.reacquire_counts.get(track_id, 0) + 1
            else:
                self.reacquire_counts[track_id] = 0

            if self.reacquire_counts[track_id] >= self.reacquire_positive_count:
                self.appearance_history.clear()
                self._lock_target(track_id, bgr, track)
                return

        if now - self.reacquire_since >= self.reacquire_timeout:
            self._drop_target()

    # -- shared helpers -----------------------------------------------------------

    def _lock_target(self, track_id, bgr, track):
        self.target_id = track_id
        self.state = STATE_TRACKING
        self.candidate_id = None
        self.candidate_hold_start = None
        self.lost_since = None
        self.reacquire_since = None
        self.reacquire_counts = {}
        if bgr is not None and track is not None:
            crop = self._crop(bgr, track)
            hist = reid.extract_appearance(crop)
            if hist is not None:
                self.appearance_history.append(hist)
                self.target_appearance = reid.average_histograms(list(self.appearance_history))
        self.get_logger().info(f'locked onto target track {track_id}')

    def _drop_target(self):
        self.get_logger().info('giving up on target, returning to SEARCH')
        self.state = STATE_SEARCH
        self.target_id = -1
        self.target_appearance = None
        self.appearance_history.clear()
        self.candidate_id = None
        self.candidate_hold_start = None
        self.lost_since = None
        self.reacquire_since = None
        self.reacquire_counts = {}

    def _publish(self, header, now):
        msg = FollowTarget()
        msg.header = header
        msg.state = self.state
        msg.target_id = self.target_id
        msg.track_ids = list(self.last_tracks_by_id.keys())
        if self.target_appearance is not None:
            scores = []
            for track_id in msg.track_ids:
                track = self.last_tracks_by_id[track_id]
                crop = self._crop(self.last_image, track) if self.last_image is not None else None
                hist = reid.extract_appearance(crop)
                scores.append(reid.similarity(hist, self.target_appearance))
            msg.reid_scores = scores

        msg.candidate_id = self.candidate_id if self.candidate_id is not None else -1
        msg.candidate_hold_elapsed = (
            max(0.0, now - self.candidate_hold_start)
            if self.candidate_hold_start is not None else 0.0)
        msg.candidate_hold_required = self.imprint_hold_time

        self.target_pub.publish(msg)

    # -- services -----------------------------------------------------------

    def _reset_topic_cb(self, _msg):
        self._drop_target()

    def _reset_service_cb(self, request, response):
        self._drop_target()
        return response

    def _imprint_service_cb(self, request, response):
        track = self.last_tracks_by_id.get(request.target_id)
        if track is None:
            self.get_logger().warn(f'imprint requested for unknown track {request.target_id}')
            response.success = False
            return response

        self._lock_target(request.target_id, self.last_image, track)
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
