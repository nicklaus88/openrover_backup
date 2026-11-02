"""visualization_node: annotated image + RViz markers.

The ROS2 analog of the original's scripts/visualization.py: draws
skeletons/boxes/track ids on the camera image (highlighting the locked
target and its state), and publishes a MarkerArray so tracks show up as
spheres in the robot's base frame in RViz.
"""
import cv2
import cv_bridge
import message_filters
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from visualization_msgs.msg import Marker, MarkerArray

from person_follower_msgs.msg import FollowTarget, TrackedPersonArray


def _bgr8_to_imgmsg(bgr, header):
    """Build a sensor_msgs/Image by hand instead of
    cv_bridge.CvBridge().cv2_to_imgmsg(bgr, encoding='bgr8').

    That call crashes here with KeyError(16): pip's opencv-python (pulled
    in by ultralytics) numbers cv2.CV_8UC3 as 64, while cv_bridge's
    compiled extension was built against real OpenCV and always returns
    the standard value (16) for 'bgr8' -- so cv_bridge's internal
    int-to-name lookup dict (built from the live, mismatched cv2 module)
    doesn't have key 16. Only this *encode* path is affected --
    imgmsg_to_cv2/compressed_imgmsg_to_cv2 (decode) don't hit this and
    work fine, hence they're untouched elsewhere in this package.
    """
    msg = Image()
    msg.header = header
    msg.height, msg.width = bgr.shape[:2]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = bgr.shape[1] * 3
    msg.data = np.ascontiguousarray(bgr).tobytes()
    return msg


# COCO-17 keypoint indices for the ankles (feet) -- used below to mark
# the pixel ground_plane's ray-cast is actually anchored on.
_L_ANKLE, _R_ANKLE = 15, 16

# COCO-17 skeleton edges (by keypoint index).
_SKELETON = [
    (5, 6), (5, 7), (7, 9), (6, 8), (8, 10),
    (5, 11), (6, 12), (11, 12),
    (11, 13), (13, 15), (12, 14), (14, 16),
    (0, 5), (0, 6),
]

_STATE_COLOR = {
    'SEARCH': (0, 200, 200),
    'TRACKING': (0, 220, 0),
    'LOST': (0, 140, 255),
    'REACQUIRE': (0, 0, 255),
}


class VisualizationNode(Node):

    def __init__(self):
        super().__init__('visualization_node')

        # Frame the track positions are expressed in -- must match
        # person_tracker_node's base_frame, since that is where they come
        # from. Was hardcoded 'base_link'; this rover has no such frame,
        # and an unknown frame_id makes RViz silently drop every marker.
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('sync_slop', 0.1)
        self.declare_parameter('marker_lifetime', 0.5)
        self.declare_parameter('use_compressed_image', True)

        self.use_compressed_image = bool(self.get_parameter('use_compressed_image').value)
        self.base_frame = self.get_parameter('base_frame').value
        self.bridge = cv_bridge.CvBridge()
        self.last_target_state = 'SEARCH'
        self.last_target_id = -1

        # annotated_image is always published raw -- it's a debug/RViz
        # output, not something we need to save WiFi bandwidth on here.
        self.image_pub = self.create_publisher(Image, '~/annotated_image', 1)
        self.markers_pub = self.create_publisher(MarkerArray, '~/markers', 1)

        self.target_sub = self.create_subscription(
            FollowTarget, '~/target', self._target_cb, 10)

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
        image_sub.registerCallback(self._on_image_received)
        tracks_sub.registerCallback(self._on_tracks_received)
        self._startup_watchdog = self.create_timer(5.0, self._startup_watchdog_cb)

        image_topic = self.resolve_topic_name('~/image')
        tracks_topic = self.resolve_topic_name('~/tracks')
        self.get_logger().info(
            f'visualization_node ready\n'
            f'  image topic:  {image_topic}\n'
            f'  tracks topic: {tracks_topic}')

    def _startup_watchdog_cb(self):
        if self._image_count and self._tracks_count:
            self.get_logger().info(
                f'all inputs flowing (image={self._image_count}, '
                f'tracks={self._tracks_count} msgs so far) -- stopping startup watchdog.')
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
                f"first tracks message received on '{self.resolve_topic_name('~/tracks')}'")

    def _target_cb(self, msg):
        self.last_target_state = msg.state
        self.last_target_id = msg.target_id

    def _callback(self, image_msg, tracks_msg):
        if self.use_compressed_image:
            bgr = self.bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding='bgr8').copy()
        else:
            bgr = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8').copy()

        for track in tracks_msg.tracks:
            is_target = track.id == self.last_target_id
            color = (
                _STATE_COLOR.get(self.last_target_state, (200, 200, 200))
                if is_target else (150, 150, 150)
            )
            thickness = 3 if is_target else 1

            x, y, w, h = int(track.bbox_x), int(track.bbox_y), int(track.bbox_w), int(track.bbox_h)
            cv2.rectangle(bgr, (x, y), (x + w, y + h), color, thickness)

            label = f'id={track.id}'
            if is_target:
                label += f' [{self.last_target_state}]'
            cv2.putText(bgr, label, (x, max(0, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Ground-plane / position debug: the actual (x, y, z) this
            # track's position was computed as (camera frame -- z is
            # "forward" distance along the optical axis, x is lateral),
            # plus a marker at the pixel that position was ray-cast
            # through (ankle midpoint, or bbox-bottom-center if ankles
            # weren't confident -- see pose_detector_node._backproject_
            # ground_plane's identical fallback logic). Lets you sanity
            # check the ground_plane math live instead of `ros2 topic
            # echo`-ing tracks by hand.
            dist_label = f'x={track.position.x:.2f}m z={track.position.z:.2f}m'
            cv2.putText(bgr, dist_label, (x, y + h + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)

            if (track.keypoints_score[_L_ANKLE] > 0.3
                    and track.keypoints_score[_R_ANKLE] > 0.3):
                gp = (int(0.5 * (track.keypoints_x[_L_ANKLE] + track.keypoints_x[_R_ANKLE])),
                      int(0.5 * (track.keypoints_y[_L_ANKLE] + track.keypoints_y[_R_ANKLE])))
            else:
                gp = (int(x + w / 2.0), int(y + h))
            cv2.drawMarker(bgr, gp, color, markerType=cv2.MARKER_CROSS,
                           markerSize=14, thickness=2)

            for a, b in _SKELETON:
                if (track.keypoints_score[a] > 0.3 and track.keypoints_score[b] > 0.3):
                    pa = (int(track.keypoints_x[a]), int(track.keypoints_y[a]))
                    pb = (int(track.keypoints_x[b]), int(track.keypoints_y[b]))
                    cv2.line(bgr, pa, pb, color, thickness)

        self.image_pub.publish(_bgr8_to_imgmsg(bgr, image_msg.header))
        self._publish_markers(tracks_msg)

    def _publish_markers(self, tracks_msg):
        lifetime = float(self.get_parameter('marker_lifetime').value)
        markers = MarkerArray()
        for track in tracks_msg.tracks:
            if not track.has_position_base_link:
                continue
            is_target = track.id == self.last_target_id
            color = _STATE_COLOR.get(self.last_target_state, (200, 200, 200)) if is_target \
                else (150, 150, 150)

            marker = Marker()
            marker.header.frame_id = self.base_frame
            marker.header.stamp = tracks_msg.header.stamp
            marker.ns = 'person_follower'
            marker.id = track.id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = track.position_base_link
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.3 if is_target else 0.2
            marker.color.b, marker.color.g, marker.color.r = (c / 255.0 for c in color)
            marker.color.a = 1.0
            marker.lifetime = Duration(seconds=lifetime).to_msg()
            markers.markers.append(marker)

        self.markers_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
