"""oakd_rgb_publisher_node: fan the OAK-D's RGB stream out onto two topics.

The camera itself is driven by `depthai_ros_driver`'s C++ `camera_node`
(started alongside this node by oakd_lite.launch.py) -- the `depthai`
*Python* bindings are not installed in this container, only the ROS
driver, so talking to the device directly from rclpy is not an option
here.

This node subscribes to the driver's RGB output and republishes it as:

    <raw_topic>          sensor_msgs/Image            (pass-through)
    <compressed_topic>   sensor_msgs/CompressedImage  (JPEG)

The raw topic defaults to /capture/camera_0/img -- the name
launch/perception.py already remaps camera_detection/perception's "img"
input to, and which the (empty) spinnaker_camera submodule was meant to
provide. So the OAK-D drops straight into the rover's perception
pipeline with no change to perception.py.

Why not just rely on image_transport? The driver does advertise a
`.../compressed` sibling topic via image_transport, but that only
materialises when the plugin loads, its JPEG quality lives in a separate
parameter namespace, and it republishes at the sensor's full rate. Doing
it here keeps both topics, the quality knob and the rate limit in one
place -- oakd_lite.yaml.
"""
import cv2
import cv_bridge
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage, Image


class OakdRgbPublisherNode(Node):

    def __init__(self):
        super().__init__('oakd_rgb_publisher_node')

        self.declare_parameter('input_topic', '/oak/rgb/image_raw')
        self.declare_parameter('raw_topic', '/capture/camera_0/img')
        self.declare_parameter(
            'compressed_topic', '/capture/camera_0/img/compressed')
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('publish_raw', True)
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('max_rate_hz', 0.0)
        self.declare_parameter('qos_reliability', 'reliable')

        self.input_topic = self.get_parameter('input_topic').value
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.publish_raw = bool(self.get_parameter('publish_raw').value)
        self.publish_compressed = bool(
            self.get_parameter('publish_compressed').value)

        max_rate = float(self.get_parameter('max_rate_hz').value)
        # 0 (or negative) means "publish every frame the driver gives us".
        self.min_period = (1.0 / max_rate) if max_rate > 0.0 else 0.0
        self.last_stamp = None

        self.bridge = cv_bridge.CvBridge()

        # Publish RELIABLE by default. The obvious choice for camera
        # frames would be qos_profile_sensor_data (BEST_EFFORT), but the
        # consumer here is camera_detection/perception, which subscribes
        # with `create_subscription(Image, "img", cb, 10)` -- a plain
        # depth-10 RELIABLE profile. A BEST_EFFORT publisher is
        # incompatible with a RELIABLE subscriber, so perception would
        # silently receive nothing. `ros2 topic echo` and rqt_image_view
        # default to RELIABLE too, which is why a BEST_EFFORT topic looks
        # empty from the command line.
        pub_qos = self._publisher_qos()

        self.raw_pub = self.create_publisher(
            Image, self.get_parameter('raw_topic').value, pub_qos)
        self.compressed_pub = self.create_publisher(
            CompressedImage, self.get_parameter('compressed_topic').value,
            pub_qos)

        self.sub = self.create_subscription(
            Image, self.input_topic, self.on_image, qos_profile_sensor_data)

        self.frames = 0
        self.drops = 0
        self.create_timer(5.0, self.report)

        self.get_logger().info(
            f'relaying {self.input_topic} -> '
            f'{self.raw_pub.topic_name} (raw) + '
            f'{self.compressed_pub.topic_name} (jpeg q{self.jpeg_quality})')

    def _publisher_qos(self):
        """Depth-10 RELIABLE by default; BEST_EFFORT on request."""
        want = str(self.get_parameter('qos_reliability').value).lower()
        if want in ('best_effort', 'sensor_data'):
            return qos_profile_sensor_data
        if want != 'reliable':
            self.get_logger().warn(
                f'unknown qos_reliability {want!r}, using reliable')
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

    def _throttled(self, msg):
        """True when this frame should be skipped to honour max_rate_hz."""
        if self.min_period <= 0.0:
            return False
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is not None and \
                (stamp - self.last_stamp) < self.min_period:
            return True
        self.last_stamp = stamp
        return False

    def on_image(self, msg):
        if self._throttled(msg):
            return

        # The raw side is a straight pass-through -- the driver already
        # hands us a well-formed sensor_msgs/Image, so there is nothing to
        # convert and nothing to copy.
        if self.publish_raw:
            self.raw_pub.publish(msg)

        if self.publish_compressed:
            try:
                # Note: only the *decode* direction of cv_bridge is usable
                # in this container. cv_bridge.cv2_to_imgmsg() raises
                # KeyError(16) here because pip's opencv-python numbers
                # CV_8UC3 as 64 while the compiled extension returns the
                # standard 16 -- see person_follower/visualization_node.py.
                # imgmsg_to_cv2() is unaffected, and the compressed message
                # is built by hand below, so this path steers clear of it.
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as exc:
                self.drops += 1
                self.get_logger().warn(
                    f'dropping frame, decode failed: {exc}',
                    throttle_duration_sec=5.0)
                return

            ok, buf = cv2.imencode(
                '.jpg', bgr,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if not ok:
                self.drops += 1
                self.get_logger().warn(
                    'dropping frame, cv2.imencode failed',
                    throttle_duration_sec=5.0)
                return

            out = CompressedImage()
            out.header = msg.header
            out.format = 'jpeg'
            out.data = buf.tobytes()
            self.compressed_pub.publish(out)

        self.frames += 1

    def report(self):
        if self.frames == 0:
            self.get_logger().warn(
                f'no frames yet on {self.input_topic} -- is the depthai '
                'driver up, and does the topic name match?')
        else:
            self.get_logger().info(
                f'{self.frames} frames relayed ({self.drops} dropped)')


def main(args=None):
    rclpy.init(args=args)
    node = OakdRgbPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
