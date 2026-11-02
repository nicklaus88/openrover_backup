"""side_camera_node: publish ONE BlueRobotics USB camera as raw + JPEG.

One process drives one camera. perception_oakd.py starts two of them --
`left_camera` and `right_camera` -- so either side can be restarted,
retuned or disabled without touching the other.

Topics (defaults, per side):

    /capture/camera_left/img              sensor_msgs/Image
    /capture/camera_left/img/compressed   sensor_msgs/CompressedImage

which follows the rover's existing /capture/camera_0/img contract that
oakd_camera already publishes for the forward OAK-D.


WHY NOT v4l2_camera / usb_cam
-----------------------------
Neither is in this image (only image_transport, camera_info_manager and
the depthai stack are). Rather than add an apt dependency to the
Dockerfile, this node does the V4L2 capture with OpenCV, matching how
oakd_camera/oakd_rgb_publisher_node.py already handles images here.


WHY THE Image MESSAGE IS BUILT BY HAND
--------------------------------------
cv_bridge.cv2_to_imgmsg() is NOT usable in this container -- it raises
KeyError(16), because pip's opencv-python numbers CV_8UC3 as 64 while the
compiled cv_bridge extension returns the standard 16. The same trap is
documented in oakd_rgb_publisher_node.py and person_follower. A
bgr8 sensor_msgs/Image is a flat row-major byte buffer, so _to_image()
below fills the six fields directly and sidesteps cv_bridge entirely.


WHY THE DEVICE DEFAULTS ARE /dev/v4l/by-path/... PATHS
------------------------------------------------------
Both cameras are the same model AND report the same USB serial
(2020032801), so /dev/v4l/by-id/ collapses them onto a single symlink --
it cannot tell left from right. /dev/videoN is no good either: the
numbers are assigned in enumeration order and can swap between boots,
silently mirroring the robot's left and right views.

The USB *port* is the only stable discriminator, so the defaults are
by-path symlinks. Each camera exposes four video nodes; only index0
actually captures (index1-3 fail to open), hence the -video-index0
suffix.
"""
import array
import threading

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage, Image

FOURCC = {
    'MJPG': cv2.VideoWriter_fourcc(*'MJPG'),
    'YUYV': cv2.VideoWriter_fourcc(*'YUYV'),
}


class SideCameraNode(Node):

    def __init__(self):
        super().__init__('side_camera')

        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('frame_id', 'side_camera_link')
        self.declare_parameter('raw_topic', '/capture/camera_side/img')
        self.declare_parameter(
            'compressed_topic', '/capture/camera_side/img/compressed')

        # MJPG is what makes 720p/1080p possible at all on this bus: the
        # sensor's uncompressed YUYV mode is capped by USB 2.0 bandwidth
        # and silently falls back to 800x600@16fps when you ask for 720p.
        # Measured on this robot: MJPG gives 1920x1080@33fps, YUYV does
        # not. Only switch to YUYV if JPEG artefacts hurt detection.
        self.declare_parameter('fourcc', 'MJPG')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30.0)

        self.declare_parameter('publish_raw', True)
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('publish_rate_hz', 0.0)
        self.declare_parameter('qos_reliability', 'reliable')
        self.declare_parameter('flip_180', False)
        self.declare_parameter('reopen_after_failures', 30)
        self.declare_parameter('buffer_size', 0)

        self.device = str(self.get_parameter('device').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.fourcc = str(self.get_parameter('fourcc').value).upper()
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.publish_raw = bool(self.get_parameter('publish_raw').value)
        self.publish_compressed = bool(
            self.get_parameter('publish_compressed').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.flip_180 = bool(self.get_parameter('flip_180').value)
        self.reopen_after = int(
            self.get_parameter('reopen_after_failures').value)
        self.buffer_size = int(self.get_parameter('buffer_size').value)

        rate = float(self.get_parameter('publish_rate_hz').value)
        # 0 (or negative) means "publish every frame the sensor gives us".
        self.min_period = (1.0 / rate) if rate > 0.0 else 0.0
        self.last_pub = None

        # RELIABLE by default, for the same reason oakd_camera is: the
        # rover's consumers (camera_detection/perception, `ros2 topic
        # echo`, rqt_image_view) subscribe with a plain depth-10 RELIABLE
        # profile, and a BEST_EFFORT publisher does not match one -- the
        # topic just looks silent. Switch to 'best_effort' only for a
        # remote viewer you are willing to drop frames to.
        pub_qos = self._publisher_qos()

        self.raw_pub = self.create_publisher(
            Image, str(self.get_parameter('raw_topic').value), pub_qos)
        self.compressed_pub = self.create_publisher(
            CompressedImage,
            str(self.get_parameter('compressed_topic').value), pub_qos)

        self.cap = None
        self.frames = 0
        self.drops = 0
        self.failures = 0
        self.stop = threading.Event()

        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        self.create_timer(5.0, self._report)

        self.get_logger().info(
            f'{self.device} -> {self.raw_pub.topic_name} (raw) + '
            f'{self.compressed_pub.topic_name} (jpeg q{self.jpeg_quality}), '
            f'{self.fourcc} {self.width}x{self.height}@{self.fps:g}')

    # ------------------------------------------------------------------
    # setup helpers
    # ------------------------------------------------------------------

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

    def _open(self):
        """Open the device, or return None so the loop can retry."""
        # CAP_V4L2 is forced: OpenCV's default backend choice can land on
        # GStreamer, which ignores the FOURCC/size properties set below.
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return None

        if self.fourcc in FOURCC:
            cap.set(cv2.CAP_PROP_FOURCC, FOURCC[self.fourcc])
        else:
            self.get_logger().warn(
                f'unknown fourcc {self.fourcc!r}, leaving the driver default')
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        # Left at the driver default (0 = don't touch). The tempting
        # setting here is BUFFERSIZE=1 "for low latency", but on this
        # V4L2 driver it HALVES the frame rate -- measured 16 fps with
        # it against 32 fps without, because a single buffer leaves the
        # driver waiting for it to come back before capturing again.
        # Latency stays low anyway: publishing a frame costs ~2 ms (see
        # _to_image), so the reader always drains the queue faster than
        # the sensor fills it.
        if self.buffer_size > 0:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, self.buffer_size)

        got_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        got_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if (got_w, got_h) != (self.width, self.height):
            # The driver silently substitutes the nearest mode it has.
            self.get_logger().warn(
                f'asked for {self.width}x{self.height}, driver gave '
                f'{got_w}x{got_h}')
        return cap

    # ------------------------------------------------------------------
    # capture
    # ------------------------------------------------------------------

    def _capture_loop(self):
        """Grab frames on a private thread and publish them.

        cap.read() blocks until the next frame, so it cannot live in a
        timer callback -- it would stall the executor (and with it the
        parameter/log plumbing) for the whole frame interval.
        """
        while not self.stop.is_set():
            if self.cap is None:
                self.cap = self._open()
                if self.cap is None:
                    self.get_logger().error(
                        f'cannot open {self.device} -- is it plugged in, and '
                        'is /dev mounted into the container?',
                        throttle_duration_sec=5.0)
                    # Don't spin hot on a missing device.
                    self.stop.wait(2.0)
                    continue
                self.failures = 0
                self.get_logger().info(f'opened {self.device}')

            ok, frame = self.cap.read()
            if not ok or frame is None:
                self.failures += 1
                self.drops += 1
                # A USB camera that browns out or is re-enumerated keeps
                # returning False forever; reopening is the only recovery.
                if self.failures >= self.reopen_after:
                    self.get_logger().warn(
                        f'{self.failures} consecutive read failures, '
                        f'reopening {self.device}')
                    self.cap.release()
                    self.cap = None
                    self.stop.wait(1.0)
                continue

            self.failures = 0
            self._publish(frame)

    def _throttled(self):
        """True when this frame should be skipped for publish_rate_hz."""
        if self.min_period <= 0.0:
            return False
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_pub is not None and (now - self.last_pub) < self.min_period:
            return True
        self.last_pub = now
        return False

    def _publish(self, frame):
        if self._throttled():
            return

        if self.flip_180:
            frame = cv2.flip(frame, -1)

        stamp = self.get_clock().now().to_msg()

        if self.publish_raw:
            self.raw_pub.publish(self._to_image(frame, stamp))

        if self.publish_compressed:
            ok, buf = cv2.imencode(
                '.jpg', frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if not ok:
                self.drops += 1
                self.get_logger().warn(
                    'dropping frame, cv2.imencode failed',
                    throttle_duration_sec=5.0)
            else:
                out = CompressedImage()
                out.header.stamp = stamp
                out.header.frame_id = self.frame_id
                out.format = 'jpeg'
                # array.array for the same reason as _to_image(): a bytes
                # assignment here would be element-checked too.
                out.data = array.array('B', buf.tobytes())
                self.compressed_pub.publish(out)

        self.frames += 1

    def _to_image(self, bgr, stamp):
        """Build a bgr8 sensor_msgs/Image without cv_bridge.

        See the module docstring: cv2_to_imgmsg() raises KeyError(16) in
        this container. The layout is just tightly packed rows of BGR
        triplets, so filling the fields by hand is exact, not a
        workaround with caveats.

        `data` MUST be assigned an array.array('B'). rclpy's generated
        setter takes that as a fast path, but for any other sequence --
        including plain bytes -- it falls through to an
        `all(isinstance(v, int) and 0 <= v < 256 ...)` check over every
        element. On a 720p frame that is 2.7 million comparisons:
        measured at 455 ms per frame, i.e. ~2 fps, versus 1.7 ms with
        array.array. That single line is the difference between this
        node running at the sensor's full rate and it crawling.
        """
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = bgr.shape[0]
        msg.width = bgr.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = bgr.shape[1] * 3
        msg.data = array.array('B', bgr.tobytes())
        return msg

    # ------------------------------------------------------------------

    def _report(self):
        if self.frames == 0:
            self.get_logger().warn(
                f'no frames yet from {self.device}')
        else:
            self.get_logger().info(
                f'{self.frames} frames published ({self.drops} dropped)')

    def shutdown(self):
        self.stop.set()
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)
        if self.cap is not None:
            self.cap.release()
            self.cap = None


def main(args=None):
    rclpy.init(args=args)
    node = SideCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
