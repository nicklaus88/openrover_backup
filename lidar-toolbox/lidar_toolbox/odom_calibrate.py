"""
Measure the rover's odometry scale error against a tape measure.

The rover's odometry is open-loop: BaseDriver::motion sets

    odom_out.twist.twist.linear.x = vx * odom_k_x;
    odom_out.twist.twist.linear.y = vy * odom_k_y;

where vx/vy are the *commanded* throttle in [-1,1], not a measured velocity.
There are no wheel encoders anywhere in the stack, so reported distance is
only ever "what I told the motors to do x gain x time". Wheel slip, battery
sag and acceleration ramp-up all make the real distance shorter.

TWO GAINS, CALIBRATED SEPARATELY. k_x and k_y were one k_linear until the
EKF work; they were split because a mecanum base loses far more to roller
slip strafing than driving forward, so k_y is expected to come out
materially below k_x. Calibrating only k_x and assuming the other matches
is exactly the error the split exists to prevent.

Usage:
    1. Put the robot on a start line, tape measure alongside.
    2. Forward gain:
         ros2 run lidar_toolbox odom_calibrate --ros-args -p gain_name:=k_x
       then drive straight FORWARD a known distance (2 m or more -- longer
       is better, the ramp-up error amortises).
    3. Strafe gain:
         ros2 run lidar_toolbox odom_calibrate --ros-args -p gain_name:=k_y
       then STRAFE sideways the same way.
    4. Ctrl-C. It prints reported distance and the corrected gain.
    5. Apply it:
         ros2 run rover base --ros-args -p k_x:=<value> -p k_y:=<value>
       or set the value in whatever launch file starts the base.

Drive in ONE direction only; the reported figure is path length, so any
reversing inflates it. Pass -p gain:=<current value> if the base is not
running the 1.28 default.
"""

import math
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomCalibrate(Node):

    def __init__(self):
        super().__init__("odom_calibrate")
        # The parameter is named for the BaseDriver gain it calibrates,
        # rather than an "axis" of x/y: in YAML, a bare y is the boolean
        # true, so -p axis:=y would fail with an unhelpful type error.
        self.declare_parameter("gain_name", "k_x")
        self.declare_parameter("gain", 1.28)
        self.gain_name = self.get_parameter("gain_name").value
        self.gain = self.get_parameter("gain").value
        self.motion = "forward" if self.gain_name == "k_x" else "sideways"

        self.start = None
        self.last = None
        self.path_length = 0.0
        self.create_subscription(
            Odometry, "/rover/base/odometry", self.callback, 10
        )
        self.get_logger().info(
            f"Recording ({self.gain_name}={self.gain}). "
            f"Drive a measured distance {self.motion}, then Ctrl-C."
        )

    def callback(self, msg: Odometry):
        p = msg.pose.pose.position
        if self.start is None:
            self.start = (p.x, p.y)
        if self.last is not None:
            self.path_length += math.dist((p.x, p.y), self.last)
        self.last = (p.x, p.y)

    def report(self):
        if self.start is None or self.last is None:
            print("\nNo odometry received.")
            return
        straight = math.dist(self.last, self.start)
        print("\n--- odometry ---")
        print(f"  straight-line displacement : {straight:.3f} m")
        print(f"  path length travelled      : {self.path_length:.3f} m")
        print("\nMeasure the true distance with a tape, then:")
        print(f"  corrected {self.gain_name} = {self.gain} * (true / {straight:.3f})")
        for true in (1.0, 2.0, 3.0):
            if straight > 1e-6:
                print(f"    if true was {true:.1f} m -> {self.gain_name} = "
                      f"{self.gain * true / straight:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = OdomCalibrate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C reaches rclpy as ExternalShutdownException, NOT as
        # KeyboardInterrupt. Catching only the latter meant spin() raised
        # straight past report() and the calibration figures -- the entire
        # point of the tool -- were never printed.
        pass
    node.report()
    node.destroy_node()


if __name__ == "__main__":
    main()
