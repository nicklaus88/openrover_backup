#!/usr/bin/env python3
"""Drive-test diagnostics for odom_source:=fused. Run it, drive, Ctrl+C.

    # terminal 1: ros2 launch launch/robot_start.py
    # terminal 2: ros2 launch launch/localization.py
    # terminal 3:
    python3 scripts/odom_diagnose.py            # add --csv run.csv to keep data

Answers the three questions that decide where fused localization goes
wrong, in the order worth asking them:

1. DO THE TWO INPUTS AGREE ON DIRECTION AND SCALE?
   rf2o's measured body-frame velocity is regressed against the base's
   commanded velocity (BaseDriver: throttle * k_x / k_y), per axis:

       slope ~ 1.0     agree -- k is calibrated
       slope  > 0, != 1  k_x / k_y is off. new_k = k * slope
       slope  < 0      one input is REVERSED (lidar TF, sign in BaseDriver)
       slope ~ 0 / low r   rf2o is not seeing the motion (degenerate scene)

   An EKF fed two inputs that disagree splits the difference and is worse
   than either. This is the first thing to fix.

2. DOES THE EKF FOLLOW rf2o OR THE COMMAND?
   Path length of rf2o, fused, and the integrated command. Fused far from
   rf2o means rf2o is being rejected (odom1_pose_rejection_threshold) or
   out-weighted by an over-confident command covariance.

2b. DOES THE FILTER KNOW IT IS TURNING? (and is the gyro's sign right)
   Heading turned by rf2o, fused and the integrated gyro, and a per-axis
   fit of gyro yaw rate against rf2o's. rf2o's yaw sign does not depend on
   the lidar mount (a pi yaw leaves rotation about z unchanged), so it is
   the reference: slope ~ +1 means the gyro can be fused as is, ~ -1
   means flip the base node's gyro_z_sign first.

3. LOOP CLOSURE.
   Mark the floor, drive a loop, stop on the mark, Ctrl+C. The start->end
   gap of each source is its drift over that loop; the smallest one is the
   one to trust. AMCL's (map frame) is the end-to-end number.

Pure observer: subscribes only, commands nothing.

Every run is saved to ROS2-WorkSpace/bags/odom_diag/<time>.txt (the
summary) and .csv (per-scan samples), so results can be read back later.
"""

import argparse
import csv
import io
import math
import os
import signal
from contextlib import redirect_stdout
from datetime import datetime

import rclpy
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros

MIN_CMD = 0.03      # m/s; below this an axis counts as "not commanded"
MIN_TURN = 0.15     # rad/s (~9 deg/s); below this a scan is "not turning"

OUT_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "bags", "odom_diag")


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Fit:
    """Least-squares slope through the origin, plus correlation."""

    def __init__(self):
        self.n = 0
        self.sxx = self.sxy = self.syy = 0.0

    def add(self, x, y):
        self.n += 1
        self.sxx += x * x
        self.sxy += x * y
        self.syy += y * y

    def report(self, label, yaw=False):
        if self.n < 5 or self.sxx == 0:
            return f"{label}: not enough motion on this axis ({self.n} samples)"
        slope = self.sxy / self.sxx
        r = self.sxy / math.sqrt(self.sxx * self.syy) if self.syy else 0.0
        if yaw:
            if r < 0.5 and slope > 0:
                verdict = "weak correlation -- drive more / faster turns"
            elif slope < 0:
                verdict = "gyro REVERSED -- set gyro_z_sign to the opposite sign"
            else:
                verdict = "agree -- gyro sign is right, safe to fuse vyaw"
                if abs(slope - 1) > 0.2:
                    verdict += f" (scale {slope:.2f}: gyro vs rf2o)"
            return f"{label}: slope {slope:+.2f}  r {r:+.2f}  n {self.n}   {verdict}"
        if slope < 0:
            verdict = "REVERSED -- inputs disagree on direction"
        elif r < 0.5:
            verdict = "weak correlation -- rf2o not tracking this axis"
        elif abs(slope - 1) < 0.1:
            verdict = "agree"
        else:
            verdict = f"scale off -- multiply k by {slope:.2f}"
        return f"{label}: slope {slope:+.2f}  r {r:+.2f}  n {self.n}   {verdict}"


class Diagnose(Node):

    def __init__(self, csv_path):
        super().__init__("odom_diagnose")
        self.cmd = (0.0, 0.0)        # latest commanded body velocity
        self.cmd_acc = [0.0, 0.0, 0]  # sum vx, sum vy, count since last rf2o
        self.cmd_pose = [0.0, 0.0, 0.0]
        self.cmd_last_t = None
        self.prev_rf2o = None
        self.first = {}
        self.last = {}
        self.length = {"rf2o": 0.0, "fused": 0.0, "cmd": 0.0, "amcl": 0.0}
        self.fused_yaw = 0.0
        self.fit_x, self.fit_y = Fit(), Fit()
        self.fit_yaw = Fit()                  # x = rf2o yaw rate, y = gyro
        self.gyro_acc = [0.0, 0]              # sum wz, count since last rf2o
        self.turned = {"rf2o": 0.0, "fused": 0.0, "gyro": 0.0}
        self.prev_fused_yaw = None
        self.gyro_last_t = None
        self.rows = []
        self.csv_path = csv_path

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        self.create_subscription(Odometry, "/rover/base/odometry", self.on_cmd, 50)
        self.create_subscription(Odometry, "/odom_rf2o", self.on_rf2o, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.on_fused, 10)
        self.create_timer(0.2, self.on_amcl)
        self.create_timer(5.0, self.progress)
        self.get_logger().info("Recording. Drive, stop on your start mark, Ctrl+C.")

    def track(self, key, x, y, yaw):
        if key not in self.first:
            self.first[key] = (x, y, yaw)
        if key in self.last:
            lx, ly, _ = self.last[key]
            self.length[key] += math.hypot(x - lx, y - ly)
        self.last[key] = (x, y, yaw)

    def on_cmd(self, m):
        vx, vy = m.twist.twist.linear.x, m.twist.twist.linear.y
        self.cmd = (vx, vy)
        self.cmd_acc[0] += vx
        self.cmd_acc[1] += vy
        self.cmd_acc[2] += 1
        wz = m.twist.twist.angular.z
        self.gyro_acc[0] += wz
        self.gyro_acc[1] += 1
        # Dead-reckon the command through the fused heading: what the EKF
        # would believe with rf2o switched off entirely.
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.cmd_last_t is not None:
            dt = t - self.cmd_last_t
            if 0 < dt < 0.5:
                c, s = math.cos(self.fused_yaw), math.sin(self.fused_yaw)
                self.cmd_pose[0] += (c * vx - s * vy) * dt
                self.cmd_pose[1] += (s * vx + c * vy) * dt
                self.track("cmd", self.cmd_pose[0], self.cmd_pose[1], self.fused_yaw)
                self.turned["gyro"] += wz * dt
        self.cmd_last_t = t

    def on_rf2o(self, m):
        p = m.pose.pose
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        x, y, yaw = p.position.x, p.position.y, yaw_of(p.orientation)
        self.track("rf2o", x, y, yaw)
        if self.prev_rf2o and self.cmd_acc[2]:
            pt, px, py, pyaw = self.prev_rf2o
            dt = t - pt
            if 0.02 < dt < 0.5:
                # World-frame delta rotated into the body frame at the start
                # of the interval, so it is comparable to the command.
                dx, dy = x - px, y - py
                c, s = math.cos(pyaw), math.sin(pyaw)
                bvx = (c * dx + s * dy) / dt
                bvy = (-s * dx + c * dy) / dt
                cvx = self.cmd_acc[0] / self.cmd_acc[2]
                cvy = self.cmd_acc[1] / self.cmd_acc[2]
                if abs(cvx) > MIN_CMD:
                    self.fit_x.add(cvx, bvx)
                if abs(cvy) > MIN_CMD:
                    self.fit_y.add(cvy, bvy)
                rwz = wrap(yaw - pyaw) / dt
                gwz = self.gyro_acc[0] / self.gyro_acc[1] if self.gyro_acc[1] else 0.0
                if abs(rwz) > MIN_TURN or abs(gwz) > MIN_TURN:
                    self.fit_yaw.add(rwz, gwz)
                self.rows.append([f"{t:.3f}", f"{cvx:.4f}", f"{cvy:.4f}",
                                  f"{bvx:.4f}", f"{bvy:.4f}",
                                  f"{rwz:.4f}", f"{gwz:.4f}"])
        if self.prev_rf2o:
            self.turned["rf2o"] += wrap(yaw - self.prev_rf2o[3])
        self.prev_rf2o = (t, x, y, yaw)
        self.cmd_acc = [0.0, 0.0, 0]
        self.gyro_acc = [0.0, 0]

    def on_fused(self, m):
        p = m.pose.pose
        self.fused_yaw = yaw_of(p.orientation)
        if self.prev_fused_yaw is not None:
            self.turned["fused"] += wrap(self.fused_yaw - self.prev_fused_yaw)
        self.prev_fused_yaw = self.fused_yaw
        self.track("fused", p.position.x, p.position.y, self.fused_yaw)

    def on_amcl(self):
        try:
            tf = self.tf_buf.lookup_transform("map", "base_footprint", Time())
        except tf2_ros.TransformException:
            return
        tr = tf.transform.translation
        self.track("amcl", tr.x, tr.y, yaw_of(tf.transform.rotation))

    def progress(self):
        self.get_logger().info(
            f"x-axis samples {self.fit_x.n}, y-axis samples {self.fit_y.n}, "
            f"turn samples {self.fit_yaw.n}, rf2o path {self.length['rf2o']:.2f} m, "
            f"turned {math.degrees(self.turned['rf2o']):+.0f} deg")

    def summary(self):
        print("\n=== 1. rf2o measured vs commanded velocity (body frame) ===")
        print("  " + self.fit_x.report("forward vx"))
        print("  " + self.fit_y.report("strafe  vy"))
        print("  (commanded = throttle * BaseDriver k_x / k_y)")

        print("\n=== 2. path length ===")
        for k in ("rf2o", "fused", "cmd", "amcl"):
            if k in self.first:
                print(f"  {k:6s} {self.length[k]:7.2f} m")
        print("  fused should sit close to rf2o. Near cmd instead = rf2o is")
        print("  being rejected or out-weighted.")

        print("\n=== 2b. heading: does the filter know it turned? ===")
        for k in ("rf2o", "fused", "gyro"):
            print(f"  {k:6s} turned {math.degrees(self.turned[k]):+9.1f} deg (net, CCW +)")
        print("  fused should match rf2o. Well short of it = turns are being")
        print("  dropped (rejection gate) or rf2o is missing fast rotation.")
        print("  " + self.fit_yaw.report("gyro vs rf2o yaw rate", yaw=True))

        print("\n=== 3. loop closure: start -> end (only meaningful if you")
        print("       stopped where you started) ===")
        for k in ("rf2o", "fused", "cmd", "amcl"):
            if k in self.first:
                (x0, y0, a0), (x1, y1, a1) = self.first[k], self.last[k]
                print(f"  {k:6s} gap {math.hypot(x1 - x0, y1 - y0):6.3f} m   "
                      f"heading {math.degrees(wrap(a1 - a0)):+7.2f} deg")

        if self.csv_path and self.rows:
            with open(self.csv_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t", "cmd_vx", "cmd_vy", "rf2o_vx", "rf2o_vy",
                            "rf2o_wz", "gyro_wz"])
                w.writerows(self.rows)
            print(f"\n  per-scan samples -> {self.csv_path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--csv", help="per-scan samples path (default: next to "
                                  "the auto-saved summary in bags/odom_diag/)")
    a = ap.parse_args()
    os.makedirs(OUT_DIR, exist_ok=True)
    stem = os.path.join(OUT_DIR, datetime.now().strftime("%Y%m%d_%H%M%S"))
    a.csv = a.csv or stem + ".csv"
    # Own the SIGINT. rclpy's handler shuts the context down under the
    # executor, and on Humble that surfaces as a RuntimeError out of
    # take_message rather than a KeyboardInterrupt -- losing the summary,
    # which is the whole point of this script.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Diagnose(a.csv)
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    while not stop:
        rclpy.spin_once(node, timeout_sec=0.1)
    buf = io.StringIO()
    with redirect_stdout(buf):
        node.summary()
    text = buf.getvalue()
    print(text)
    with open(stem + ".txt", "w") as f:
        f.write(text)
    print(f"  summary -> {stem}.txt")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
