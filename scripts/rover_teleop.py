#!/usr/bin/env python3
"""Keyboard teleop for the rover with uniform, repeatable motion.

    python3 scripts/rover_teleop.py
    python3 scripts/rover_teleop.py --speed 0.25 --turn 0.4 --pulse 0.6

WHY NOT teleop_twist_keyboard
-----------------------------
teleop_twist_keyboard publishes ONCE per key press and then nothing, and
BaseDriver keeps executing the last command until its 2 s watchdog
(vel_timeout) halts it. So a tap turned for ~2 s, a held key for as long
as the terminal's key repeat kept firing, and two taps 1 s apart for ~3 s
-- the same key moved the robot a different amount every time.

This publishes at a fixed rate and stops --pulse seconds after the last
key event, so:

    tap          one pulse of exactly --pulse seconds, every time
    hold         continuous (terminal key repeat keeps renewing it)
    release      stops within --pulse seconds, not 2 s
    space        stops immediately

--pulse must exceed your terminal's key-repeat delay (usually 0.5 s) or
holding a key stutters for its first half-second.

Velocities are BaseDriver THROTTLE in [-1, 1], not m/s: 1.0 forward is
about k_x = 1.28 m/s. Rotation has no calibrated rate at all.

KEYS
----
    w / s     forward / back           q / e     rotate left / right
    a / d     strafe left / right      space     stop
    + / -     scale all speeds         Ctrl+C    quit (sends a stop)
"""

import argparse
import select
import signal
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.signals import SignalHandlerOptions

TOPIC = "/rover/base/velocity/set"

# key -> (vx, vy, wz) unit direction
KEYS = {
    "w": (1, 0, 0), "s": (-1, 0, 0),
    "a": (0, 1, 0), "d": (0, -1, 0),
    "q": (0, 0, 1), "e": (0, 0, -1),
}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--speed", type=float, default=0.25,
                    help="linear throttle (default 0.25 ~ 0.3 m/s)")
    ap.add_argument("--turn", type=float, default=0.4,
                    help="rotation throttle (default 0.4)")
    ap.add_argument("--pulse", type=float, default=0.6,
                    help="seconds a key press keeps the robot moving")
    ap.add_argument("--rate", type=float, default=20.0, help="publish Hz")
    a = ap.parse_args()

    # Own SIGINT/SIGTERM. rclpy's handler would shut the context down
    # underneath us, so Ctrl+C neither exited the loop nor could send the
    # final stop -- the one thing a teleop must always do.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("rover_teleop")
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    pub = node.create_publisher(Twist, TOPIC, 10)

    def send(vx=0.0, vy=0.0, wz=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        pub.publish(t)

    scale = 1.0
    direction = (0, 0, 0)
    last_key = 0.0
    print(__doc__[__doc__.index("KEYS"):])
    print(f"speed {a.speed}  turn {a.turn}  pulse {a.pulse}s  -> {TOPIC}")

    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        period = 1.0 / a.rate
        while not stop:
            # Drain every key that arrived this tick; the last one wins.
            while not stop and select.select([sys.stdin], [], [], 0)[0]:
                k = sys.stdin.read(1)
                if k in KEYS:
                    direction, last_key = KEYS[k], time.monotonic()
                elif k == " ":
                    direction = (0, 0, 0)
                elif k in "+=":
                    scale *= 1.25
                    print(f"\rscale {scale:.2f}  speed {a.speed * scale:.2f}  "
                          f"turn {a.turn * scale:.2f}   ", end="", flush=True)
                elif k in "-_":
                    scale /= 1.25
                    print(f"\rscale {scale:.2f}  speed {a.speed * scale:.2f}  "
                          f"turn {a.turn * scale:.2f}   ", end="", flush=True)
            if time.monotonic() - last_key > a.pulse:
                direction = (0, 0, 0)
            vx, vy, wz = direction
            # Publishing zeros continuously is deliberate: it keeps the
            # base's watchdog fed with an explicit stop, rather than
            # letting a stale command run out the 2 s timeout.
            send(vx * a.speed * scale, vy * a.speed * scale, wz * a.turn * scale)
            rclpy.spin_once(node, timeout_sec=0)
            time.sleep(period)
    finally:
        send()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\nstopped")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
