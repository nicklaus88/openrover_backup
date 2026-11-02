"""goal_publisher_node: FollowTarget + TrackedPersonArray -> geometry_msgs/PoseStamped goal.

An alternative to robot_controller_node for setups with their own
obstacle-avoiding local planner ("bug" algorithm, Nav2, etc.) instead of
driving /cmd_vel directly. Publishes the same equilibrium point
robot_controller_node's proportional control law implicitly converges
to -- standing `distance` meters from the target, roughly along the
current bearing to them, facing them -- as an explicit goal pose in
the robot's base frame, once per synced tracks update while a target is actively
TRACKING. Whatever consumes this topic is responsible for the actual
path planning/driving/obstacle avoidance to reach it.

Only ever one of goal_publisher_node / robot_controller_node should
actually be driving the robot at a time -- see use_goal_publisher in
launch/person_follower.launch.py.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from person_follower_msgs.msg import FollowTarget, TrackedPersonArray


class GoalPublisherNode(Node):

    def __init__(self):
        super().__init__('goal_publisher_node')

        self.declare_parameter('distance', 1.0)
        # Must match person_tracker_node's base_frame -- the positions
        # this republishes come straight from it. base_footprint on this
        # rover; there is no base_link.
        self.declare_parameter('frame_id', 'base_footprint')

        self.distance = float(self.get_parameter('distance').value)
        self.frame_id = self.get_parameter('frame_id').value

        self.target_id = -1
        self.target_state = 'SEARCH'

        self.goal_pub = self.create_publisher(PoseStamped, '~/goal', 1)
        self.target_sub = self.create_subscription(
            FollowTarget, '~/target', self._target_callback, 10)
        self.tracks_sub = self.create_subscription(
            TrackedPersonArray, '~/tracks', self._tracks_callback, 10)

        self.get_logger().info(
            f'goal_publisher_node ready. distance={self.distance}m frame_id={self.frame_id!r} '
            f'-- publishing standoff goals on {self.resolve_topic_name("~/goal")} '
            f'while TRACKING; publishes nothing otherwise.')

    def _target_callback(self, msg):
        self.target_id = msg.target_id
        self.target_state = msg.state

    def _tracks_callback(self, msg):
        if self.target_state != 'TRACKING' or self.target_id < 0:
            return

        track = next((t for t in msg.tracks if t.id == self.target_id), None)
        if track is None or not track.has_position_base_link:
            return

        x = track.position_base_link.x
        y = track.position_base_link.y
        r = math.hypot(x, y)
        if r < 1e-6:
            return  # target exactly at the robot's origin -- no sane direction

        # Stand `distance` short of the target, along the line from here
        # to them (same equilibrium point robot_controller_node's
        # proportional law converges to). If they're already closer than
        # `distance`, this lands behind the robot -- a legitimate "back
        # up" goal, same as robot_controller_node's enable_back case.
        scale = 1.0 - self.distance / r
        goal_x = x * scale
        goal_y = y * scale
        yaw = math.atan2(y, x)  # face the target, not the goal point

        goal = PoseStamped()
        goal.header = msg.header
        goal.header.frame_id = self.frame_id
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        # Yaw-only rotation -> quaternion, computed directly (no need to
        # pull in tf_transformations for this one line).
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_pub.publish(goal)


def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
