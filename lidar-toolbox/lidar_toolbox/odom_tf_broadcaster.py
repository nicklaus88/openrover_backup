"""
Re-broadcast wheel odometry as a TF transform.

The rover's `base` node publishes an Odometry *message* on
/rover/base/odometry, but never broadcasts the corresponding TF (there is no
TransformBroadcaster anywhere in src/rover). slam_toolbox needs a complete
map -> odom -> base_footprint -> laser chain: it supplies map->odom itself
and the lidar mount supplies base_footprint->laser, which leaves
odom->base_footprint as the missing link. Without it slam_toolbox never
publishes /map at all.

Frames are taken from the incoming message (header.frame_id -> child_frame_id)
rather than hardcoded, so this keeps matching whatever the driver reports.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):

    def __init__(self):
        super().__init__("odom_tf_broadcaster")

        self.declare_parameter("odom_topic", "/rover/base/odometry")
        odom_topic = self.get_parameter("odom_topic").value

        self.tf = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )
        self.reported = False
        self.get_logger().info(f"Broadcasting TF from odometry on {odom_topic}")

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(t)
        if not self.reported:
            self.get_logger().info(
                f"Publishing {t.header.frame_id} -> {t.child_frame_id}"
            )
            self.reported = True


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()
