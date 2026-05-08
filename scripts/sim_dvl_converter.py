#!/usr/bin/env python3
"""Converts stonefish_ros2/DVL → nav_msgs/Odometry for AQUA-SLAM.

Runs inside the testbed container (which has stonefish_ros2 installed).
Publishes /bluerov2/dvl so AQUA-SLAM can subscribe without a stonefish_ros2 dependency.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from stonefish_ros2.msg import DVL


class SimDvlConverter(Node):
    def __init__(self):
        super().__init__('sim_dvl_converter')
        self._sub = self.create_subscription(DVL, '/girona500/dvl', self._cb, 50)
        self._pub = self.create_publisher(Odometry, '/bluerov2/dvl', 50)

    def _cb(self, msg: DVL):
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = msg.velocity.x
        odom.twist.twist.linear.y = msg.velocity.y
        odom.twist.twist.linear.z = msg.velocity.z
        # velocity_covariance is row-major 3×3; map to twist covariance 6×6 (linear block)
        vc = msg.velocity_covariance
        odom.twist.covariance[0]  = vc[0]
        odom.twist.covariance[1]  = vc[1]
        odom.twist.covariance[2]  = vc[2]
        odom.twist.covariance[6]  = vc[3]
        odom.twist.covariance[7]  = vc[4]
        odom.twist.covariance[8]  = vc[5]
        odom.twist.covariance[12] = vc[6]
        odom.twist.covariance[13] = vc[7]
        odom.twist.covariance[14] = vc[8]
        self._pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = SimDvlConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
