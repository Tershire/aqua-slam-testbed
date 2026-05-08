#!/usr/bin/env python3
"""Forward straight-line test for AQUA-SLAM initialization.

Drives the Girona500 forward with a trapezoidal thrust profile:
  ramp-up → cruise → ramp-down → stop

Usage:
    python3 forward_test.py [--thrust THRUST] [--cruise CRUISE] [--ramp RAMP]

    --thrust  Peak normalized thrust (0–1, default 0.3)
    --cruise  Cruise duration in seconds (default 20.0)
    --ramp    Ramp duration in seconds   (default 3.0)

Topic:
    /girona500/surge_thruster/setpoint  (std_msgs/Float64)

The trapezoidal profile is repeatable and gives a smooth velocity
curve, which is better for IMU integration than a step command.
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class ForwardTest(Node):
    def __init__(self, thrust: float, cruise: float, ramp: float):
        super().__init__('forward_test')
        self._pub = self.create_publisher(
            Float64, '/girona500/surge_thruster/setpoint', 10
        )
        self._thrust = thrust
        self._cruise = cruise
        self._ramp   = ramp

    def run(self):
        total = 2 * self._ramp + self._cruise
        self.get_logger().info(
            f'Forward test: thrust={self._thrust:.2f}  '
            f'ramp={self._ramp:.1f}s  cruise={self._cruise:.1f}s  '
            f'total={total:.1f}s'
        )

        rate_hz = 50.0
        dt      = 1.0 / rate_hz
        rate    = self.create_rate(rate_hz)
        t0      = self.get_clock().now().nanoseconds * 1e-9
        msg     = Float64()

        while rclpy.ok():
            t = self.get_clock().now().nanoseconds * 1e-9 - t0

            if t < self._ramp:
                # linear ramp-up
                setpoint = self._thrust * (t / self._ramp)
            elif t < self._ramp + self._cruise:
                # cruise
                setpoint = self._thrust
            elif t < total:
                # linear ramp-down
                setpoint = self._thrust * (1.0 - (t - self._ramp - self._cruise) / self._ramp)
            else:
                setpoint = 0.0
                msg.data = 0.0
                self._pub.publish(msg)
                self.get_logger().info('Forward test complete.')
                break

            msg.data = setpoint
            self._pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            rate.sleep()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--thrust', type=float, default=0.3,
                        help='Peak normalized thrust (0–1)')
    parser.add_argument('--cruise', type=float, default=20.0,
                        help='Cruise duration [s]')
    parser.add_argument('--ramp',   type=float, default=3.0,
                        help='Ramp up/down duration [s]')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = ForwardTest(args.thrust, args.cruise, args.ramp)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
