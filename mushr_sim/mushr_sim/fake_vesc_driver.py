#!/usr/bin/env python

# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped


class FakeVescDriver(Node):
    def __init__(self):
        super().__init__("fake_vesc_driver")
        self.speed_sub = self.create_subscription(
            Float64, "commands/motor/speed", self.speed_cb, 10
        )
        self.servo_position_sub = self.create_subscription(
            Float64, "commands/servo/position", self.servo_position_cb, 10
        )

        self.state_pub = self.create_publisher(
            VescStateStamped, "sensors/core", 10
        )
        self.servo_pub = self.create_publisher(
            Float64, "sensors/servo_position_command", 10
        )

    def speed_cb(self, msg):
        vss = VescStateStamped()
        vss.header.stamp = self.get_clock().now().to_msg()
        vss.state.speed = msg.data
        self.state_pub.publish(vss)

    def servo_position_cb(self, msg):
        out_msg = Float64()
        out_msg.data = msg.data
        self.servo_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeVescDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
