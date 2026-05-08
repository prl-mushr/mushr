#!/usr/bin/env python

# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.
from threading import Thread
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
import std_msgs


class FoxgloveTeleop(Node):
    def __init__(self):
        super().__init__("foxglove_teleop")
        self.declare_parameter("speed", 2.0)
        self.declare_parameter("max_steering_angle", 0.34) 

        self.max_velocity = self.get_parameter("speed").value
        # self.get_logger().info("Speed value: " + str(self.max_velocity))

        self.max_steering_angle = self.get_parameter("max_steering_angle").value

        self.state_pub = self.create_publisher(
            AckermannDriveStamped, "mux/ackermann_cmd_mux/input/teleop", 1
        )

        self.foxglove_inputs = self.create_subscription(
            Twist, "foxglove/teleop", self.drive_cb, 1
        )

    def drive_cb(self, msg):
        ack = AckermannDriveStamped()
        ack.header = std_msgs.msg.Header()
        ack.header.stamp = self.get_clock().now().to_msg()
        ack.header.frame_id = "map"

        x = msg.linear.x
        z = msg.linear.z

        # x and z dictate the directions
        cmd_up = x == 1
        cmd_down = x == -1
        cmd_left = z == -1
        cmd_right = z == 1

        # In Foxglove, you can only currently press 1 button at a time,
        # so assume that the direction is forward when turning

        if cmd_up:
            ack.drive.speed = self.max_velocity
        elif cmd_down:
            ack.drive.speed = -self.max_velocity
        elif cmd_left:
            ack.drive.speed = self.max_velocity
            ack.drive.steering_angle = self.max_steering_angle
        elif cmd_right:
            ack.drive.speed = self.max_velocity
            ack.drive.steering_angle = -self.max_steering_angle

        if self.state_pub is not None:
            self.state_pub.publish(ack)


class TeleopThread(Thread):
    def __init__(self, node):
        super(TeleopThread, self).__init__()
        self.teleop = node

    def run(self):
        self.teleop.get_logger().info("Starting Foxglove teleop thread")
        try:
            rclpy.spin(self.teleop)  # blocks until Ctrl+C
        except KeyboardInterrupt:
            pass               # Ctrl+C lands here
        finally:
            self.teleop.destroy_node()
            rclpy.shutdown()

    def shutdown(self):
        self.teleop.root.quit()
        super(TeleopThread, self).shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = FoxgloveTeleop()   # create node first
    teleop_thread = TeleopThread(node)
    teleop_thread.start()

    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()


if __name__ == "__main__":
    main()
