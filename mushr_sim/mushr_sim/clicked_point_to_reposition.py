#!/usr/bin/env python

# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped

def main(args=None):
    
    rclpy.init(args=None)

    def point_clicked_cb(msg):
        as_pose = PoseStamped(header=msg.header)
        as_pose.pose.position = msg.point
        as_pose.pose.orientation.w = 1.0
        pub.publish(as_pose)

    node = Node('clicked_point_to_reposition')

    sub = node.create_subscription(PointStamped, "/clicked_point", point_clicked_cb, 1)
    pub = node.create_publisher(PoseStamped, "reposition", 1)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()