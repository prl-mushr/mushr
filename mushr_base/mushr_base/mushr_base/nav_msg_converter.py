#!/usr/bin/env python

"""
Converts FoxGlove ROS messages into a format compatible with the MuSHR stack.
Author: Schiffer
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class NavMsgConverter(Node):
  def __init__(self) -> None:
    """
    Initialize Navigation Messages Converter.
    Attributes:
        name (string) rosnode name
    """
    super().__init__("nav_msg_converter")
    self.type = None
    self.declare_parameter("~pose_topic", "/car/car_pose")
    self.declare_parameter("~type_topic", "/foxglove/click_type")
    self.declare_parameter("~goal_topic", "/goal_pose")
    self.declare_parameter("~estimate_topic", "/pose_estimate")
    self.declare_parameter("~start_topic", "/mushr_sim/reposition")
    self.declare_parameter("car_name", "car")
    # Create the subscribers
    self.pose_sub = self.create_subscription(
      PoseStamped, self.get_parameter("~pose_topic").value, self.publish_pose, qos_profile=100
    )
    self.type_sub = self.create_subscription(
      String, self.get_parameter("~type_topic").value, self.save_type, qos_profile=100
    )

    # Create the publishers
    self.goal_pub = self.create_publisher(PoseStamped, self.get_parameter("~goal_topic").value,  qos_profile=1)
    self.car_pose_pub = self.create_publisher(PoseStamped, self.get_parameter("~start_topic").value, qos_profile=1)
    self.pose_estimate_pub = self.create_publisher(PoseStamped, self.get_parameter("~estimate_topic").value, qos_profile=1)

  def publish_pose(self, pose_msg: PoseStamped) -> None:
    """
    Take a pose stamped message and save it.
    """
    print("publishing")
    if self.type == 'goal':
      self.goal_pub.publish(pose_msg)
    elif self.type == 'estimate':
      self.pose_estimate_pub.publish(pose_msg)
    else:
      self.car_pose_pub.publish(pose_msg)
  
  def save_type(self, type_msg: String) -> None:
    self.type = type_msg.data
    print("type: " + str(self.type))
    if self.type != 'pose' and self.type != 'goal' and self.type != 'estimate':
      raise Exception(f'Invalid type detected {self.type}')


def main(args=None):
  rclpy.init(args=args)
  node = NavMsgConverter()
  try:
    rclpy.spin(node)
  finally:
    node.destroy_node()
    rclpy.shutdown()
