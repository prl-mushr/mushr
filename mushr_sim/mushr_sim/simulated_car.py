#!/usr/bin/env python

from __future__ import absolute_import, division, print_function

from threading import Lock

import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped
# from mushr_base import utils
from mushr_sim.fake_urg import FakeURG
from mushr_base.motion_model import KinematicCarMotionModel
from mushr_sim_interfaces.srv import CarPose
from mushr_sim.utils import wrap_angle
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
import mushr_base.utils as utils


class SimulatedCar():
    """
    Publishes joint and tf information about the racecar
    """

    def __init__(self, node,  car_name, x, y, theta, map_info=None, permissible_region=None, speed_to_erpm_offset=None,
                 speed_to_erpm_gain=None, motion_model=None, sensor_model=None, tf_prefix=None):
        
        self.node = node
        self.car_name = car_name
        self.tf_prefix = tf_prefix
        if self.tf_prefix is None:
            self.tf_prefix = ""
        self.speed_to_erpm_offset = speed_to_erpm_offset
        self.speed_to_erpm_gain = speed_to_erpm_gain
        # The most recent transform from odom to base_footprint
        self.odom_to_base_trans = np.array([x, y], dtype=float)
        self.odom_to_base_trans = np.nan_to_num(self.odom_to_base_trans)
        self.odom_to_base_rot = np.nan_to_num(theta)
        self.odom_to_base_lock = Lock()
        self.permissible_region = permissible_region
        self.map_info = map_info

        # The most recent speed (m/s)
        self.last_speed = 0.0
        self.last_speed_lock = Lock()

        # The most recent steering angle (rad)
        self.last_steering_angle = 0.0
        self.last_steering_angle_lock = Lock()

        # Internal transform from the map to odom
        self.map_to_odom_trans = np.array([0, 0], dtype=float)
        self.map_to_odom_rot = 0
        self.map_to_odom_lock = Lock()

        # Message used to publish joint values
        self.joint_msg = JointState()
        self.joint_msg.name = [
            "front_left_wheel_throttle",
            "front_right_wheel_throttle",
            "back_left_wheel_throttle",
            "back_right_wheel_throttle",
            "front_left_wheel_steer",
            "front_right_wheel_steer",
        ]
        self.joint_msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_msg.velocity = []
        self.joint_msg.effort = []

        self.fake_laser = sensor_model
        # Publishes joint values
        self.state_pub = self.node.create_publisher(PoseStamped, f"{car_name}/car_pose", 1)
        self.odom_pub = self.node.create_publisher(Odometry, f"{car_name}/odom", 5)

        # Publishes joint values
        self.cur_joints_pub = self.node.create_publisher(JointState, f"{car_name}/joint_states", 1) 
        # Subscribes to the initial pose of the car
        self.init_pose_sub = self.node.create_subscription(PoseStamped, "/mushr_sim/reposition", self.init_pose_cb, 1)

        # Subscribes to info about the bldc (particularly the speed in rpm)
        self.speed_sub = self.node.create_subscription(VescStateStamped, f"/{car_name}/vesc/sensors/core", self.speed_cb, 1)

        # Subscribes to the position of the servo arm
        self.servo_sub = self.node.create_subscription(Float64, f"/{car_name}/vesc/sensors/servo_position_command", self.servo_cb, 1)
        self.motion_model = motion_model

    def init_pose_cb(self, msg):
        """
         init_pose_cb: Callback to capture the initial pose of the car
           msg: geometry_msg/PoseStamped containing the initial pose
         """
        # Get the pose of the car w.r.t the map in meters
        rx_trans = np.array(
            [msg.pose.position.x, msg.pose.position.y], dtype=float
        )
        rx_rot = utils.quaternion_to_angle(msg.pose.orientation)

        # Get the pose of the car w.r.t the map in pixels
        if self.map_info is not None:
            map_rx_pose = utils.world_to_map(
                (rx_trans[0], rx_trans[1], rx_rot), self.map_info
            )
            # Update the pose of the car if either bounds checking is not enabled,
            # or bounds checking is enabled but the car is in-bounds
            if not check_position_in_bounds(map_rx_pose[0], map_rx_pose[1], self.permissible_region):
                self.node.get_logger().warn("Requested reposition into obstacle. Ignoring.")
                return

        with self.odom_to_base_lock:
            # Move the vehicle by updating the odom->base transform
            self.odom_to_base_trans = rx_trans
            self.odom_to_base_rot = rx_rot

    def speed_cb(self, msg):
        """
        Callback to capture the speed of the car
          msg: vesc_msgs/VescStateStamped message containing the speed of the car (rpm)
        """
        self.last_speed_lock.acquire()
        self.node.get_logger().debug(f"Received speed rpm: {msg.state.speed}")
        self.last_speed = (msg.state.speed - self.speed_to_erpm_offset) / self.speed_to_erpm_gain
        self.last_speed_lock.release()

    def servo_cb(self, msg):
        """
        Callback to capture the steering angle of the car
         msg: std_msgs/Float64 message containing the servo value
        """
        with self.last_steering_angle_lock:
            self.last_steering_angle = (msg.data - self.motion_model.steering_to_servo_offset) / self.motion_model.steering_to_servo_gain

    def reposition(self, x, y, theta):
        rx_trans = np.array(
            [x, y], dtype=float
        )
        rx_rot = theta

        # Get the pose of the car w.r.t the map in pixels
        if self.map_info is not None:
            map_rx_pose = utils.world_to_map(
                (rx_trans[0], rx_trans[1], rx_rot), self.map_info
            )
            # Update the pose of the car if either bounds checking is not enabled,
            # or bounds checking is enabled but the car is in-bounds

            if not check_position_in_bounds(map_rx_pose[0], map_rx_pose[1], self.permissible_region):
                self.node.get_logger().warn("Requested reposition into obstacle. Ignoring.")
                return

        with self.odom_to_base_lock:
            # Move the vehicle by updating the odom->base transform
            self.odom_to_base_trans = rx_trans
            self.odom_to_base_rot = rx_rot

        with self.odom_to_base_lock:
            self.odom_to_base_trans = np.array([x, y], dtype=float)
            self.odom_to_base_rot = theta

    def simulate(self, dt, now):

        # Add noise to the speed
        with self.last_speed_lock:
            v = self.last_speed
        # Add noise to the steering angle
        with self.last_steering_angle_lock:
            delta = self.last_steering_angle

        with self.odom_to_base_lock:
            # Apply kinematic car model to the previous pose
            new_pose = np.array(
                [
                    self.odom_to_base_trans[0],
                    self.odom_to_base_trans[1],
                    self.odom_to_base_rot,
                ],
                dtype=float,
            )
            # self.get_logger().warn(f"Requested reposition to map coords: {v, delta, dt}")

            dt = dt / 1000000000.0
            state_changes, joint_changes = self.motion_model.apply_motion_model(new_pose[np.newaxis, ...],
                                                                             np.array([[v, delta]]), dt)

            state_changes = state_changes.squeeze()
            # self.node.get_logger().warn(f"Requested reposition to map coords: {state_changes}")

            joint_changes = joint_changes.squeeze()
            in_bounds = True
            if self.permissible_region is not None:
                # Compute the new pose w.r.t the map in meters
                new_map_pose = np.zeros(3, dtype=float)
                new_map_pose[0] = self.map_to_odom_trans[0] + (
                        new_pose[0] * np.cos(self.map_to_odom_rot)
                        - new_pose[1] * np.sin(self.map_to_odom_rot)
                )
                new_map_pose[1] = self.map_to_odom_trans[1] + (
                        new_pose[0] * np.sin(self.map_to_odom_rot)
                        + new_pose[1] * np.cos(self.map_to_odom_rot)
                )
                new_map_pose[2] = self.map_to_odom_rot + new_pose[2]    

                # Get the new pose w.r.t the map in pixels
                if self.permissible_region is not None:
                    # self.node.get_logger().info(f"map_info.origin: {self.map_info.origin}")
                    new_map_pose = utils.world_to_map(new_map_pose, self.map_info)

                    # print(f"x_pos is <{new_map_pose[0]}>, y_pos is <{new_map_pose[1]}>, is in bounds: <{check_position_in_bounds(new_map_pose[0], new_map_pose[1], self.permissible_region)}>")#\npermissible region is: <{self.permissible_region}>")
                    # print(np.count_nonzero(self.permissible_region), self.permissible_region.size)
                    # print(np.unique(self.permissible_region, return_counts=True))
                    in_bounds = check_position_in_bounds(new_map_pose[0], new_map_pose[1], self.permissible_region)
                    
            if in_bounds:
                # Update pose of base_footprint w.r.t odom
                # self.node.get_logger().info(f"Updating pose to: {new_pose}")
                self.odom_to_base_trans[0] = new_pose[0]
                self.odom_to_base_trans[1] = new_pose[1]
                self.odom_to_base_rot = new_pose[2]

                # Update joint values
                self.joint_msg.position[0] += joint_changes[0]
                self.joint_msg.position[1] += joint_changes[1]
                self.joint_msg.position[2] += joint_changes[0]
                self.joint_msg.position[3] += joint_changes[1]
                self.joint_msg.position[4] = joint_changes[2]
                self.joint_msg.position[5] = joint_changes[3]

                # Clip all joint angles
                for i in range(len(self.joint_msg.position)):
                    self.joint_msg.position[i] = utils.wrap_angle(self.joint_msg.position[i])
            else:
                self.node.get_logger().warn("Not in bounds")

            # Publish the joint states
            self.joint_msg.header.stamp = now.to_msg()
            self.cur_joints_pub.publish(self.joint_msg)

            t = utils.make_transform_msg(self.odom_to_base_trans, self.odom_to_base_rot,
                                         self.tf_prefix + "base_footprint", self.tf_prefix + "odom", now)

            # Tell the laser where we are
            # rospy.logerr_throttle(1,t)
            # rospy.logerr_throttle(1, new_pose)
            self.fake_laser.transform = t.transform
            # self.node.get_logger().info(f"Publishing transform: {t}")
            self.transform = t
            self.publish_updated_transforms(self.transform, state_changes)

        # Publish current state as a PoseStamped topic

    def publish_updated_transforms(self, transform, changes):
        cur_pose = PoseStamped()
        cur_pose.header.frame_id = "map"

        t = rclpy.time.Time.from_msg(transform.header.stamp)
        t = rclpy.duration.Duration(seconds=0.5) + t
        cur_pose.header.stamp = t.to_msg()

        # for visualization purposes, delay header stamp
        cur_pose.pose.position.x = (
                self.odom_to_base_trans[0] + self.map_to_odom_trans[0]
        )
        cur_pose.pose.position.y = (
                self.odom_to_base_trans[1] + self.map_to_odom_trans[1]
        )
        cur_pose.pose.position.z = 0.0
        rot = self.odom_to_base_rot + self.map_to_odom_rot
        cur_pose.pose.orientation = utils.angle_to_quaternion(rot)
        self.state_pub.publish(cur_pose)

        odom_msg = Odometry()
        odom_msg.header.stamp = transform.header.stamp
        odom_msg.header.frame_id = self.tf_prefix + "odom"
        odom_msg.pose.pose.position.x = transform.transform.translation.x
        odom_msg.pose.pose.position.y = transform.transform.translation.y
        odom_msg.pose.pose.position.z = transform.transform.translation.z
        odom_msg.pose.pose.orientation = transform.transform.rotation

        odom_msg.child_frame_id = self.tf_prefix + "base_footprint"
        odom_msg.twist.twist.linear.x = changes[0]
        odom_msg.twist.twist.linear.y = changes[1]
        odom_msg.twist.twist.angular.z = 0.0 # changes[2]

        self.odom_pub.publish(odom_msg)


def check_position_in_bounds(x, y, permissible_region):
    if permissible_region is None:
        return True
    return (
            0 <= x < permissible_region.shape[1] and \
            0 <= y < permissible_region.shape[0] and \
            permissible_region[
                int(y + 0.5), int(x + 0.5)
            ]
    )
