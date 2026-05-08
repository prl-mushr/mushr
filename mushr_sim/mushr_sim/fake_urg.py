#!/usr/bin/env python

# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

from __future__ import absolute_import, division, print_function

from threading import Lock

import numpy as np
import range_libc
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Transform
# print("problem after this")
import mushr_base.utils as utils
from ndarray_msg_utils import to_ros_msg, from_ros_msg, NDArray
from sensor_msgs.msg import LaserScan


class FakeURG():
    def __init__(self, node=Node, map_msg=None, topic_namespace="", x_offset=None, **kwargs):
        

        self.node = node
        required = {"update_rate", "theta_discretization", "min_range_meters", "max_range_meters", "angle_step",
                    "angle_min", "angle_max", "z_short", "z_max", "z_blackout_max", "z_rand", "z_hit", "z_sigma",
                    "tf_prefix", "laser_yaw_offset",
                    }
        if not set(kwargs).issubset(required):
            raise ValueError("Invalid keyword argument provided")
        # These next two lines set the instance attributes from the
        # kwargs dictionaries. For example, the key "hit_std" becomes the
        # instance attribute self.hit_std.
        self.__dict__.update(kwargs)

        self.angles = np.arange(
            self.angle_min, self.angle_max, self.angle_step, dtype=np.float32
        )

        occ_map = range_libc.PyOMap(map_msg)
        max_range_px = int(self.max_range_meters / map_msg.info.resolution)
        self.range_method = range_libc.PyCDDTCast(
            occ_map, max_range_px, self.theta_discretization
        )

        self.x_offset = x_offset

        self.laser_pub = self.node.create_publisher(LaserScan, topic_namespace, 10)

        self.update_timer = self.node.create_timer(1.0/self.update_rate, self.timer_cb)

        # Start at 0,0,0
        self.transform = Transform()
        self.transform.rotation.w = 0.0
        self.ranges_lock = Lock()
        self.ranges = np.zeros(len(self.angles) * 1, dtype=np.float32)

    def noise_laser_scan(self, ranges):
        indices = np.zeros(ranges.shape[0], dtype=int)
        prob_sum = self.z_hit + self.z_rand + self.z_short
        hit_count = int((self.z_hit / prob_sum) * indices.shape[0])
        rand_count = int((self.z_rand / prob_sum) * indices.shape[0])
        short_count = indices.shape[0] - hit_count - rand_count
        indices[hit_count: hit_count + rand_count] = 1
        indices[hit_count + rand_count:] = 2
        np.random.shuffle(indices)

        hit_indices = indices == 0
        ranges[hit_indices] += np.random.normal(
            loc=0.0, scale=self.z_sigma, size=hit_count
        )[:]

        rand_indices = indices == 1
        ranges[rand_indices] = np.random.uniform(
            low=self.min_range_meters, high=self.max_range_meters, size=rand_count
        )[:]

        short_indices = indices == 2
        ranges[short_indices] = np.random.uniform(
            low=self.min_range_meters, high=ranges[short_indices], size=short_count
        )[:]

        max_count = (self.z_max / (prob_sum + self.z_max)) * ranges.shape[0]
        while max_count > 0:
            cur = np.random.randint(low=0, high=ranges.shape[0], size=1)
            blackout_count = np.random.randint(low=1, high=self.z_blackout_max, size=1)
            while (
                    0 < cur < ranges.shape[0]
                    and blackout_count > 0
                    and max_count > 0
            ):
                if not np.isnan(ranges[cur]):
                    ranges[cur] = np.nan
                    cur += 1
                    blackout_count -= 1
                    max_count -= 1
                else:
                    break

    def timer_cb(self):
        now = rclpy.clock.Clock().now().to_msg()
        ls = LaserScan()
        ls.header.frame_id = self.tf_prefix + "laser_link"
        ls.header.stamp = now
        ls.angle_increment = self.angle_step
        ls.angle_min = self.angle_min
        ls.angle_max = self.angle_max
        ls.range_min = self.min_range_meters
        ls.range_max = self.max_range_meters
        ls.intensities = []

        car_yaw = utils.quaternion_to_angle(self.transform.rotation)
        laser_pose_x = self.transform.translation.x + self.x_offset * np.cos(car_yaw)
        laser_pose_y = self.transform.translation.y + self.x_offset * np.sin(car_yaw)
        laser_angle = car_yaw + self.laser_yaw_offset

        range_pose = np.array(
            (laser_pose_x, laser_pose_y, laser_angle), dtype=np.float32
        ).reshape(1, 3)
        with self.ranges_lock:
            self.range_method.calc_range_repeat_angles(range_pose, self.angles, self.ranges)
            self.noise_laser_scan(self.ranges)
            ls.ranges = self.ranges.astype(float).tolist()
            self.laser_pub.publish(ls)
