#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def _static_tf(name, arguments):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=arguments,
        output="screen",
    )


def generate_launch_description():
    return LaunchDescription(
        [
            _static_tf(
                "base_link_to_imu",
                ["0.245", "0.0", "0.117", "0.7071067811865475", "0.7071067811865475", "0.0", "0.0", "base_link", "base_imu_link"],
            ),
            _static_tf(
                "base_link_to_laser",
                ["0.285", "0.0", "0.127", "0.0", "0.0", "0.0", "1.0", "base_link", "laser"],
            ),
            _static_tf(
                "base_link_to_camera",
                ["0.2538", "-0.0262", "0.1983", "0.0", "0.0", "0.0", "1.0", "base_link", "camera_link"],
            ),
            _static_tf(
                "base_link_to_base_footprint",
                ["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "1.0", "base_link", "base_footprint"],
            ),
            _static_tf(
                "base_link_to_chassis",
                ["0.0", "0.0", "0.05", "0.0", "0.0", "0.0", "1.0", "base_link", "chassis"],
            ),
            _static_tf(
                "chassis_to_left_rear_wheel",
                ["0.0", "0.1", "0.0", "0.0", "0.0", "1.5708", "chassis", "left_rear_wheel"],
            ),
            _static_tf(
                "chassis_to_right_rear_wheel",
                ["0.0", "-0.1", "0.0", "0.0", "0.0", "1.5708", "chassis", "right_rear_wheel"],
            ),
            _static_tf(
                "chassis_to_left_steering_hinge",
                ["0.325", "0.1", "0.0", "0.0", "1.5708", "0.0", "chassis", "left_steering_hinge"],
            ),
            _static_tf(
                "chassis_to_right_steering_hinge",
                ["0.325", "-0.1", "0.0", "0.0", "1.5708", "0.0", "chassis", "right_steering_hinge"],
            ),
            _static_tf(
                "left_steering_hinge_to_left_front_wheel",
                ["0.0", "0.0", "0.0", "0.0", "0.0", "1.5708", "left_steering_hinge", "left_front_wheel"],
            ),
            _static_tf(
                "right_steering_hinge_to_right_front_wheel",
                ["0.0", "0.0", "0.0", "0.0", "0.0", "1.5708", "right_steering_hinge", "right_front_wheel"],
            ),
        ]
    )
