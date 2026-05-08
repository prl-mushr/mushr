#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_node(context, *args, **kwargs):
    racecar_version = LaunchConfiguration("racecar_version").perform(context)
    vesc_config = os.path.join(
        get_package_share_directory("vesc_main"),
        "config",
        racecar_version,
        "vesc.yaml",
    )

    return [
        Node(
            package="mushr_base",
            executable="racecar_state",
            name="racecar_state",
            output="screen",
            parameters=[
                vesc_config,
                {"update_rate": 20.0},
                {"speed_offset": 0.0},
                {"speed_noise": 0.0001},
                {"steering_angle_offset": 0.0},
                {"steering_angle_noise": 0.000001},
                {"forward_offset": 0.0},
                {"forward_fix_noise": 0.0000001},
                {"forward_scale_noise": 0.001},
                {"side_offset": 0.0},
                {"side_fix_noise": 0.000001},
                {"side_scale_noise": 0.001},
                {"theta_offset": 0.0},
                {"theta_fix_noise": 0.000001},
                {"force_in_bounds": LaunchConfiguration("force_in_bounds")},
                {"tf_prefix": LaunchConfiguration("tf_prefix")},
                {"use_mocap": LaunchConfiguration("use_mocap")},
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("force_in_bounds", default_value="false"),
            DeclareLaunchArgument("tf_prefix", default_value=""),
            DeclareLaunchArgument("use_mocap", default_value="false"),
            DeclareLaunchArgument("racecar_version", default_value="racecar-uw-nano"),
            OpaqueFunction(function=_make_node),
        ]
    )
