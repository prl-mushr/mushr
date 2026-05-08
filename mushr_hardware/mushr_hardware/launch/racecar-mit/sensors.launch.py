#!/usr/bin/env python3

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_sensor_config(config_path):
    with open(config_path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _node_params(sensor_config, node_name):
    return sensor_config.get(node_name, {}).get("ros__parameters", {})


def _launch_setup(context, *args, **kwargs):
    sensors_config = LaunchConfiguration("sensors_config").perform(context)
    sensor_config = _load_sensor_config(sensors_config)

    camera_params = _node_params(sensor_config, "camera")
    laser_params = _node_params(sensor_config, "laser_node")

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py",
            )
        ),
        launch_arguments={
            "camera_name": "camera",
            "camera_namespace": "camera",
            "config_file": "''",
            **{key: str(value) for key, value in camera_params.items()},
        }.items(),
    )

    laser_node = Node(
        package="urg_node",
        executable="urg_node",
        name="laser_node",
        output="screen",
        parameters=[laser_params],
    )

    return [realsense_launch, laser_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sensors_config",
                default_value=os.path.join(
                    get_package_share_directory("mushr_hardware"),
                    "config",
                    "racecar-mit",
                    "sensors.yaml",
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
