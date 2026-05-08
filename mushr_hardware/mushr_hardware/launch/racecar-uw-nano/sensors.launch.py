#!/usr/bin/env python3

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def _load_sensor_config(config_path):
    with open(config_path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _node_params(sensor_config, node_name):
    return sensor_config.get(node_name, {}).get("ros__parameters", {})


def _launch_setup(context, *args, **kwargs):
    car_name = LaunchConfiguration("car_name").perform(context)
    sensors_config = LaunchConfiguration("sensors_config").perform(context)

    sensor_config = _load_sensor_config(sensors_config)
    camera_params = _node_params(sensor_config, "camera")

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
            "camera_namespace": car_name,
            "config_file": "''",
            **{key: str(value) for key, value in camera_params.items()},
        }.items(),
    )

    lidar_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("ydlidar"),
                        "launch",
                        "lidar.launch.py",
                    )
                ),
                launch_arguments={
                    "params_file": os.path.join(
                        get_package_share_directory("ydlidar"),
                        "config",
                        "ydlidar.yaml",
                    )
                }.items(),
            )
        ],
    )

    # push_button_node = Node(
    #     package="push_button_utils",
    #     executable="push_button.py",
    #     name="push_button",
    #     output="screen",
    # )

    return [realsense_launch, lidar_launch] #, push_button_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("racecar_version", default_value="racecar-uw-nano"),
            DeclareLaunchArgument("car_name", default_value="car"),
            DeclareLaunchArgument("tf_prefix", default_value=LaunchConfiguration("car_name")),
            DeclareLaunchArgument(
                "sensors_config",
                default_value=os.path.join(
                    get_package_share_directory("mushr_hardware"),
                    "config",
                    "racecar-uw-nano",
                    "sensors.yaml",
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
