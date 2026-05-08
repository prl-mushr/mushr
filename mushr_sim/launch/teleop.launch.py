#!/usr/bin/env python3
"""
Python launch equivalent of the provided XML launch file.

Notes:
- This launch file *includes* other launch files (rosbridge_websocket, map_server, nav_msg_converter, single_car_sim).
- It supports either XML or Python included launch files via AnyLaunchDescriptionSource.
- MAP environment variable takes precedence over the `map` argument (same as your XML).
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def _compute_map_arg(context, *args, **kwargs):
    """
    Implements: (eval _env_map if _env_map else map)
    where _env_map is optenv(MAP).
    """
    env_map = os.environ.get("MAP", "")
    if env_map:
        return env_map
    return LaunchConfiguration("map").perform(context)


def _include_with_map(context, *args, **kwargs):
    map_value = _compute_map_arg(context)

    map_server_include = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mushr_sim"), "launch", "map_server.launch.py"]
            )
        ),
        launch_arguments={"map": map_value}.items(),
    )

    return [map_server_include]


def generate_launch_description():
    # Args (match your XML defaults)
    map_server = LaunchConfiguration("map_server")
    nav_msg_converter = LaunchConfiguration("nav_msg_converter")
    foxglove_teleop = LaunchConfiguration("foxglove_teleop")
    keyboard_teleop = LaunchConfiguration("keyboard_teleop")

    map_arg = LaunchConfiguration("map")
    car_name = LaunchConfiguration("car_name")
    use_tf_prefix = LaunchConfiguration("use_tf_prefix")
    fake_localization = LaunchConfiguration("fake_localization")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_theta = LaunchConfiguration("initial_theta")

    # Includes
    rosbridge_websocket_include = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rosbridge_server"), "launch", "rosbridge_websocket.launch.py"]
            )
        ),
        launch_arguments={"port": "9090"}.items(),
    )

    nav_msg_converter_include = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mushr_base"), "launch", "includes", "nav_msg_converter.launch.py"]
            )
        )
    )

    single_car_sim_include = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mushr_sim"), "launch", "single_car_sim.launch.py"]
            )
        ),
        launch_arguments={
            "racecar_version": "mushr_nano",
            "racecar_color": "",
            "foxglove_teleop": foxglove_teleop,
            "keyboard_teleop": keyboard_teleop,
            "fake_localization": fake_localization,
            "initial_x": initial_x,
            "initial_y": initial_y,
            "initial_theta": initial_theta,
            "use_tf_prefix": use_tf_prefix,
            "car_name": car_name,
        }.items(),
    )

    # Global remap:
    # <remap from="/$(arg car_name)/initialpose" to="/initialpose" />
    # In ROS2 launch, SetRemap applies to nodes launched after it.
    remap_initialpose = SetRemap(
        src=[ "/", car_name, "/initialpose" ],
        dst="/initialpose",
    )

    # Conditional groups (match XML <group if=...>)
    foxglove_group = GroupAction(
        actions=[rosbridge_websocket_include],
        condition=IfCondition(foxglove_teleop),
    )

    map_server_group = GroupAction(
        actions=[
            # Use OpaqueFunction so MAP env var can override `map` arg at runtime
            OpaqueFunction(function=_include_with_map),
        ],
        condition=IfCondition(map_server),
    )

    nav_msg_converter_group = GroupAction(
        actions=[nav_msg_converter_include],
        condition=IfCondition(nav_msg_converter),
    )

    ld = LaunchDescription()

    # Declare args
    ld.add_action(DeclareLaunchArgument("map_server", default_value="1"))
    ld.add_action(DeclareLaunchArgument("nav_msg_converter", default_value="1"))
    ld.add_action(DeclareLaunchArgument("foxglove_teleop", default_value="0"))
    ld.add_action(DeclareLaunchArgument("keyboard_teleop", default_value="0"))

    ld.add_action(DeclareLaunchArgument("map", default_value=PathJoinSubstitution(
        [FindPackageShare("mushr_sim"), "maps", "sandbox.yaml"]
    )))
    ld.add_action(DeclareLaunchArgument("car_name", default_value="car"))
    ld.add_action(DeclareLaunchArgument("use_tf_prefix", default_value="true"))
    ld.add_action(DeclareLaunchArgument("fake_localization", default_value="true"))
    ld.add_action(DeclareLaunchArgument("initial_x", default_value="0"))
    ld.add_action(DeclareLaunchArgument("initial_y", default_value="0"))
    ld.add_action(DeclareLaunchArgument("initial_theta", default_value="0"))

    # Add actions in roughly the same order as your XML
    ld.add_action(foxglove_group)
    ld.add_action(map_server_group)
    ld.add_action(nav_msg_converter_group)

    ld.add_action(remap_initialpose)
    ld.add_action(single_car_sim_include)

    return ld
