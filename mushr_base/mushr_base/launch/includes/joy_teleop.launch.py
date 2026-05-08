#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _make_nodes(context, *args, **kwargs):
    car_name = LaunchConfiguration("car_name").perform(context)
    joy_config = LaunchConfiguration("joy_config").perform(context)
    joy_teleop_config = LaunchConfiguration("joy_teleop_config").perform(context)

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[joy_config],
    )

    joy_teleop_node = Node(
        package="mushr_base",
        executable="joy_teleop",
        name="joy_teleop",
        output="screen",
        parameters=[
            {
                "car_name": car_name,
                "teleop_config": joy_teleop_config,
            }
        ],
    )

    return [joy_node, joy_teleop_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "joy_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("mushr_base"), "config", "joy_node.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "joy_teleop_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("mushr_base"), "config", "joy_teleop.yaml"]
                ),
            ),
            DeclareLaunchArgument("car_name", default_value="/car"),
            OpaqueFunction(function=_make_nodes),
        ]
    )
