#!/usr/bin/env python3
"""
Python launch equivalent of nav_msg_converter.launch.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Args (match XML defaults)
    start_topic = LaunchConfiguration("start_topic")
    estimate_topic = LaunchConfiguration("estimate_topic")
    goal_topic = LaunchConfiguration("goal_topic")
    car_name = LaunchConfiguration("car_name")
    pose_topic = LaunchConfiguration("pose_topic")
    type_topic = LaunchConfiguration("type_topic")

    nav_msg_converter_node = Node(
        package="mushr_base",
        executable="nav_msg_converter",
        name="nav_msg_converter",
        parameters=[
            {"car_name": car_name},
            {"start_topic": start_topic},
            {"goal_topic": goal_topic},
            {"pose_topic": pose_topic},
            {"estimate_topic": estimate_topic},
            {"type_topic": type_topic},
        ],
    )

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("start_topic", default_value="/mushr_sim/reposition"))
    ld.add_action(DeclareLaunchArgument("estimate_topic", default_value="/pose_estimate"))
    ld.add_action(DeclareLaunchArgument("goal_topic", default_value="/move_base_simple/goal"))
    ld.add_action(DeclareLaunchArgument("car_name", default_value="car"))
    ld.add_action(DeclareLaunchArgument("pose_topic", default_value="/foxglove/pose_stamped"))
    ld.add_action(DeclareLaunchArgument("type_topic", default_value="/foxglove/click_type"))

    ld.add_action(nav_msg_converter_node)

    return ld
