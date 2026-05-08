#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


RACECAR_XACRO_MAP = {
    "racecar-mit": "racecar-mit.urdf.xacro",
    "racecar-uw-nano": "mushr_nano.urdf.xacro",
    "racecar-uw-tx2": "mushr_tx2.urdf.xacro",
}


def _package_file(package_name, candidates):
    package_share = get_package_share_directory(package_name)
    for candidate in candidates:
        candidate_path = os.path.join(package_share, candidate)
        if os.path.exists(candidate_path):
            return candidate_path
    raise FileNotFoundError(
        f"Unable to find any of {candidates} in package '{package_name}'"
    )


def _include_launch(package_name, candidates, launch_arguments=None, condition=None):
    launch_path = _package_file(package_name, candidates)
    source_cls = (
        PythonLaunchDescriptionSource
        if launch_path.endswith(".launch.py") or launch_path.endswith(".py")
        else AnyLaunchDescriptionSource
    )
    return IncludeLaunchDescription(
        source_cls(launch_path),
        launch_arguments=(launch_arguments or {}).items(),
        condition=condition,
    )


def _make_launch(context, *args, **kwargs):
    car_name = LaunchConfiguration("car_name").perform(context)
    racecar_version = LaunchConfiguration("racecar_version").perform(context)
    use_mocap = LaunchConfiguration("use_mocap")
    foxglove_teleop = LaunchConfiguration("foxglove_teleop")

    xacro_name = RACECAR_XACRO_MAP.get(
        racecar_version, f"{racecar_version}.urdf.xacro"
    )
    xacro_path = os.path.join(
        get_package_share_directory("mushr_description"),
        "robots",
        xacro_name,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(["xacro ", xacro_path]),
                "frame_prefix": f"{car_name}/",
                "tf_prefix": car_name,
            }
        ],
    )

    rosbridge_include = _include_launch(
        "rosbridge_server",
        [
            "launch/rosbridge_websocket.launch.py",
            "launch/rosbridge_websocket_launch.xml",
            "launch/rosbridge_websocket.launch.xml",
            "launch/rosbridge_websocket.launch",
        ],
        {"port": "9090"},
        condition=IfCondition(foxglove_teleop),
    )

    vesc_include = _include_launch(
        "vesc_main",
        [
            "launch/vesc.launch.py",
        ],
        {
            "racecar_version": racecar_version,
            "car_name": car_name,
        },
    )

    teleop_include = _include_launch(
        "mushr_base",
        ["launch/includes/joy_teleop.launch.py"],
        {"car_name": f"/{car_name}"},
    )

    mux_include = _include_launch(
        "ackermann_cmd_mux",
        ["launch/mux.launch.py"],
    )

    sensors_include = _include_launch(
        "mushr_hardware",
        [
            f"launch/{racecar_version}/sensors.launch.py",
            f"launch/{racecar_version}/sensors.launch.xml",
            f"launch/{racecar_version}/sensors.launch",
        ],
        {
            "racecar_version": racecar_version,
            "tf_prefix": car_name,
            "car_name": car_name,
        },
    )

    racecar_state_include = _include_launch(
        "mushr_base",
        ["launch/includes/racecar_state.launch.py"],
        {
            "tf_prefix": car_name,
            "use_mocap": use_mocap,
            "racecar_version": racecar_version,
        },
    )

    nav_msg_converter_include = _include_launch(
        "mushr_base",
        ["launch/includes/nav_msg_converter.launch.py"],
        {"car_name": car_name},
    )

    return [
        rosbridge_include,
        GroupAction(
            [
                PushRosNamespace(car_name),
                GroupAction([PushRosNamespace("vesc"), vesc_include]),
                GroupAction([PushRosNamespace("teleop"), teleop_include]),
                GroupAction([PushRosNamespace("mux"), mux_include]),
                sensors_include,
                racecar_state_include,
                nav_msg_converter_include,
                robot_state_publisher,
            ]
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("car_name", default_value="car"),
            DeclareLaunchArgument("racecar_version", default_value="racecar-uw-nano"),
            DeclareLaunchArgument("use_mocap", default_value="false"),
            DeclareLaunchArgument("foxglove_teleop", default_value="0"),
            OpaqueFunction(function=_make_launch),
        ]
    )
