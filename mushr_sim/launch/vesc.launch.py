# vesc_sim.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mux_output_topic = LaunchConfiguration("mux_output_topic")
    car_name = LaunchConfiguration("car_name")
    fake_vesc = LaunchConfiguration("fake_vesc")

    # ROS1: default="$(find mushr_sim)/config/vesc.yaml"
    vesc_config = LaunchConfiguration("vesc_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "mux_output_topic",
            default_value="ackermann_cmd",
        ),
        DeclareLaunchArgument(
            "car_name",
            default_value="car",
        ),
        DeclareLaunchArgument(
            "vesc_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("mushr_sim"),
                "config",
                "vesc.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "fake_vesc",
            default_value="true",
            description="Use mushr_sim fake_vesc_driver (sim). Set false to use real vesc_driver hardware.",
        ),

        # ROS1: <rosparam file="$(arg vesc_config)" command="load" />
        # ROS2: pass YAML as parameters to nodes that need it
        Node(
            package="vesc_ackermann",
            executable="ackermann_to_vesc_node",   # confirm actual ROS2 executable name
            name="ackermann_to_vesc",
            namespace="vesc",
            output="screen",
            parameters=[vesc_config],
            remappings=[
                # ROS1: <remap from="ackermann_cmd" to="$(arg mux_output_topic)" />
                ("ackermann_cmd", mux_output_topic),

                # ROS1: speed -> unsmoothed_speed
                ("commands/motor/speed", "commands/motor/unsmoothed_speed"),

                # ROS1: position -> unsmoothed_position
                ("commands/servo/position", "commands/servo/unsmoothed_position"),
            ],
        ),

        Node(
            package="mushr_sim",
            executable="fake_vesc_driver",
            name="vesc_driver",
            namespace="vesc",
            output="screen",
            condition=IfCondition(fake_vesc),
        ),

        Node(
            package="vesc_driver",
            executable="vesc_driver_node",
            name="vesc_driver",
            namespace="vesc",
            output="screen",
            parameters=[vesc_config],
            condition=UnlessCondition(fake_vesc),
        ),

        Node(
            package="mushr_sim",
            executable="throttle_interpolator",   # confirm actual ROS2 executable name
            name="throttle_interpolator",
            namespace="vesc",
            output="screen",
            parameters=[{
                "car_name": car_name,
            }, vesc_config],
        ),
    ])
