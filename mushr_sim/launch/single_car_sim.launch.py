from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from rclpy.parameter import Parameter


def generate_launch_description():
    car_name = LaunchConfiguration("car_name")
    use_tf_prefix = LaunchConfiguration("use_tf_prefix")
    fake_localization = LaunchConfiguration("fake_localization")
    teleop = LaunchConfiguration("teleop")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_theta = LaunchConfiguration("initial_theta")
    foxglove_teleop = LaunchConfiguration("foxglove_teleop")
    keyboard_teleop = LaunchConfiguration("keyboard_teleop")
    racecar_version = LaunchConfiguration("racecar_version")
    racecar_color = LaunchConfiguration("racecar_color")

    mushr_share = FindPackageShare("mushr_sim")

    mushr_sim_yaml = PathJoinSubstitution(
        [mushr_share, "config", "mushr_sim.yaml"]
    )

    sensors_yaml = PathJoinSubstitution(
        [mushr_share, "config", racecar_version, "sensors.yaml"]
    )
    # sensors_yaml = "/home/dbzfan2012/ros2_ws/src/mushr/mushr_sim/config/racecar-mit/sensors.yaml"

    single_car_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [mushr_share, "launch", "single_car.launch.py"]
            )
        ),
        launch_arguments={
            "car_name": car_name,
            "racecar_version": racecar_version,
            "fake_localization": fake_localization,
            "racecar_color": racecar_color,
            "use_tf_prefix": use_tf_prefix,
            "teleop": teleop,
            "foxglove_teleop": foxglove_teleop,
            "keyboard_teleop": keyboard_teleop,
        }.items(),
    )

    group_car = GroupAction(
        [
            #PushRosNamespace("car"),
            single_car_launch,
        ]
    )

    clicked_point = Node(
        package="mushr_sim",
        executable="clicked_point_to_reposition",
        name="clicked_point_to_reposition",
        output="screen",
        remappings=[
            ("/reposition", "/mushr_sim/reposition"),
        ],
    )

    mushr_sim_node = Node(
        package="mushr_sim",
        executable="sim_node",
        name="mushr_sim",
        output="screen",
        parameters=[
            mushr_sim_yaml,
            sensors_yaml,
            {
                "use_tf_prefix": ParameterValue(
                    use_tf_prefix, value_type=bool
                ),
                "car_names": ["car"],
                # Equivalent to ROS1:
                # <param name="$(arg car_name)/initial_x" ... />
                "car.initial_x": ParameterValue(initial_x, value_type=float),
                "car.initial_y": ParameterValue(initial_y, value_type=float),
                "car.initial_theta": ParameterValue(initial_theta, value_type=float),
            },
        ],
        remappings=[
            (
                "/scan",
                [TextSubstitution(text="/"), "car", TextSubstitution(text="/scan")],
            ),
            (
                "/car_pose",
                [TextSubstitution(text="/"), "car", TextSubstitution(text="/car_pose")],
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("car_name", default_value="car"),
            DeclareLaunchArgument("use_tf_prefix", default_value="false"),
            DeclareLaunchArgument("fake_localization", default_value="true"),
            DeclareLaunchArgument("teleop", default_value="true"),
            DeclareLaunchArgument("initial_x", default_value="0"),
            DeclareLaunchArgument("initial_y", default_value="0"),
            DeclareLaunchArgument("initial_theta", default_value="0"),
            DeclareLaunchArgument("foxglove_teleop", default_value="0"),
            DeclareLaunchArgument("keyboard_teleop", default_value="1"),
            DeclareLaunchArgument(
                "racecar_version", default_value="mushr_nano"
            ),
            DeclareLaunchArgument("racecar_color", default_value=""),

            group_car,
            clicked_point,
            mushr_sim_node,
        ]
    )
