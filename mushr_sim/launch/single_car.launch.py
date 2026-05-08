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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression 
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # --------------------
    # Launch arguments
    # --------------------
    car_name = LaunchConfiguration("car_name")
    use_tf_prefix = LaunchConfiguration("use_tf_prefix")
    fake_localization = LaunchConfiguration("fake_localization")
    teleop = LaunchConfiguration("teleop")
    foxglove_teleop = LaunchConfiguration("foxglove_teleop")
    keyboard_teleop = LaunchConfiguration("keyboard_teleop")
    racecar_version = LaunchConfiguration("racecar_version")
    racecar_color = LaunchConfiguration("racecar_color")

    def make_nodes(context, *args, **kwargs):
        car = context.perform_substitution(car_name)

        tf_prefix_on = context.perform_substitution(
            use_tf_prefix
        ).lower() in ("true", "1", "yes")

        mushr_desc_share = get_package_share_directory("mushr_description")
        version = context.perform_substitution(racecar_version)
        color = context.perform_substitution(racecar_color)

        urdf_path = os.path.join(
            mushr_desc_share,
            "robots",
            f"{version}{color}.urdf.xacro",
        )

        if not os.path.exists(urdf_path):
            raise RuntimeError(f"URDF not found: {urdf_path}")

        with open(urdf_path, "r") as f:
            robot_description_xml = f.read()

        frame_prefix = f"{car}/" if tf_prefix_on else ""

        # Path to xacro file
        xacro_file = PathJoinSubstitution([
            FindPackageShare('mushr_description'),
            'robots',
            'mushr_nano.urdf.xacro'
        ])

        rsp_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path]),
                "frame_prefix": frame_prefix,
                # keep for compatibility if downstream nodes expect it
                "tf_prefix": car if tf_prefix_on else "",
            }],
        )

        fake_loc_container = ComposableNodeContainer(
            name="fake_localization_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="fake_localization_ros2",
                    plugin="fake_localization_ros2::FakeOdomNode",
                    name="fake_localization",
                    remappings=[
                        ("base_pose_ground_truth", f"/{car}/odom"),
                    ],
                    parameters=[{
                        "base_frame_id": (
                            f"{car}/base_footprint" if tf_prefix_on else "base_footprint"
                        ),
                        "odom_frame_id": (
                            f"{car}/odom" if tf_prefix_on else "odom"
                        ),
                        "use_sim_time": True,
                        # "delta_yaw": 0.0,
                    }],
                )
            ],
        )


        mushr_sim_share = get_package_share_directory("mushr_sim")

        foxglove_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    mushr_sim_share,
                    "launch",
                    "foxglove_teleop.launch.py",
                )
            ),
            condition=IfCondition(
                PythonExpression(
                    ["'", foxglove_teleop, "' == '1'"]
                )
            ),
        )

        keyboard_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    mushr_sim_share,
                    "launch",
                    "keyboard_teleop.launch.py",
                )
            ),
            condition=IfCondition(
                PythonExpression(
                    ["'", keyboard_teleop, "' == '1'"]
                )
            ),
        )

        mux_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("ackermann_cmd_mux"),
                    "launch",
                    "mux.launch.py",
                )
            )
        )

        vesc_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    mushr_sim_share,
                    "launch",
                    "vesc.launch.py",
                )
            ),
            launch_arguments={
                "mux_output_topic": f"/{car}/mux/output",
                "car_name": f"{car}",
            }.items(),
        )

        return [
            GroupAction(
                [
                    PushRosNamespace(car_name),
                    GroupAction(
                        [fake_loc_container],
                        condition=IfCondition(
                            PythonExpression(
                                ["'", fake_localization, "' == 'true'"]
                            )
                        ),
                    ),
                    rsp_node,
                    GroupAction(
                        [foxglove_launch],
                        condition=IfCondition(teleop),
                    ),
                    GroupAction(
                        [keyboard_launch],
                        condition=IfCondition(teleop),
                    ),
                    GroupAction([PushRosNamespace("mux"), mux_include]),
                    vesc_include,
                ]
            )
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("car_name", default_value="car"),
            DeclareLaunchArgument("use_tf_prefix", default_value="true"),
            DeclareLaunchArgument("fake_localization", default_value="true"),
            DeclareLaunchArgument("teleop", default_value="true"),
            DeclareLaunchArgument("foxglove_teleop", default_value="0"),
            DeclareLaunchArgument("keyboard_teleop", default_value="1"),
            DeclareLaunchArgument("racecar_version", default_value="mushr_nano"),
            DeclareLaunchArgument("racecar_color", default_value=""),

            OpaqueFunction(function=make_nodes),
        ]
    )
