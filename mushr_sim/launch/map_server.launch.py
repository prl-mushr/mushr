import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnProcessStart


def generate_launch_description():
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map')
    }])

    map_lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
    }])

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('mushr_sim'),
                'maps',
                'sandbox.yaml',
            ]),
        ),
        map_server_node,
        # map_lifecycle_manager_node,
        TimerAction(period=8.0, actions=[map_lifecycle_manager_node]),
    ])
