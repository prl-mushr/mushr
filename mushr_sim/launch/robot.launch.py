from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    # Path to xacro file
    xacro_file = PathJoinSubstitution([
        FindPackageShare('mushr_description'),
        'robots',
        'racecar-mit.urdf.xacro'
    ])

    # Robot description from xacro
    robot_description = {
        'robot_description': Command(['xacro ', xacro_file])
    }

    return LaunchDescription([

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
        ),
    ])
