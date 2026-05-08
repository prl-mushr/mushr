from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory('ackermann_cmd_mux'),
        'param',
        'mux.yaml'
    )

    # Container for all composable nodes (replaces nodelet manager)
    container = ComposableNodeContainer(
        name='ackermann_cmd_mux_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='ackermann_cmd_mux',
                plugin='ackermann_cmd_mux::AckermannCmdMuxNode',
                name='ackermann_cmd_mux',
                parameters=[{'yaml_config_file': config_file}]
            )
        ]
    )

    return LaunchDescription([container])
