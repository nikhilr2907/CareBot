"""
Brings up the mock Nav2 server alongside the WebSocket bridge in one process tree.

Usage:
    ros2 launch mock_nav2_server mock_and_bridge.launch.py
    ros2 launch mock_nav2_server mock_and_bridge.launch.py \\
        config_path:=/care_configs/hospital_3nodes_consumables.json
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mock_share = FindPackageShare("mock_nav2_server")
    bridge_share = FindPackageShare("care_ml_websocket_bridge")

    mock_config = PathJoinSubstitution([mock_share, "config", "default.yaml"])
    bridge_config = PathJoinSubstitution([bridge_share, "config", "bridge_config.yaml"])

    config_path_arg = DeclareLaunchArgument(
        "config_path",
        default_value="/care_configs/ilc_pilot_v1_care_robotics.json",
        description="Path inside the container to the CareRobotics environment JSON.",
    )

    mock_node = Node(
        package="mock_nav2_server",
        executable="mock_nav2_server",
        name="mock_nav2_server",
        output="screen",
        emulate_tty=True,
        parameters=[
            mock_config,
            {"config_path": LaunchConfiguration("config_path")},
        ],
    )

    bridge_node = Node(
        package="care_ml_websocket_bridge",
        executable="care_ml_websocket_bridge",
        name="care_ml_websocket_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[bridge_config],
    )

    return LaunchDescription([
        config_path_arg,
        mock_node,
        bridge_node,
    ])
