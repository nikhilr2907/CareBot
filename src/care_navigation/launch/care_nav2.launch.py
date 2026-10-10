#!/usr/bin/env python3
"""
Single launch file to test CareBot navigation in the school-room world.

By default this uses the saved map from care_navigation/maps/school_room.yaml
with map_server + AMCL. Set use_saved_map:=false to fall back to live
slam_toolbox mapping while navigating.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Ensure Gazebo can find ROS plugins
    gazebo_plugin_path = '/opt/ros/humble/lib'
    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = gazebo_plugin_path + ':' + os.environ['GAZEBO_PLUGIN_PATH']
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = gazebo_plugin_path

    # Include workspace install path so Gazebo can resolve package:// mesh URIs
    workspace_model_path = os.path.join(
        os.path.dirname(get_package_share_directory('care_description')))
    gazebo_model_path = workspace_model_path + ':/opt/ros/humble/share'
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path + ':' + os.environ['GAZEBO_MODEL_PATH']
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    # Package directories
    pkg_care_description = get_package_share_directory('care_description')
    pkg_care_navigation = get_package_share_directory('care_navigation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Paths
    urdf_file = os.path.join(pkg_care_description, 'urdf', 'CareBot.urdf')
    world_file = os.path.join(pkg_care_navigation, 'worlds', 'simple_hallway.world')
    nav2_params_file = os.path.join(pkg_care_navigation, 'config', 'nav2_params.yaml')
    docking_params_file = os.path.join(pkg_care_navigation, 'config', 'docking_params.yaml')
    docks_file = os.path.join(pkg_care_navigation, 'config', 'docks.yaml')
    slam_config = os.path.join(pkg_care_navigation, 'config', 'slam_mapping_params.yaml')
    default_map_file = os.path.join(pkg_care_navigation, 'maps', 'school_room.yaml')
    rviz_config_file = os.path.join(pkg_care_navigation, 'rviz', 'care_bot_nav.rviz')
    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gazebo_gui = LaunchConfiguration('gazebo_gui', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_saved_map = LaunchConfiguration('use_saved_map', default='true')
    map_file = LaunchConfiguration('map', default=default_map_file)

    # Server only — the default gazebo.launch.py loads libgazebo_ros_eol_gui.so
    # which segfaults on Humble. Launch gzclient separately.
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'gui': 'false',
            'server': 'true',
        }.items()
    )

    gazebo_client = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['gzclient'],
                output='screen',
                condition=IfCondition(gazebo_gui)
            )
        ]
    )

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'care_bot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # slam_toolbox publishes /map live when testing without a saved map.
    slam_toolbox = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('slam_toolbox'),
                        'launch',
                        'online_async_launch.py'
                    )
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'slam_params_file': slam_config
                }.items(),
                condition=UnlessCondition(use_saved_map)
            )
        ]
    )

    localization = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'map': map_file,
                    'params_file': nav2_params_file,
                    'autostart': 'true',
                    'use_composition': 'False'
                }.items(),
                condition=IfCondition(use_saved_map)
            )
        ]
    )

    # Use navigation_launch.py for planner/controller behavior. /map and
    # map->odom come from either localization_launch.py or slam_toolbox above.
    nav2_navigation = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': nav2_params_file,
                    'autostart': 'true',
                    'use_composition': 'False'
                }.items()
            )
        ]
    )

    # Docking server (delayed to allow Nav2 to finish starting)
    docking_server = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='opennav_docking',
                executable='opennav_docking',
                name='docking_server',
                output='screen',
                parameters=[
                    docking_params_file,
                    {
                        'use_sim_time': use_sim_time,
                        'dock_database': docks_file,
                    },
                ],
            )
        ]
    )

    # RViz (delayed to allow Nav2 to start)
    rviz = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': True}],
                output='screen',
                condition=IfCondition(use_rviz),
                additional_env={
                    'LIBGL_ALWAYS_SOFTWARE': '1',
                    'QT_OPENGL': 'software',
                    'MESA_GL_VERSION_OVERRIDE': '3.3',
                }
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_saved_map', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map_file),

        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_entity,
        slam_toolbox,
        localization,
        nav2_navigation,
        docking_server,
        rviz,
    ])
