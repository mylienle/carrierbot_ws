"""Run the GUI -> MQTT -> LOS-PD flow against a Gazebo Carrierbot on B1."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('carrierbot_bringup')
    map_file = LaunchConfiguration('map')
    world_file = LaunchConfiguration('world')
    mqtt_enabled = LaunchConfiguration('mqtt_enabled')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_share, 'maps', 'new_map2.yaml'),
        description='B1 occupancy-grid YAML, used for RViz visualization.',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Gazebo world. The default keeps the B1 behavioral test obstacle-free.',
    )
    mqtt_enabled_arg = DeclareLaunchArgument(
        'mqtt_enabled',
        default_value='false',
        description='Enable production MQTT GUI I/O. Keep false while a real robot is online.',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'my_robot_gazebo.launch.py')),
        launch_arguments={'world': world_file}.items(),
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file, 'use_sim_time': True}],
    )
    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='map_lifecycle_manager',
        output='screen',
        parameters=[{'autostart': True, 'node_names': ['map_server'], 'use_sim_time': True}],
    )
    simulated_pose = Node(
        package='carrierbot_bringup',
        executable='simulated_b1_pose.py',
        name='simulated_b1_pose',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    guidance = Node(
        package='carrierbot_navigation',
        executable='fablab_guidance_node',
        name='fablab_guidance_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    mqtt_bridge = Node(
        package='carrierbot_mqtt',
        executable='fablab_waypoint_bridge',
        name='fablab_waypoint_bridge',
        output='screen',
        condition=IfCondition(mqtt_enabled),
    )
    mqtt_location = Node(
        package='carrierbot_mqtt',
        executable='location_publisher',
        name='mqtt_location_publisher',
        output='screen',
        condition=IfCondition(mqtt_enabled),
    )

    return LaunchDescription([
        map_arg,
        world_arg,
        mqtt_enabled_arg,
        gazebo,
        map_server,
        map_lifecycle,
        TimerAction(period=6.0, actions=[simulated_pose]),
        # The Gazebo launch spawns ros2_control after the robot model exists.
        # Start the guidance/bridge only after odometry is available, otherwise
        # a first MQTT route could be anchored at the default (0, 0) pose.
        TimerAction(period=8.0, actions=[guidance, mqtt_bridge, mqtt_location]),
    ])
