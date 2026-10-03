import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    world = LaunchConfiguration("world")
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="empty.world",
        description="Gazebo Classic world name or absolute world path",
    )

    # Package paths
    my_robot_description_dir = get_package_share_directory("carrierbot_description")
    my_robot_bringup_dir = get_package_share_directory("carrierbot_bringup")
    robot_controller_dir = get_package_share_directory("carrierbot_controller")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    
    # File paths
    urdf_path = os.path.join(my_robot_description_dir, "urdf", "my_robot.urdf.xacro")
    rviz_config_path = os.path.join(my_robot_description_dir, "rviz", "urdf_config.rviz")

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            'robot_description': ParameterValue(
                Command([
                    'xacro ', urdf_path,
                    ' is_sim:=true',
                    ' is_ignition:=False',
                ]),
                value_type=str,
            )
        }]
    )

    # Gazebo Classic (the simulator supported by ROS 2 Foxy)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            'world': world,
        }.items()
    )

    # Spawn Robot
    spawn_robot_node = TimerAction(
        period=2.0,
        actions=[Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-topic", "robot_description", "-entity", "carrierbot"],
            output="screen",
        )]
    )

    # RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen"
    )
    
    # Include Controller Launch File (delayed to ensure robot is spawned first)
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_controller_dir, "launch", "controller.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items()
    )
    
    # Delay controller spawning by 5 seconds to ensure Gazebo and robot are ready
    delayed_controller_launch = TimerAction(
        period=5.0,
        actions=[controller_launch]
    )
    
    return LaunchDescription([
        world_arg,
        robot_state_publisher_node,
        gazebo,
        spawn_robot_node,
        rviz_node,
        delayed_controller_launch,
    ])
