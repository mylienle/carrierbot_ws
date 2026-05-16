from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='carrierbot_mqtt',
            executable='reach_goal',
            name='reach_goal_mqtt',
            output='screen',
        ),
    ])
