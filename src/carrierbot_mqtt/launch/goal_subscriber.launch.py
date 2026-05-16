import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='carrierbot_mqtt',
            executable='goal_subscriber',
            name='mqtt_goal_subscriber',
            output='screen',
        ),
    ])