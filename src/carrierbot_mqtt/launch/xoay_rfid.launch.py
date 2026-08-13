import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='carrierbot_mqtt',
            executable='xoay_subscriber',
            name='mqtt_xoay_subscriber',
            output='screen',
        ),
        launch_ros.actions.Node(
            package='carrierbot_mqtt',
            executable='rfid_publisher',
            name='rfid_mqtt_publisher',
            output='screen',
        ),
    ])
