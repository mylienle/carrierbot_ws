import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    drive_mode = LaunchConfiguration("drive_mode")
    
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
    )

    drive_mode_arg = DeclareLaunchArgument(
        "drive_mode",
        default_value="1",
        description="Robot 1 CAN drive mode (robot_fablab_ws uses 1)",
    )

    robot_description = ParameterValue(Command([
        "xacro ",
        os.path.join(
            get_package_share_directory("carrierbot_description"), 
            "urdf", 
            "my_robot.urdf.xacro"
        ),
        " is_sim:=", use_sim_time,
        " drive_mode:=", drive_mode
    ]),
    value_type=str
)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
            },
            os.path.join(
                get_package_share_directory("carrierbot_controller"),
                "config",
                "controller.yaml"
            )
        ],
        remappings=[
            ("/carrierbot_controller/cmd_vel_unstamped", "/cmd_vel")
        ]
    )
    return LaunchDescription([
        use_sim_time_arg,
        drive_mode_arg,
        robot_state_publisher_node,
        controller_manager
    ])
