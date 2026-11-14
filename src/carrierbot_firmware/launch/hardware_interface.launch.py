import os
import glob
import tempfile
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    drive_mode = LaunchConfiguration("drive_mode")
    can_port = LaunchConfiguration("can_port")

    can_port_default = next(
        iter(sorted(glob.glob("/dev/serial/by-id/usb-1a86_USB_Serial-*"))),
        "/dev/usbcan",
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
    )

    drive_mode_arg = DeclareLaunchArgument(
        "drive_mode",
        default_value="1",
        description="Robot 1 CAN drive mode (robot_fablab_ws uses 1)",
    )

    can_port_arg = DeclareLaunchArgument(
        "can_port",
        default_value=can_port_default,
        description="Serial device used by the USB-CAN adapter",
    )

    can_port_log = LogInfo(msg=["Using USB-CAN serial device: ", can_port])

    robot_description = ParameterValue(Command([
        "xacro ",
        os.path.join(
            get_package_share_directory("carrierbot_description"), 
            "urdf", 
            "my_robot.urdf.xacro"
        ),
        " is_sim:=", use_sim_time,
        " drive_mode:=", drive_mode,
        " can_port:=", can_port,
    ]),
    value_type=str
)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # Both launches that include this one (real_robot, slam_toolbox) run the EKF, which publishes
    # odom->base_footprint. A second publisher from the diff drive controller (enable_odom_tf: true,
    # e.g. a local edit kept for mapping) makes the pose flip between two yaws every cycle, so the
    # controller TF is always switched off here, whatever controller.yaml says.
    with open(os.path.join(
            get_package_share_directory("carrierbot_controller"), "config", "controller.yaml")) as f:
        controller_config = yaml.safe_load(f)
    controller_config["carrierbot_controller"]["ros__parameters"]["enable_odom_tf"] = False
    with tempfile.NamedTemporaryFile(
            "w", prefix="carrierbot_controller_", suffix=".yaml", delete=False) as f:
        yaml.safe_dump(controller_config, f)
        controller_params = f.name

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
            },
            controller_params
        ],
        remappings=[
            ("/carrierbot_controller/cmd_vel_unstamped", "/cmd_vel")
        ]
    )
    return LaunchDescription([
        use_sim_time_arg,
        drive_mode_arg,
        can_port_arg,
        can_port_log,
        robot_state_publisher_node,
        controller_manager
    ])
