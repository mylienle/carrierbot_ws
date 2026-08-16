import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ========================
    # Launch Configurations
    # ========================
    use_sim_time = LaunchConfiguration("use_sim_time")
    drive_mode = LaunchConfiguration("drive_mode")
    use_imu = LaunchConfiguration("use_imu")

    channel_type = LaunchConfiguration("channel_type")
    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    frame_id = LaunchConfiguration("frame_id")
    inverted = LaunchConfiguration("inverted")
    angle_compensate = LaunchConfiguration("angle_compensate")
    scan_mode = LaunchConfiguration("scan_mode")

    slam_config = LaunchConfiguration("slam_config")

    # ========================
    # Declare Arguments
    # ========================
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false"
    )

    drive_mode_arg = DeclareLaunchArgument(
        "drive_mode",
        default_value="1",
        description="Robot 1 CAN drive mode",
    )

    use_imu_arg = DeclareLaunchArgument(
        "use_imu",
        default_value="false",
        description="Launch the optional BNO055 I2C driver"
    )

    channel_type_arg = DeclareLaunchArgument(
        "channel_type",
        default_value="serial"
    )

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/lidar"
    )

    serial_baudrate_arg = DeclareLaunchArgument(
        "serial_baudrate",
        default_value="1000000"
    )

    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="laser"
    )

    inverted_arg = DeclareLaunchArgument(
        "inverted",
        default_value="false"
    )

    angle_compensate_arg = DeclareLaunchArgument(
        "angle_compensate",
        default_value="true"
    )

    scan_mode_arg = DeclareLaunchArgument(
        "scan_mode",
        default_value="DenseBoost"
    )

    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=os.path.join(
            get_package_share_directory("carrierbot_slam"),
            "config",
            "mapper_params_online_async.yaml"
        )
    )

    # ========================
    # Robot Description (Xacro)
    # ========================
    urdf_path = os.path.join(
        get_package_share_directory("carrierbot_description"),
        "urdf",
        "my_robot.urdf.xacro"
    )

    robot_description = ParameterValue(
        Command(["xacro ", urdf_path]),
        value_type=str
    )

    # ========================
    # Nodes
    # ========================

    # Robot state publisher (TF)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen"
    )

    hardware_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("carrierbot_firmware"),
                "launch",
                "hardware_interface.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "drive_mode": drive_mode,
        }.items()
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("carrierbot_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items()
    )

    # LiDAR
    lidar = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[{
            "channel_type": channel_type,
            "serial_port": serial_port,
            "serial_baudrate": serial_baudrate,
            "frame_id": frame_id,
            "inverted": inverted,
            "angle_compensate": angle_compensate,
            "scan_mode": scan_mode,
            "use_sim_time": use_sim_time
        }],
        output="screen"
    )

    # IMU
    imu = Node(
        package="imu_bno055",
        executable="bno055_i2c_node",
        name="imu_node",
        namespace="imu",
        output="screen",
        parameters=[
            {"device": "/dev/i2c-1"},
            {"address": 40},
            {"frame_id": "imu"},
        ],
        condition=IfCondition(use_imu),
    )

    # EKF localization for odom -> base_footprint TF
    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("carrierbot_slam"),
                "config",
                "ekf.yaml"
            ),
            {"use_sim_time": use_sim_time},
        ],
    )

    # Laser filter (NHỚ: phải có setup.py entry point)
    laser_filter = Node(
        package="carrierbot_slam",
        executable="laser_filter",
        name="laser_filter",
        output="screen",
        parameters=[{
            "min_range": 0.5,
            "max_range": 12.0,
        }],
        remappings=[
            ("scan", "/scan"),
            ("scan_filtered", "/scan_filtered"),
        ]
    )

    # SLAM Toolbox
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_config],
        remappings=[
            ("scan", "/scan_filtered"),
        ]
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("carrierbot_slam"),
                "rviz",
                "slam_toolbox.rviz"
            )
        ],
        output="screen",
    )

    # ========================
    # Launch Description
    # ========================
    return LaunchDescription([

        # Arguments
        use_sim_time_arg,
        drive_mode_arg,
        use_imu_arg,
        channel_type_arg,
        serial_port_arg,
        serial_baudrate_arg,
        frame_id_arg,
        inverted_arg,
        angle_compensate_arg,
        scan_mode_arg,
        slam_config_arg,

        # 1. TF + control
        robot_state_publisher,
        hardware_interface,
        controller,

        # 2. Sensors
        lidar,
        imu,
        robot_localization,

        # 3. Filter (delay nhẹ)
        TimerAction(
            period=3.0,
            actions=[laser_filter]
        ),

        # 4. SLAM (delay)
        TimerAction(
            period=5.0,
            actions=[slam_toolbox]
        ),

        # 5. RViz
        TimerAction(
            period=7.0,
            actions=[rviz]
        ),
    ])
