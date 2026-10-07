"""Launch RTAB-Map against the topics provided by the physical robot.

The RGB-D camera driver is intentionally not launched here: its package and
topic names depend on the camera selected for the Xavier.  Start the driver
first, then override the topic arguments below if it does not use the common
``/rgb`` and ``/depth`` topic contract used by the simulation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("frame_id", default_value="base_footprint"),
        DeclareLaunchArgument("odom_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument("rgb_topic", default_value="/rgb/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/depth/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/rgb/camera_info"),
        DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
        DeclareLaunchArgument("scan_topic", default_value="/scan_filtered"),
        DeclareLaunchArgument(
            "database_path",
            default_value=PathJoinSubstitution(
                [EnvironmentVariable("HOME"), "carrierbot_ws", "maps", "rtabmap_real.db"]
            ),
        ),
        DeclareLaunchArgument("rtabmap_viz", default_value="true"),
    ]

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("rtabmap_launch"),
                "/launch/rtabmap.launch.py",
            ]
        ),
        launch_arguments={
            "use_sim_time": "false",
            "frame_id": LaunchConfiguration("frame_id"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "rgb_topic": LaunchConfiguration("rgb_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "subscribe_scan": "true",
            "scan_topic": LaunchConfiguration("scan_topic"),
            "visual_odometry": "false",
            "approx_sync": "true",
            "wait_imu_to_init": "false",
            "database_path": LaunchConfiguration("database_path"),
            "rtabmap_args": "--delete_db_on_start --Grid/FromDepth true",
            "rtabmap_viz": LaunchConfiguration("rtabmap_viz"),
            "rviz": "false",
        }.items(),
    )

    return LaunchDescription(arguments + [rtabmap_launch])
