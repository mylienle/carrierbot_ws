"""RGB-D camera for rtabmap_real.launch.py using the ZED SDK (GPU depth).

Same topics, frames and mount arguments as zed_usb2_rgbd.launch.py, so the two
are interchangeable.  Do not run both at the same time.
"""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("depth_mode", default_value="ULTRA",
                              description="NEURAL, ULTRA, QUALITY or PERFORMANCE"),
        DeclareLaunchArgument("max_depth", default_value="3.0"),
        DeclareLaunchArgument(
            "cam_x", default_value="0.10", description="Camera position from base_link, forward (m)"),
        DeclareLaunchArgument(
            "cam_y", default_value="0.06", description="Camera position from base_link, left (m)"),
        DeclareLaunchArgument(
            "cam_z", default_value="1.44", description="Camera position from base_link, up (m)"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="0.0"),
        DeclareLaunchArgument("cam_roll", default_value="0.0"),
    ]

    rgbd = Node(
        package="carrierbot_slam",
        executable="zed_sdk_rgbd",
        name="zed_sdk_rgbd",
        namespace="zed",
        output="screen",
        parameters=[
            {"depth_mode": LaunchConfiguration("depth_mode")},
            {"frame_id": "zed_left_optical_frame"},
            {"max_depth": LaunchConfiguration("max_depth")},
        ],
    )

    base_to_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_zed",
        arguments=[
            LaunchConfiguration("cam_x"),
            LaunchConfiguration("cam_y"),
            LaunchConfiguration("cam_z"),
            LaunchConfiguration("cam_yaw"),
            LaunchConfiguration("cam_pitch"),
            LaunchConfiguration("cam_roll"),
            "base_link",
            "zed_left_camera_frame",
        ],
    )

    # ROS camera frame (x forward, z up) -> optical frame (z forward, x right, y down).
    camera_to_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="zed_to_optical",
        arguments=[
            "0", "0", "0",
            str(-math.pi / 2), "0", str(-math.pi / 2),
            "zed_left_camera_frame",
            "zed_left_optical_frame",
        ],
    )

    return LaunchDescription(arguments + [rgbd, base_to_camera, camera_to_optical])
