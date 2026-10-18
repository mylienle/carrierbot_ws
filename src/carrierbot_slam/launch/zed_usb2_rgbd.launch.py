"""RGB-D camera for rtabmap_real.launch.py using a ZED 2 on USB 2.0.

Starts v4l2_camera for the raw side-by-side stereo stream, zed_usb2_rgbd for
rectification + StereoSGBM depth, and the static TF from base_link to the
camera.  cam_x/cam_y/cam_z default to the measured mount (meters, base_link
frame: x forward, y left, z up).
"""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_device = LaunchConfiguration("video_device")
    calibration_file = LaunchConfiguration("calibration_file")
    max_depth = LaunchConfiguration("max_depth")

    arguments = [
        DeclareLaunchArgument(
            "video_device",
            description="ZED 2 video device, check with: v4l2-ctl --list-devices",
        ),
        DeclareLaunchArgument(
            "calibration_file", default_value="/usr/local/zed/settings/SN29068780.conf"
        ),
        DeclareLaunchArgument("max_depth", default_value="3.0"),
        # Left lens on the robot head: 1.54 m above the floor, 6 cm left of center
        # (half the ZED 2 baseline).  cam_x is an estimate.  The head is slanted
        # so the camera looks up (negative pitch): ~12.6 deg, fitted from the
        # floor in the B1 RTAB-Map depth images.
        DeclareLaunchArgument(
            "cam_x", default_value="0.10", description="Camera position from base_link, forward (m)"),
        DeclareLaunchArgument(
            "cam_y", default_value="0.06", description="Camera position from base_link, left (m)"),
        DeclareLaunchArgument(
            "cam_z", default_value="1.44", description="Camera position from base_link, up (m)"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="-0.22"),
        DeclareLaunchArgument("cam_roll", default_value="0.0"),
    ]

    v4l2_camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="zed_v4l2_camera",
        namespace="zed",
        output="screen",
        parameters=[
            {"video_device": video_device},
            {"pixel_format": "YUYV"},
            {"output_encoding": "rgb8"},
            {"image_size": [1344, 376]},
        ],
        remappings=[("image_raw", "stereo/image_raw")],
    )

    rgbd = Node(
        package="carrierbot_slam",
        executable="zed_usb2_rgbd",
        name="zed_usb2_rgbd",
        namespace="zed",
        output="screen",
        parameters=[
            {"calibration_file": calibration_file},
            {"frame_id": "zed_left_optical_frame"},
            {"max_depth": max_depth},
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

    return LaunchDescription(arguments + [v4l2_camera, rgbd, base_to_camera, camera_to_optical])
