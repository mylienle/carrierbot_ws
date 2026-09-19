#!/usr/bin/env python3
"""Expose Gazebo odometry as B1 map-frame localization for guidance tests."""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(orientation):
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))


def quaternion_from_yaw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class SimulatedB1Pose(Node):
    """Anchor the simulator's odom origin at a selected B1 map coordinate."""

    def __init__(self):
        super().__init__('simulated_b1_pose')
        self.initial_x = self.declare_parameter('initial_x', 17.878477).value
        self.initial_y = self.declare_parameter('initial_y', 20.452072).value
        self.initial_yaw = self.declare_parameter('initial_yaw', 0.0).value
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value

        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)

        self.get_logger().info(
            'B1 simulation localization: odom origin = map (%.3f, %.3f, %.3f)',
            self.initial_x, self.initial_y, self.initial_yaw)

    def odom_callback(self, message):
        odom_pose = message.pose.pose
        cos_yaw = math.cos(self.initial_yaw)
        sin_yaw = math.sin(self.initial_yaw)

        map_x = self.initial_x + cos_yaw * odom_pose.position.x - sin_yaw * odom_pose.position.y
        map_y = self.initial_y + sin_yaw * odom_pose.position.x + cos_yaw * odom_pose.position.y
        map_yaw = self.initial_yaw + yaw_from_quaternion(odom_pose.orientation)
        qz, qw = quaternion_from_yaw(map_yaw)

        pose = PoseWithCovarianceStamped()
        pose.header = message.header
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = map_x
        pose.pose.pose.position.y = map_y
        pose.pose.pose.orientation.z = qz
        pose.pose.pose.orientation.w = qw
        pose.pose.covariance[0] = 0.01
        pose.pose.covariance[7] = 0.01
        pose.pose.covariance[35] = 0.02
        self.pose_publisher.publish(pose)

        # The simulated diff-drive controller has odom TF disabled in its
        # shared configuration. Publish it here, together with map -> odom,
        # so RViz can show the robot on the B1 map.
        map_to_odom = TransformStamped()
        map_to_odom.header = message.header
        map_to_odom.header.frame_id = 'map'
        map_to_odom.child_frame_id = message.header.frame_id or 'odom'
        map_to_odom.transform.translation.x = self.initial_x
        map_to_odom.transform.translation.y = self.initial_y
        map_to_odom.transform.rotation.z, map_to_odom.transform.rotation.w = quaternion_from_yaw(
            self.initial_yaw)

        odom_to_base = TransformStamped()
        odom_to_base.header = message.header
        odom_to_base.header.frame_id = message.header.frame_id or 'odom'
        odom_to_base.child_frame_id = message.child_frame_id or self.base_frame
        odom_to_base.transform.translation.x = odom_pose.position.x
        odom_to_base.transform.translation.y = odom_pose.position.y
        odom_to_base.transform.translation.z = odom_pose.position.z
        odom_to_base.transform.rotation = odom_pose.orientation
        self.tf_broadcaster.sendTransform([map_to_odom, odom_to_base])


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedB1Pose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
