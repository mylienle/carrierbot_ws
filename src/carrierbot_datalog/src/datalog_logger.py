#!/usr/bin/env python3

import csv
import datetime
import math
import os

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from carrierbot_msgs.msg import CarrierbotTelemetry

def make_default_csv_path():
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    log_dir = '/home/nvidia/carrierbot_ws/src/carrierbot_datalog/data'
    return os.path.join(log_dir, f'amcl_odom_log_{timestamp}.csv')


def format_value(value):
    if value is None:
        return ''
    if isinstance(value, float) and not math.isfinite(value):
        return 'nan'
    return f'{value:.4f}'


class DataLogger(Node):
    def __init__(self):
        super().__init__('carrierbot_datalogger')

        self.declare_parameter('amcl_topic', '/amcl_pose')
        self.declare_parameter('filtered_odom_topic', '/odometry/filtered')
        self.declare_parameter('telemetry_topic', '/carrierbot/telemetry')
        self.declare_parameter('csv_path', make_default_csv_path())
        self.declare_parameter('publish_rate', 2.0)

        self.amcl_x = None
        self.amcl_y = None
        self.amcl_qz = None
        self.amcl_qw = None
        self.filtered_vx = None
        self.filtered_wz = None
        self.left_velocity = None
        self.right_velocity = None
        self.left_rps = None
        self.right_rps = None
        self.voltage = None
        self.relative_time = 0.0

        self.csv_path = os.path.expanduser(str(self.get_parameter('csv_path').value))
        csv_dir = os.path.dirname(self.csv_path)
        if csv_dir:
            os.makedirs(csv_dir, exist_ok=True)

        self.get_logger().info(f'CSV path: {self.csv_path}')

        self.csv_file = open(self.csv_path, 'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        if self.csv_file.tell() == 0:
            self.csv_writer.writerow([
                'time_s',
                'amcl_x',
                'amcl_y',
                'amcl_qz',
                'amcl_qw',
                'filtered_vx',
                'filtered_wz',
                'left_velocity_cmd',
                'right_velocity_cmd',
                'left_velocity_encoder',
                'right_velocity_encoder',
                'voltage',
            ])
            self.csv_file.flush()

        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('amcl_topic').value,
            self.amcl_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('filtered_odom_topic').value,
            self.filtered_odom_callback,
            10,
        )
        self.create_subscription(
            CarrierbotTelemetry,
            self.get_parameter('telemetry_topic').value,
            self.telemetry_callback,
            10,
        )

        publish_rate = float(self.get_parameter('publish_rate').value)
        period = 1.0 / publish_rate if publish_rate > 0.0 else 0.1
        self.time_increment = 1.0 / publish_rate if publish_rate > 0.0 else 0.1
        self.create_timer(period, self.write_row)

        self.get_logger().info(f'Data logger started, writing to {self.csv_path}')

    def amcl_callback(self, msg):
        pose = msg.pose.pose
        self.amcl_x = pose.position.x
        self.amcl_y = pose.position.y
        self.amcl_qx = pose.orientation.x
        self.amcl_qy = pose.orientation.y
        self.amcl_qz = pose.orientation.z
        self.amcl_qw = pose.orientation.w

    def filtered_odom_callback(self, msg):
        twist = msg.twist.twist
        self.filtered_vx = twist.linear.x
        self.filtered_wz = twist.angular.z

    def telemetry_callback(self, msg):
        self.left_velocity = msg.left_velocity
        self.right_velocity = msg.right_velocity
        self.left_rps = msg.left_rps
        self.right_rps = msg.right_rps
        self.voltage = msg.voltage

    def write_row(self):
        self.csv_writer.writerow([
            f'{self.relative_time:.1f}',
            format_value(self.amcl_x),
            format_value(self.amcl_y),
            format_value(self.amcl_qz),
            format_value(self.amcl_qw),
            format_value(self.filtered_vx),
            format_value(self.filtered_wz),
            format_value(self.left_velocity),
            format_value(self.right_velocity),
            format_value(self.left_rps),
            format_value(self.right_rps),
            format_value(self.voltage),
        ])
        self.csv_file.flush()
        self.relative_time += self.time_increment


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if getattr(node, 'csv_file', None) is not None:
            node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()