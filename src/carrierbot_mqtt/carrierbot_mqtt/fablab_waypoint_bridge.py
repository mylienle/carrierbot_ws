#!/usr/bin/env python3
"""Bridge the Robot Fablab MQTT waypoint protocol to the ROS 2 guidance port."""

import json

import paho.mqtt.client as mqtt
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Int32

from .mqtt_config import (
    MQTT_HOST,
    MQTT_KEEPALIVE_INTERVAL,
    MQTT_PASSWORD,
    MQTT_PORT,
    MQTT_QOS,
    MQTT_TOPICS,
    MQTT_USERNAME,
)


class FablabWaypointBridge(Node):
    """Publish one ROS waypoint message per MQTT route element.

    This intentionally mirrors robot_fablab_ws: the guidance node owns its
    waypoint list and progresses it itself. It does not send Nav2 actions.
    """

    def __init__(self):
        super().__init__('fablab_waypoint_bridge')
        self.waypoint_publisher = self.create_publisher(PoseStamped, '/fablab_waypoints', 100)
        self.create_subscription(Bool, '/fablab_arrival', self.arrival_callback, 10)
        # GUI mode button: 1 = gamepad, 2 = autonomous (firmware sends CAN 0x020, guidance pauses)
        self.drive_mode_publisher = self.create_publisher(Int32, '/carrierbot/drive_mode', 10)

        # Arrival that could not be sent while the broker link was down; sent on reconnect
        self.arrival_pending = False

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_disconnect = self.on_disconnect
        self.mqtt_client.on_message = self.on_message
        # loop_start() reconnects by itself after a Wi-Fi drop; the old manual loop() never did,
        # so after one drop no waypoint reached the robot and no arrival reached the GUI.
        # connect_async: a broker not reachable yet at launch is retried instead of killing the node
        self.mqtt_client.reconnect_delay_set(min_delay=1, max_delay=5)
        self.mqtt_client.connect_async(MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE_INTERVAL)
        self.mqtt_client.loop_start()

        self.get_logger().info(
            f'Fablab waypoint bridge ready: MQTT {MQTT_TOPICS["waypoints"]} -> /fablab_waypoints')

    def on_connect(self, client, _userdata, _flags, result_code):
        self.get_logger().info(f'Connected to MQTT broker (rc={result_code})')
        client.subscribe(MQTT_TOPICS['waypoints'], MQTT_QOS)
        client.subscribe(MQTT_TOPICS['drive_mode'], MQTT_QOS)
        if self.arrival_pending:
            self.publish_arrival()

    def on_disconnect(self, _client, _userdata, result_code):
        if result_code != 0:
            self.get_logger().warning(f'MQTT broker connection lost (rc={result_code}); reconnecting')

    def on_message(self, _client, _userdata, message):
        if message.topic == MQTT_TOPICS['drive_mode']:
            self.handle_drive_mode(message.payload)
            return
        try:
            decoded = json.loads(message.payload.decode('utf-8'))
        except (UnicodeDecodeError, json.JSONDecodeError):
            self.get_logger().warning('robot/waypoints must contain valid JSON.')
            return

        waypoints = [decoded] if isinstance(decoded, dict) else decoded
        if not isinstance(waypoints, list) or not waypoints:
            self.get_logger().warning('robot/waypoints requires a non-empty waypoint list.')
            return

        for index, waypoint in enumerate(waypoints):
            try:
                waypoint_x = float(waypoint['x'])
                waypoint_y = float(waypoint['y'])
            except (KeyError, TypeError, ValueError):
                self.get_logger().warning(f'Ignoring invalid waypoint {index}.')
                return

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = waypoint_x
            pose.pose.position.y = waypoint_y
            pose.pose.orientation.w = 1.0
            self.waypoint_publisher.publish(pose)
            self.get_logger().info(
                f'Published Fablab waypoint {index + 1}/{len(waypoints)}: '
                f'({waypoint_x:.3f}, {waypoint_y:.3f})')

    def handle_drive_mode(self, payload):
        text = payload.decode('utf-8', errors='replace').strip()
        if text not in ('1', '2'):
            self.get_logger().warning(f'robot/drive_mode must be 1 or 2, got {text!r}')
            return
        self.drive_mode_publisher.publish(Int32(data=int(text)))
        self.get_logger().info(f'Drive mode {text} requested from MQTT')

    def arrival_callback(self, message):
        if not message.data:
            return
        self.publish_arrival()

    def publish_arrival(self):
        result = self.mqtt_client.publish(MQTT_TOPICS['arrival'], 'true', qos=MQTT_QOS, retain=False)
        self.arrival_pending = result.rc != mqtt.MQTT_ERR_SUCCESS
        if self.arrival_pending:
            self.get_logger().error(
                f'Could not publish arrival (rc={result.rc}); will resend on reconnect')
        else:
            self.get_logger().info(f'Published arrival to {MQTT_TOPICS["arrival"]}')


def main(args=None):
    rclpy.init(args=args)
    node = FablabWaypointBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt_client.loop_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
