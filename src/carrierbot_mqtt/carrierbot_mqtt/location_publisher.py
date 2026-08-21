#!/usr/bin/env python3
"""Publish the AMCL pose to MQTT at a fixed rate.

This keeps the ROS 2 telemetry contract compatible with the ROS 1 project:
topic ``robot/location`` and payload ``{"x": ..., "y": ..., "theta": ...}``,
where theta is in degrees.
"""

import json
import math

import paho.mqtt.client as mqtt
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

from .mqtt_config import (
    MQTT_HOST,
    MQTT_KEEPALIVE_INTERVAL,
    MQTT_PASSWORD,
    MQTT_PORT,
    MQTT_QOS,
    MQTT_RETAIN,
    MQTT_TOPICS,
    MQTT_USERNAME,
)

MQTT_TOPIC = MQTT_TOPICS["location"]


class LocationPublisher(Node):
    """ROS /amcl_pose -> MQTT robot/location, published every 0.5 seconds."""

    def __init__(self):
        super().__init__("mqtt_location_publisher")

        self._latest_pose = None
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._amcl_pose_callback,
            10,
        )
        self.create_timer(0.5, self._publish_location)

        self.mqttc = mqtt.Client()
        self.mqttc.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.mqttc.on_connect = self._on_connect
        self.mqttc.on_disconnect = self._on_disconnect

        try:
            self.mqttc.connect(MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE_INTERVAL)
            self.mqttc.loop_start()
            self.get_logger().info(
                "MQTT location publisher started: /amcl_pose -> robot/location (2 Hz)"
            )
        except Exception as error:
            self.get_logger().error(f"Can't connect to MQTT broker: {error}")

    def _on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"Connected to MQTT broker (rc={rc})")

    def _on_disconnect(self, client, userdata, rc):
        if rc != 0:
            self.get_logger().warn("MQTT location publisher disconnected")

    def _amcl_pose_callback(self, msg):
        self._latest_pose = msg.pose.pose

    def _publish_location(self):
        if self._latest_pose is None:
            return
        if not self.mqttc.is_connected():
            return

        position = self._latest_pose.position
        orientation = self._latest_pose.orientation
        yaw_rad = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        payload = json.dumps(
            {
                "x": position.x,
                "y": position.y,
                "theta": math.degrees(yaw_rad),
            }
        )
        result = self.mqttc.publish(
            MQTT_TOPIC, payload, qos=MQTT_QOS, retain=MQTT_RETAIN
        )
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            self.get_logger().error(f"MQTT location publish failed rc={result.rc}")

    def destroy_node(self):
        try:
            self.mqttc.loop_stop()
            self.mqttc.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LocationPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
