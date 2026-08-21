#!/usr/bin/env python3
"""Publish measured wheel velocities using the ROS 1 MQTT contract."""

import json

import paho.mqtt.client as mqtt
import rclpy
from carrierbot_msgs.msg import CarrierbotTelemetry
from rclpy.node import Node


MQTT_HOST = "45.117.177.157"
MQTT_PORT = 1883
MQTT_KEEPALIVE_INTERVAL = 5
MQTT_USERNAME = "client"
MQTT_PASSWORD = "viam1234"
MQTT_TOPIC = "robot/velocity"


class VelocityPublisher(Node):
    """ROS encoder telemetry -> MQTT robot/velocity at 2 Hz."""

    def __init__(self):
        super().__init__("mqtt_velocity_publisher")
        self._latest_telemetry = None

        self.create_subscription(
            CarrierbotTelemetry,
            "/carrierbot/telemetry",
            self._telemetry_callback,
            10,
        )
        self.create_timer(0.5, self._publish_velocity)

        self.mqttc = mqtt.Client()
        self.mqttc.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.mqttc.on_connect = self._on_connect
        self.mqttc.on_disconnect = self._on_disconnect
        try:
            self.mqttc.connect(MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE_INTERVAL)
            self.mqttc.loop_start()
            self.get_logger().info(
                "MQTT velocity publisher started: /carrierbot/telemetry -> "
                "robot/velocity (2 Hz)"
            )
        except Exception as error:
            self.get_logger().error(f"Can't connect to MQTT broker: {error}")

    def _on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"Connected to MQTT broker (rc={rc})")

    def _on_disconnect(self, client, userdata, rc):
        if rc != 0:
            self.get_logger().warn("MQTT velocity publisher disconnected")

    def _telemetry_callback(self, msg):
        self._latest_telemetry = msg

    def _publish_velocity(self):
        if self._latest_telemetry is None or not self.mqttc.is_connected():
            return

        payload = json.dumps(
            {
                "left": self._latest_telemetry.left_velocity,
                "right": self._latest_telemetry.right_velocity,
            }
        )
        result = self.mqttc.publish(MQTT_TOPIC, payload, qos=0, retain=False)
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            self.get_logger().error(f"MQTT velocity publish failed rc={result.rc}")

    def destroy_node(self):
        try:
            self.mqttc.loop_stop()
            self.mqttc.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
