#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt

# --- Config MQTT (same as v1 MQTT/name_publisher.py) ---
MQTT_HOST = "45.117.177.157"
MQTT_PORT = 1883
MQTT_KEEPALIVE_INTERVAL = 5
MQTT_USERNAME = "client"
MQTT_PASSWORD = "viam1234"
MQTT_TOPIC = "robot/attendance"


class RFIDPublisher(Node):
    """ROS /carrierbot/rfid (from firmware CAN 0x019) → MQTT robot/attendance."""

    def __init__(self):
        super().__init__('rfid_mqtt_publisher')

        self.create_subscription(String, '/carrierbot/rfid', self.on_rfid, 10)
        self.get_logger().info("=== RFID MQTT Publisher started ===")
        self.get_logger().info(f"Publishing attendance to MQTT topic: {MQTT_TOPIC}")

        self.mqttc = mqtt.Client()
        self.mqttc.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.mqttc.on_connect = self.on_connect

        try:
            self.mqttc.connect(MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE_INTERVAL)
            self.get_logger().info(f"Connecting to MQTT broker {MQTT_HOST}:{MQTT_PORT} ...")
            self.mqttc.loop_start()
        except Exception as e:
            self.get_logger().error(f"Can't connect to MQTT broker: {e}")
            raise

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"Connect to MQTT broker success (rc={rc})")

    def on_rfid(self, msg: String):
        try:
            payload = msg.data.strip()
            data = json.loads(payload)
            if not isinstance(data, dict):
                raise ValueError("RFID payload is not a JSON object")

            out = json.dumps(data)
            result = self.mqttc.publish(MQTT_TOPIC, out)
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                self.get_logger().info(
                    f"Published attendance: user={data.get('user')} "
                    f"message={data.get('message')} time={data.get('time')}"
                )
            else:
                self.get_logger().error(f"MQTT publish failed rc={result.rc}")
        except Exception as e:
            self.get_logger().error(f"Error publishing RFID attendance: {e}")

    def destroy_node(self):
        try:
            self.mqttc.loop_stop()
            self.mqttc.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RFIDPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
