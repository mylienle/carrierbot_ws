#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import paho.mqtt.client as mqtt

# --- Config MQTT (same as v1 subscribe_mqtt/xoay_subscriber.py) ---
MQTT_HOST = "45.117.177.157"
MQTT_PORT = 1883
MQTT_KEEPALIVE_INTERVAL = 5
MQTT_USERNAME = "client"
MQTT_PASSWORD = "viam1234"
MQTT_TOPIC = "robot/xoay"


class MQTTXoaySubscriber(Node):
    """MQTT robot/xoay → ROS /carrierbot/rotate_angle → firmware CAN 0x040."""

    def __init__(self):
        super().__init__('mqtt_xoay_subscriber')

        self.pub = self.create_publisher(Float32, '/carrierbot/rotate_angle', 10)
        self.get_logger().info("=== MQTT Xoay Subscriber started ===")
        self.get_logger().info(f"Listening to MQTT topic: {MQTT_TOPIC}")

        self.mqttc = mqtt.Client()
        self.mqttc.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.mqttc.on_connect = self.on_connect
        self.mqttc.on_subscribe = self.on_subscribe
        self.mqttc.on_message = self.on_message

        try:
            self.mqttc.connect(MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE_INTERVAL)
            self.get_logger().info(f"Connecting to MQTT broker {MQTT_HOST}:{MQTT_PORT} ...")
        except Exception as e:
            self.get_logger().error(f"Can't connect to MQTT broker: {e}")
            raise

        self.create_timer(0.1, self.mqtt_loop_callback)

    def on_message(self, mosq, obj, msg):
        try:
            payload = msg.payload.decode("utf-8").strip()
            self.get_logger().info(f"=== Received MQTT {msg.topic}: {payload} ===")

            try:
                data = json.loads(payload)
                if isinstance(data, dict) and "angle" in data:
                    angle_value = float(data["angle"])
                else:
                    angle_value = float(payload)
            except (json.JSONDecodeError, TypeError, ValueError):
                angle_value = float(payload)

            angle_msg = Float32()
            angle_msg.data = float(angle_value)
            self.pub.publish(angle_msg)
            self.get_logger().info(f"Published /carrierbot/rotate_angle = {angle_msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error handling MQTT xoay message: {e}")

    def on_connect(self, mosq, obj, flags, rc):
        self.get_logger().info(f"Connect to MQTT broker success (rc={rc})")
        mosq.subscribe(MQTT_TOPIC, 0)

    def on_subscribe(self, mosq, obj, mid, granted_qos):
        self.get_logger().info(f"Subscribed to topic: {MQTT_TOPIC}")

    def mqtt_loop_callback(self):
        self.mqttc.loop(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = MQTTXoaySubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
