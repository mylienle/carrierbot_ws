#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import paho.mqtt.client as mqtt
import rclpy
from action_msgs.msg import GoalStatusArray
from rclpy.node import Node

MQTT_HOST = "45.117.177.157"
MQTT_PORT = 1883
MQTT_TOPIC = "robot2/arrival"
MQTT_USERNAME = "client"
MQTT_PASSWORD = "viam1234"
NAV2_STATUS_TOPIC = "/navigate_to_pose/_action/status"
STATUS_SUCCEEDED = 4


class ActionStatusMqtt(Node):
    def __init__(self):
        super().__init__('action_status_mqtt')
        self._last_succeeded_goal_id = None
        self._mqtt_lock = threading.Lock()

        self._mqtt = mqtt.Client()
        self._mqtt.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self._mqtt.connect(MQTT_HOST, MQTT_PORT, 5)
        self._mqtt.loop_start()

        self._sub = self.create_subscription(
            GoalStatusArray,
            NAV2_STATUS_TOPIC,
            self._on_status,
            10,
        )

        self.get_logger().info(
            f'Listening on {NAV2_STATUS_TOPIC} and publishing MQTT to {MQTT_TOPIC}'
        )

    def _publish_arrival_once(self, goal_id_hex: str) -> None:
        with self._mqtt_lock:
            if goal_id_hex == self._last_succeeded_goal_id:
                return
            self._last_succeeded_goal_id = goal_id_hex
            self._mqtt.publish(MQTT_TOPIC, 'arrival', qos=0)
            self.get_logger().info(f'Published MQTT arrival for goal_id={goal_id_hex}')

    def _on_status(self, msg: GoalStatusArray) -> None:
        for status in msg.status_list:
            if status.status != STATUS_SUCCEEDED:
                continue

            goal_id_hex = ''.join(f'{byte:02x}' for byte in status.goal_info.goal_id.uuid)
            self._publish_arrival_once(goal_id_hex)

    def destroy_node(self):
        try:
            self._mqtt.loop_stop()
            self._mqtt.disconnect()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ActionStatusMqtt()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
