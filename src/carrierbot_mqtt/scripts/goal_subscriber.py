#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import paho.mqtt.client as mqtt

# --- Config MQTT ---
MQTT_HOST = "45.117.177.157"
MQTT_PORT = 1883
MQTT_KEEPALIVE_INTERVAL = 5
MQTT_USERNAME = "client"
MQTT_PASSWORD = "viam1234"
MQTT_TOPIC = "robot2/goal"

# --- Goal coordinates mapping ---
GOAL_COORDINATES = {
    "DestinationPoint1": {"x": 5.6745, "y": 3.7549, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": -0.01, "qw": 1.0},
    "DestinationPoint2": {"x": 1.0, "y": 2.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.5, "qw": 1.5},
    "DestinationPoint3": {"x": 3.0, "y": 4.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
    # Add more goals here
}

# --- MQTTGoalSubscriber Node ---
class MQTTGoalSubscriber(Node):
    def __init__(self):
        super().__init__('mqtt_goal_subscriber')
        
        # Create ROS publisher for /goal_pose
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.get_logger().info("=== MQTT Goal Subscriber started ===")
        self.get_logger().info(f"Listening to MQTT topic: {MQTT_TOPIC}")
        
        # Create client MQTT
        self.mqttc = mqtt.Client()
        self.mqttc.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        
        # Set callbacks
        self.mqttc.on_connect = self.on_connect
        self.mqttc.on_subscribe = self.on_subscribe
        self.mqttc.on_message = self.on_message
        
        # Connect to broker
        try:
            self.mqttc.connect(MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE_INTERVAL)
            self.get_logger().info(f"Connecting to MQTT broker {MQTT_HOST}:{MQTT_PORT} ...")
        except Exception as e:
            self.get_logger().error(f"Can't connect to MQTT broker: {e}")
            raise
        
        # Create a timer for MQTT loop
        self.create_timer(0.1, self.mqtt_loop_callback)
    
    def on_message(self, mosq, obj, msg):
        try:
            self.get_logger().info(f"=== Received MQTT message from topic: {msg.topic} ===")
            payload = msg.payload.decode("utf-8").strip().strip('"')
            self.get_logger().info(f"Payload: {payload}")

            if payload in GOAL_COORDINATES:
                coords = GOAL_COORDINATES[payload]
                
                # Create PoseStamped message
                goal_pose = PoseStamped()
                goal_pose.header.stamp = self.get_clock().now().to_msg()
                goal_pose.header.frame_id = "map"  # Assuming map frame
                
                goal_pose.pose.position.x = coords["x"]
                goal_pose.pose.position.y = coords["y"]
                goal_pose.pose.position.z = coords["z"]
                
                goal_pose.pose.orientation.x = coords["qx"]
                goal_pose.pose.orientation.y = coords["qy"]
                goal_pose.pose.orientation.z = coords["qz"]
                goal_pose.pose.orientation.w = coords["qw"]
                
                # Publish to /goal_pose
                self.pub.publish(goal_pose)
                self.get_logger().info(f"Published goal to /goal_pose: {payload} -> ({coords['x']:.2f}, {coords['y']:.2f})")
            else:
                self.get_logger().warn(f"Unknown goal: {payload}")

        except Exception as e:
            self.get_logger().error(f"Error handling MQTT message: {e}")

    def on_connect(self, mosq, obj, flags, rc):
        self.get_logger().info(f"Connect to MQTT broker success (rc={rc})")
        mosq.subscribe(MQTT_TOPIC, 0)

    def on_subscribe(self, mosq, obj, mid, granted_qos):
        self.get_logger().info(f"Subscribed to topic: {MQTT_TOPIC}")
    
    def mqtt_loop_callback(self):
        self.mqttc.loop(0.1)  # handle MQTT in loop

# --- Main function ---
def main(args=None):
    rclpy.init(args=args)
    node = MQTTGoalSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()