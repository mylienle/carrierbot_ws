#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray
import paho.mqtt.client as mqtt

# --- Config MQTT ---
MQTT_HOST = "45.117.177.157"
MQTT_PORT = 1883
MQTT_KEEPALIVE_INTERVAL = 5
MQTT_USERNAME = "client"
MQTT_PASSWORD = "viam1234"
MQTT_ARRIVAL_TOPIC = "robot2/arrival"

class GoalStatusMonitor(Node):
    def __init__(self):
        super().__init__('reach_goal')
        
        # Create MQTT client
        self.mqttc = mqtt.Client()
        self.mqttc.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        
        self.mqttc.on_connect = self.on_mqtt_connect
        self.mqttc.on_disconnect = self.on_mqtt_disconnect
        
        # Connect to MQTT broker
        try:
            self.mqttc.connect(MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE_INTERVAL)
            self.get_logger().info(f"🔌 Connecting to MQTT broker {MQTT_HOST}:{MQTT_PORT}")
            self.mqttc.loop_start()
        except Exception as e:
            self.get_logger().error(f"❌ MQTT Error: {e}")
            raise
        
        # Subscribe to goal status topic
        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.status_callback,
            10
        )
        
        self.get_logger().info("✓ Subscribed to /navigate_to_pose/_action/status")
        self.get_logger().info(f"📡 Will publish to: {MQTT_ARRIVAL_TOPIC}")
        
        self.last_status = None
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"✓ MQTT Connected (rc={rc})")
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        if rc != 0:
            self.get_logger().warn(f"⚠️  MQTT Disconnected")
    
    def status_callback(self, msg):
        """Callback when goal status is received"""
        try:
            if len(msg.status_list) > 0:
                latest = msg.status_list[-1]
                
                # Status 4 = SUCCEEDED
                if latest.status == 4 and self.last_status != 4:
                    self.last_status = 4
                    self.get_logger().info(f"🎯 Goal SUCCEEDED (status={latest.status})")
                    self.publish_arrival()
                
                # Reset when new goal starts
                elif latest.status in [1, 2]:  # ACCEPTED or EXECUTING
                    if self.last_status != latest.status:
                        self.last_status = latest.status
                        self.get_logger().info(f"⏳ Goal status: {latest.status}")
        
        except Exception as e:
            self.get_logger().error(f"❌ Error in callback: {e}")
    
    def publish_arrival(self):
        """Publish arrival to MQTT"""
        try:
            if self.mqttc.is_connected():
                self.mqttc.publish(
                    MQTT_ARRIVAL_TOPIC,
                    "true",
                    qos=1,
                    retain=False
                )
                self.get_logger().info(f"✅ Published 'true' to {MQTT_ARRIVAL_TOPIC}")
            else:
                self.get_logger().error("❌ MQTT not connected")
        except Exception as e:
            self.get_logger().error(f"❌ Publish error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GoalStatusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.mqttc.is_connected():
            node.mqttc.disconnect()
        node.mqttc.loop_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

