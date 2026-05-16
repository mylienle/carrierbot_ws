#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
import paho.mqtt.client as mqtt

# --- Config MQTT ---
MQTT_HOST = "45.117.177.157"
MQTT_PORT = 1883
MQTT_KEEPALIVE_INTERVAL = 5
MQTT_USERNAME = "client"
MQTT_PASSWORD = "viam1234"
MQTT_TOPIC_SUB = "robot2/goal"
MQTT_TOPIC_PUB = "robot/arrival"

# --- Goal coordinates mapping ---
GOAL_COORDINATES = {
    "home": {"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
    "point1": {"x": 1.0, "y": 2.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
    "point2": {"x": 3.0, "y": 4.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
    # Add more goals here
}

# Global MQTT client for publishing
mqtt_client = None

# --- Callback function when received MQTT message ---
def on_message(mosq, obj, msg):
    try:
        rospy.loginfo("=== Received MQTT message from topic: %s ===", msg.topic)
        payload = msg.payload.decode("utf-8")
        rospy.loginfo("Payload: %s", payload)

        if payload in GOAL_COORDINATES:
            coords = GOAL_COORDINATES[payload]

            # Create PoseStamped message
            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = "map"  # Assuming map frame

            goal_pose.pose.position.x = coords["x"]
            goal_pose.pose.position.y = coords["y"]
            goal_pose.pose.position.z = coords["z"]

            goal_pose.pose.orientation.x = coords["qx"]
            goal_pose.pose.orientation.y = coords["qy"]
            goal_pose.pose.orientation.z = coords["qz"]
            goal_pose.pose.orientation.w = coords["qw"]

            # Publish to /goal_pose
            pub.publish(goal_pose)
            rospy.loginfo("Published goal to /goal_pose: %s -> (%.2f, %.2f)", payload, coords["x"], coords["y"])

            # Send "true" to MQTT topic robot/arrival
            if mqtt_client:
                mqtt_client.publish(MQTT_TOPIC_PUB, "true", qos=0)
                rospy.loginfo("Sent 'true' to MQTT topic: %s", MQTT_TOPIC_PUB)

        else:
            rospy.logwarn("Unknown goal: %s", payload)

    except Exception as e:
        rospy.logerr("Error handling MQTT message: %s", e)

# --- Callback when connect MQTT ---
def on_connect(mosq, obj, flags, rc):
    rospy.loginfo("Connect to MQTT broker success (rc=%s)", str(rc))
    mosq.subscribe(MQTT_TOPIC_SUB, 0)

# --- Callback when subscribe ---
def on_subscribe(mosq, obj, mid, granted_qos):
    rospy.loginfo("Subscribed to topic: %s", MQTT_TOPIC_SUB)

# --- Main function ---
if __name__ == '__main__':
    rospy.init_node('mqtt_goal_subscriber', anonymous=True)

    # Create ROS publisher for /goal_pose
    pub = rospy.Publisher('/goal_pose', PoseStamped, queue_size=10)
    rospy.loginfo("=== MQTT Goal Subscriber started ===")
    rospy.loginfo("Listening to MQTT topic: %s", MQTT_TOPIC_SUB)

    # Create MQTT client
    mqtt_client = mqtt.Client()
    mqtt_client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

    # Set callbacks
    mqtt_client.on_connect = on_connect
    mqtt_client.on_subscribe = on_subscribe
    mqtt_client.on_message = on_message

    # Connect to broker
    try:
        mqtt_client.connect(MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE_INTERVAL)
        rospy.loginfo("Connecting to MQTT broker %s:%d ...", MQTT_HOST, MQTT_PORT)
    except Exception as e:
        rospy.logerr("Can't connect to MQTT broker: %s", e)
        exit(1)

    # --- Running ROS + MQTT ---
    while not rospy.is_shutdown():
        mqtt_client.loop(0.1)  # handle MQTT in loop
        rospy.sleep(0.1)