"""Shared MQTT broker, topic, and delivery configuration."""

MQTT_HOST = "103.179.190.242"
MQTT_PORT = 1883
MQTT_KEEPALIVE_INTERVAL = 5
MQTT_USERNAME = "client"
MQTT_PASSWORD = "viam1234"

MQTT_TOPICS = {
    "goal": "robot/goal",
    "water_intake": "robot/water_intake",
    "waypoints": "robot/waypoints",
    "arrival": "robot/arrival",
    "location": "robot/location",
    "velocity": "robot/velocity",
    "attendance": "robot/attendance",
    "battery": "robot/battery",
    "telemetry": "robot/telemetry",
    "xoay": "robot/xoay",
}

MQTT_QOS = 0
MQTT_RETAIN = False
