#!/usr/bin/env python3
"""Bridge the Robot1 MQTT protocol to Nav2 actions.

The reception app publishes a *name* on ``robot/goal`` first. Its own graph
planner then expands that name into a map-coordinate route and publishes the
route on ``robot/waypoints``. Only that route is a movement command in the
Robot1 protocol, so this node intentionally subscribes only to that topic.
"""

import json
import math

import paho.mqtt.client as mqtt
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

DUPLICATE_START_THRESHOLD_M = 0.3

from .mqtt_config import (
    MQTT_HOST,
    MQTT_KEEPALIVE_INTERVAL,
    MQTT_PASSWORD,
    MQTT_PORT,
    MQTT_QOS,
    MQTT_TOPICS,
    MQTT_USERNAME,
)

MQTT_WAYPOINTS_TOPIC = MQTT_TOPICS["waypoints"]
MQTT_ARRIVAL_TOPIC = MQTT_TOPICS["arrival"]

GOAL_COORDINATES = {
    "DestinationPoint1": {"x": 5.6745, "y": 3.7549, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": -0.01, "qw": 1.0},
    "DestinationPoint2": {"x": -9.502463883942948, "y": 3.604801781709745, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 1.0, "qw": 0.0},
    "DestinationPoint3": {"x": 1.4895777244658523, "y": 1.1899790896165576, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 1.0, "qw": 0.0},
    "DestinationPoint4": {"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 1.0, "qw": 0.0},
    # B1 GUI destinations. The GUI's B1_config_wp.json uses pixels on
    # new_map2 (1675 x 1039), resolution 0.05 m/px and origin
    # [-23.371523, -12.247928]. These are the corresponding map-frame
    # coordinates. Keep the older DestinationPoint entries above for
    # backward compatibility with existing clients.
    "Table_1": {"x": 19.828477, "y": 15.302072, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
    "Restroom": {"x": 12.478477, "y": 14.452072, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
    "WaterIntake": {"x": 13.628477, "y": 26.252072, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
    "Water intake": {"x": 13.628477, "y": 26.252072, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
    "Home": {"x": 17.878477, "y": 20.452072, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
    "Chemistry hall": {"x": 19.128477, "y": 26.252072, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
    "Robotics lab": {"x": -0.171523, "y": 0.102072, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
    "Stairs": {"x": 17.628477, "y": 3.352072, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
    "Table2": {"x": 23.128477, "y": 24.452072, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
}

class MQTTGoalSubscriber(Node):
    """Receive app commands and use the matching Nav2 action."""

    def __init__(self):
        super().__init__('mqtt_goal_subscriber')
        self.navigate_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.active_goal_handle = None
        self.mission_id = 0
        self.pending_mission = None
        self.current_pose = None
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.amcl_pose_callback, 10)

        self.mqttc = mqtt.Client()
        self.mqttc.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.mqttc.on_connect = self.on_connect
        self.mqttc.on_subscribe = self.on_subscribe
        self.mqttc.on_message = self.on_message
        try:
            self.mqttc.connect(MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE_INTERVAL)
        except Exception as error:
            self.get_logger().error(
                f'Cannot connect to MQTT broker {MQTT_HOST}:{MQTT_PORT}: {error}')
            raise
        self.create_timer(0.1, self.mqtt_loop_callback)
        self.create_timer(0.2, self.dispatch_pending_mission)
        self.get_logger().info('MQTT Nav2 bridge started.')
        self.get_logger().info(
            f'Robot1-compatible MQTT protocol: movement route={MQTT_WAYPOINTS_TOPIC}')

    def on_connect(self, client, _userdata, _flags, rc):
        self.get_logger().info(f'Connected to MQTT broker (rc={rc})')
        client.subscribe(MQTT_WAYPOINTS_TOPIC, MQTT_QOS)
        self.get_logger().info(f'Subscribed to: {MQTT_WAYPOINTS_TOPIC}')

    def amcl_pose_callback(self, msg):
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def on_subscribe(self, _client, _userdata, mid, _granted_qos):
        self.get_logger().debug(f'MQTT subscription acknowledged (mid={mid})')

    def on_message(self, _client, _userdata, msg):
        payload = msg.payload.decode('utf-8').strip()
        self.get_logger().info(f'Received MQTT {msg.topic}: {payload}')
        try:
            if msg.topic == MQTT_WAYPOINTS_TOPIC:
                poses = self.parse_waypoints(payload)
                if poses:
                    self.queue_mission(poses)
        except Exception as error:
            self.get_logger().error(f'MQTT navigation command failed: {error}')

    def parse_waypoints(self, raw_payload):
        """Accept the Robot1 list or one-waypoint payload formats."""
        try:
            decoded = json.loads(raw_payload)
        except json.JSONDecodeError:
            self.get_logger().warning('robot/waypoints must contain valid JSON.')
            return None

        if isinstance(decoded, dict) and 'x' in decoded and 'y' in decoded:
            waypoints = [decoded]
        else:
            waypoints = decoded
        if not isinstance(waypoints, list) or not waypoints:
            self.get_logger().warning(
                'robot/waypoints requires a non-empty list of map poses.')
            return None

        waypoints = self.drop_duplicate_start_waypoint(waypoints)

        poses = []
        has_explicit_orientation = []
        for index, waypoint in enumerate(waypoints):
            if not isinstance(waypoint, dict):
                self.get_logger().warning(f'Waypoint {index} is not an object.')
                return None
            pose = self.pose_from_coordinates(waypoint)
            if pose is None:
                self.get_logger().warning(
                    f'Waypoint {index} requires numeric x and y in map meters.')
                return None
            poses.append(pose)
            has_explicit_orientation.append(
                'yaw' in waypoint or
                all(key in waypoint for key in ('qx', 'qy', 'qz', 'qw')))

        # A waypoint is a position constraint first.  When the app does not
        # supply an orientation, align it with the next app-selected segment
        # so the goal checker does not make the robot rotate to yaw=0 at every
        # intermediate waypoint.
        for index, pose in enumerate(poses):
            if has_explicit_orientation[index] or len(poses) == 1:
                continue
            source = pose.pose.position
            if index > 0:
                prev = poses[index - 1].pose.position
                dx, dy = source.x - prev.x, source.y - prev.y
            else:
                target = poses[index + 1].pose.position
                dx, dy = target.x - source.x, target.y - source.y
            self.set_pose_yaw(pose, math.atan2(dy, dx))
        return poses

    def drop_duplicate_start_waypoint(self, waypoints):
        """Drop a leading waypoint that just duplicates the robot's current position."""
        if len(waypoints) <= 1 or self.current_pose is None:
            return waypoints
        first = waypoints[0]
        if not isinstance(first, dict):
            return waypoints
        try:
            fx = float(first['x'])
            fy = float(first['y'])
        except (KeyError, TypeError, ValueError):
            return waypoints
        cx, cy = self.current_pose
        dist = math.hypot(fx - cx, fy - cy)
        if dist < DUPLICATE_START_THRESHOLD_M:
            self.get_logger().warning(
                f'Dropping first waypoint as duplicate of current position '
                f'(dist={dist:.3f} m): ({fx:.3f}, {fy:.3f})')
            return waypoints[1:]
        return waypoints

    def pose_from_coordinates(self, coordinates):
        try:
            x = float(coordinates['x'])
            y = float(coordinates['y'])
            z = float(coordinates.get('z', 0.0))
        except (KeyError, TypeError, ValueError):
            return None

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        if all(key in coordinates for key in ('qx', 'qy', 'qz', 'qw')):
            pose.pose.orientation.x = float(coordinates['qx'])
            pose.pose.orientation.y = float(coordinates['qy'])
            pose.pose.orientation.z = float(coordinates['qz'])
            pose.pose.orientation.w = float(coordinates['qw'])
        else:
            self.set_pose_yaw(pose, float(coordinates.get('yaw', 0.0)))
        return pose

    @staticmethod
    def set_pose_yaw(pose, yaw):
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

    def begin_mission(self):
        self.mission_id += 1
        self.pending_mission = None
        if self.active_goal_handle is not None:
            self.active_goal_handle.cancel_goal_async()
            self.get_logger().info('Cancelled previous MQTT mission.')
        self.active_goal_handle = None
        return self.mission_id

    def queue_mission(self, poses):
        mission_id = self.begin_mission()
        self.pending_mission = (mission_id, poses)
        self.get_logger().info(
            f'Queued MQTT mission with {len(poses)} waypoint(s); '
            'waiting for NavigateToPose.')

    def dispatch_pending_mission(self):
        if self.pending_mission is None:
            return

        mission_id, poses = self.pending_mission
        if not self.navigate_client.server_is_ready():
            return

        self.pending_mission = None
        self.send_waypoint(poses, mission_id, 0)

    def send_waypoint(self, poses, mission_id, waypoint_index):
        pose = poses[waypoint_index]
        goal = NavigateToPose.Goal()
        goal.pose = pose
        future = self.navigate_client.send_goal_async(goal)
        future.add_done_callback(
            lambda result, identifier=mission_id, index=waypoint_index:
            self.waypoint_goal_response(result, identifier, poses, index))
        self.get_logger().info(
            f'Sent waypoint {waypoint_index + 1}/{len(poses)}: '
            f'({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f})')

    def waypoint_goal_response(self, future, mission_id, poses, waypoint_index):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(f'Waypoint request failed: {error}')
            return
        if not goal_handle.accepted:
            self.get_logger().error(
                f'Nav2 rejected waypoint {waypoint_index + 1}/{len(poses)}.')
            return
        if mission_id != self.mission_id:
            goal_handle.cancel_goal_async()
            return

        self.active_goal_handle = goal_handle
        self.get_logger().info(
            f'Nav2 accepted waypoint {waypoint_index + 1}/{len(poses)}.')
        goal_handle.get_result_async().add_done_callback(
            lambda result, identifier=mission_id, index=waypoint_index:
            self.waypoint_result_callback(result, identifier, poses, index))

    def waypoint_result_callback(self, future, mission_id, poses, waypoint_index):
        if mission_id != self.mission_id:
            return
        try:
            status = future.result().status
            self.get_logger().info(
                f'Waypoint {waypoint_index + 1}/{len(poses)} finished with '
                f'status={status}.')
            if status != GoalStatus.STATUS_SUCCEEDED:
                self.active_goal_handle = None
                return
            if waypoint_index + 1 == len(poses):
                self.publish_arrival()
                self.active_goal_handle = None
            else:
                self.active_goal_handle = None
                self.send_waypoint(poses, mission_id, waypoint_index + 1)
        except Exception as error:
            self.get_logger().error(f'Waypoint result failed: {error}')
            self.active_goal_handle = None

    def publish_arrival(self):
        result = self.mqttc.publish(
            MQTT_ARRIVAL_TOPIC, 'true', qos=MQTT_QOS, retain=False)
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            self.get_logger().info(f'Published true to {MQTT_ARRIVAL_TOPIC}')
        else:
            self.get_logger().error(
                f'MQTT arrival publish failed (rc={result.rc})')

    def mqtt_loop_callback(self):
        self.mqttc.loop(0.1)


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
