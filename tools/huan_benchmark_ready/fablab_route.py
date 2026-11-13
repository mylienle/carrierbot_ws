"""Read Fablab's internal route; construct an explicitly estimated time model."""
import json
import math
import time
import threading

SOURCE = "fablab_internal_waypoints_v1"
TOPIC = "/avoidance_preview/route_state"
WAYPOINT_TOPIC = "/avoidance_preview/waypoints"
PREVIEW_NODE = "fablab_avoidance_preview"
TEST_WAYPOINTS = [[1.0, 0.0], [1.0, 1.0], [5.0, 1.0]]


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def xy(value):
    return isinstance(value, (list, tuple)) and len(value) == 2 and all(finite(v) for v in value)


def clipped_polyline(points, distance):
    result = [list(points[0])]
    for end in points[1:]:
        start = result[-1]
        length = math.hypot(end[0]-start[0], end[1]-start[1])
        if length < 1e-9:
            continue
        if length >= distance:
            result.append([start[i]+(end[i]-start[i])*distance/length for i in range(2)])
            break
        result.append(list(end))
        distance -= length
    return result


def nominal_positions(points, speed, horizon=2.0):
    result = [{"dt_s": 0.0, "xy_m": list(points[0])}]
    if speed <= 0:
        return result + [{"dt_s": horizon, "xy_m": list(points[0])}]
    elapsed = 0.0
    for start, end in zip(points, points[1:]):
        length = math.hypot(end[0]-start[0], end[1]-start[1])
        if length < 1e-9:
            continue
        duration = length/speed
        if elapsed+duration >= horizon:
            fraction = (horizon-elapsed)/duration
            result.append({"dt_s": horizon,
                           "xy_m": [start[i]+fraction*(end[i]-start[i]) for i in range(2)]})
            return result
        elapsed += duration
        result.append({"dt_s": elapsed, "xy_m": list(end)})
    result.append({"dt_s": horizon, "xy_m": list(result[-1]["xy_m"])})
    return result


def route_segments(points):
    segments = []
    for (x, y), (end_x, end_y) in zip(points, points[1:]):
        dx, dy = end_x-x, end_y-y
        segments.append((x, y, dx, dy, dx*dx+dy*dy))
    return segments


def point_in_corridor(center, first, segments, separation):
    x, y = center
    if math.hypot(x-first[0], y-first[1]) <= separation:
        return True
    for start_x, start_y, dx, dy, length2 in segments:
        px, py = x-start_x, y-start_y
        t = max(0.0, min(1.0, (px*dx+py*dy)/length2)) if length2 else 0.0
        if math.hypot(px-t*dx, py-t*dy) <= separation:
            return True
    return False


def intersects_route_corridor(center, points, separation):
    if not xy(center) or not isinstance(points, list) or not points or not all(xy(p) for p in points):
        return None
    return point_in_corridor(center, points[0], route_segments(points), separation)


def route_candidate(snapshot, robot, now, map_to_odom):
    """The geometry is Fablab's; speed/time along it is a nominal assumption."""
    if not isinstance(snapshot, dict) or snapshot.get("source") != SOURCE or snapshot.get("frame_id") != "map":
        raise ValueError("INVALID_ROUTE_SOURCE")
    stamp = snapshot.get("stamp")
    if not finite(stamp) or not finite(now) or not 0 <= now-stamp <= .5:
        raise ValueError("STALE_OR_FUTURE_ROUTE")
    pose_stamp = snapshot.get("pose_tf_stamp")
    if (snapshot.get("active") is not True or snapshot.get("finished") is not False or
            snapshot.get("pose_tf_valid") is not True or not finite(pose_stamp) or
            not 0 <= stamp-pose_stamp <= .3):
        raise ValueError("ROUTE_OR_SOURCE_POSE_NOT_READY")
    points = snapshot.get("waypoints_map")
    index = snapshot.get("current_segment")
    if (not isinstance(points, list) or not 2 <= len(points) <= 2000 or not all(xy(p) for p in points) or
            not isinstance(index, int) or isinstance(index, bool) or not 0 <= index < len(points)-1):
        raise ValueError("INVALID_REMAINING_ROUTE")
    if not all(finite(snapshot.get(k)) and 0 < snapshot[k] <= .2
               for k in ("linear_speed_mps", "linear_speed_max_mps")):
        raise ValueError("UNSUPPORTED_ROUTE_SPEED")
    for flag in ("aligning", "safety_stop", "avoidance_stop"):
        if not isinstance(snapshot.get(flag), bool):
            raise ValueError("INVALID_GUIDANCE_FLAGS")
    if (not isinstance(robot, dict) or not all(finite(robot.get(k)) for k in ("x", "y", "stamp")) or
            not 0 <= now-robot["stamp"] <= .3):
        raise ValueError("CURRENT_ROBOT_POSE_NOT_READY")
    if (not isinstance(map_to_odom, dict) or
            not all(finite(map_to_odom.get(k)) for k in ("x", "y", "yaw", "stamp")) or
            abs(map_to_odom["stamp"]-stamp) > .001):
        raise ValueError("NO_EXACT_ROUTE_TIME_TRANSFORM")
    c, s = math.cos(map_to_odom["yaw"]), math.sin(map_to_odom["yaw"])
    def convert(point):
        return [map_to_odom["x"]+c*point[0]-s*point[1],
                map_to_odom["y"]+s*point[0]+c*point[1]]
    source_pose = snapshot.get("robot_pose_map")
    if not isinstance(source_pose, list) or len(source_pose) != 3 or not all(finite(v) for v in source_pose):
        raise ValueError("INVALID_SOURCE_ROBOT_POSE")
    converted_pose = convert(source_pose[:2])
    if math.hypot(converted_pose[0]-robot["x"], converted_pose[1]-robot["y"]) > .15:
        raise ValueError("ROUTE_AND_CURRENT_POSE_DISAGREE")
    remaining = [[robot["x"], robot["y"]]] + [convert(p) for p in points[index+1:]]
    hold = any(snapshot[k] for k in ("aligning", "safety_stop", "avoidance_stop"))
    speed = 0.0 if hold else min(snapshot["linear_speed_mps"], snapshot["linear_speed_max_mps"])
    return {"frame_id": "odom", "stamp": now, "kind": "fablab_route_nominal_timing",
            "positions": nominal_positions(remaining, speed),
            "route_geometry_odom_m": remaining,
            "route_corridor_xy_m": clipped_polyline(remaining, 3.0),
            "map_to_odom": map_to_odom,
            "route_source": SOURCE, "route_source_stamp": stamp,
            "route_source_sequence": snapshot.get("sequence"), "route_segment": index,
            "route_snapshot": snapshot, "speed_assumption_mps": speed,
            "guidance_hold": hold, "timing_validated": False,
            "timing_model": "hold_while_guidance_stops_or_aligns" if hold else "constant_nominal_speed_along_remaining_polyline",
            "scope": "Internal Fablab route geometry; timing estimate, not measured or guaranteed robot motion."}


class FablabRouteReader:
    def __init__(self, node, buffer, topic=TOPIC, source_node=PREVIEW_NODE, clock=None):
        from std_msgs.msg import String
        self.node, self.buffer = node, buffer
        self.topic, self.source_node = topic, source_node
        self.clock = clock
        self.lock = threading.RLock()
        self.latest, self.wall, self.error = None, 0.0, "NO_ROUTE_MESSAGE"
        self.subscription = node.create_subscription(String, topic, self.receive, 1)

    def receive(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict) or value.get("source") != SOURCE:
                raise ValueError("INVALID_ROUTE_SOURCE")
            with self.lock:
                self.latest, self.wall, self.error = value, time.monotonic(), None
        except (ValueError, TypeError) as error:
            with self.lock:
                self.latest, self.error = None, str(error)

    def message_ready(self):
        with self.lock:
            return self._message_ready()

    def _message_ready(self):
        publishers = self.node.get_publishers_info_by_topic(self.topic)
        now = self.clock() if self.clock is not None else self.node.get_clock().now().nanoseconds/1e9
        return (self.latest is not None and time.monotonic()-self.wall <= 1.0 and
                finite(self.latest.get("stamp")) and 0 <= now-self.latest["stamp"] <= .5 and
                len(publishers) == 1 and publishers[0].node_name == self.source_node)

    def candidate(self, robot, now):
        with self.lock:
            return self._candidate_locked(robot, now)

    def _candidate_locked(self, robot, now):
        from rclpy.time import Time
        from tf2_ros import TransformException
        if not self.message_ready():
            return None, self.error or "ROUTE_STREAM_NOT_READY"
        try:
            stamp = self.latest.get("stamp")
            if not finite(stamp):
                raise ValueError("INVALID_ROUTE_STAMP")
            transform = self.buffer.lookup_transform("odom", "map", Time(nanoseconds=round(stamp*1e9)))
            p, q = transform.transform.translation, transform.transform.rotation
            if (not all(finite(v) for v in (q.x, q.y, q.z, q.w, p.x, p.y)) or
                    abs(q.x) > .001 or abs(q.y) > .001 or
                    abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > .001):
                raise ValueError("NONPLANAR_OR_INVALID_ROUTE_TRANSFORM")
            converted = {"x": p.x, "y": p.y,
                         "yaw": math.atan2(2*q.w*q.z, 1-2*q.z*q.z),
                         "stamp": transform.header.stamp.sec+transform.header.stamp.nanosec/1e9}
            return route_candidate(self.latest, robot, now, converted), None
        except (TransformException, ValueError, TypeError) as error:
            return None, str(error)


def ensure_preview_mission(node, reader, safety_snapshot):
    """Send only waypoints. No velocity publisher or bridge is created."""
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from sensor_alignment_observer import AlignmentAvailability
    def wait_for(predicate):
        deadline = time.monotonic()+15.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            safety = safety_snapshot()
            if AlignmentAvailability.FATAL.intersection(safety["blockers"]):
                raise RuntimeError("Preview route lost zero-command/guard isolation: " + str(safety["blockers"]))
            if safety["ready"] and predicate():
                return
        raise RuntimeError("Route preview not ready. Build/start with --route-preview; see route_state topic.")
    wait_for(reader.message_ready)
    existing = reader.latest.get("waypoints_map", [])
    if existing:
        def includes_mission():
            cursor = 0
            for point in existing:
                if xy(point) and math.dist(point, TEST_WAYPOINTS[cursor]) < .01:
                    cursor += 1
                    if cursor == len(TEST_WAYPOINTS):
                        return True
            return False
        if not includes_mission() or reader.latest.get("finished") is not False:
            raise RuntimeError("A different or partial preview mission exists. Restart this preview session.")
        wait_for(lambda: reader.message_ready() and reader.latest.get("active") is True)
        return {"waypoints_sent_this_run": 0, "mode": "reuse_existing_mission",
                "mission_waypoints_map": TEST_WAYPOINTS}
    publisher = node.create_publisher(PoseStamped, WAYPOINT_TOPIC, 10)
    try:
        def subscriber_ready():
            peers = node.get_subscriptions_info_by_topic(WAYPOINT_TOPIC)
            return len(peers) == 1 and peers[0].node_name == PREVIEW_NODE
        wait_for(subscriber_ready)
        for waypoint in TEST_WAYPOINTS:
            wait_for(reader.message_ready)
            before = reader.latest.get("sequence", -1)
            message = PoseStamped()
            message.header.frame_id = "map"
            message.header.stamp = node.get_clock().now().to_msg()
            message.pose.orientation.w = 1.0
            message.pose.position.x, message.pose.position.y = waypoint
            publisher.publish(message)
            def acknowledged():
                value = reader.latest
                points = value.get("waypoints_map", []) if value else []
                return (reader.message_ready() and value.get("sequence", -1) > before and
                        bool(points) and xy(points[-1]) and math.dist(points[-1], waypoint) < .01)
            wait_for(acknowledged)
        wait_for(lambda: reader.message_ready() and reader.latest.get("active") is True)
    finally:
        node.destroy_publisher(publisher)
    return {"waypoints_sent_this_run": len(TEST_WAYPOINTS), "mode": "sent_once_in_order",
            "mission_waypoints_map": TEST_WAYPOINTS}
