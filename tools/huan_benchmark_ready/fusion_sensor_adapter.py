"""Read measured scan/TF and Huân packets. No Gazebo truth or actuator publisher."""
from collections import deque
import json
import math
import time
import threading

from fablab_route import FablabRouteReader
from fusion_yield_policy import raw_route_corridor, release_intent
from sensor_pair_geometry import associate, fit_circle, scan_clusters, transform_xy


def stamp(message):
    return message.sec+message.nanosec/1e9


def select_transformed_scan(history, now, wall, lookup, transform_errors):
    """Newest fresh scan whose exact-time TF has arrived; never use latest TF.

    A scan callback can run before the TF callback for the same stamp. Keep
    both freshness limits unchanged when trying the preceding buffered scan.
    """
    skipped = []
    for received_wall, scan in reversed(history):
        t = stamp(scan.header.stamp)
        if not -.02 <= now-t <= .3 or not 0 <= wall-received_wall <= .6:
            skipped.append({"scan_stamp": t, "reason": "SCAN_NOT_FRESH"})
            continue
        try:
            p, q, _ = lookup("odom", scan.header.frame_id, t)
            r, _, _ = lookup("odom", "base_footprint", t)
            return scan, p, q, r, received_wall, skipped
        except transform_errors as error:
            skipped.append({"scan_stamp": t, "reason": str(error)})
    raise ValueError("NO_FRESH_TIME_MATCHED_SCAN: "+json.dumps(skipped[:3]))


def project_scan(scan, position, rotation, robot_at_scan):
    points, returns = [], 0
    for i, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
            continue
        returns += 1
        angle = scan.angle_min+i*scan.angle_increment
        point = transform_xy([distance*math.cos(angle), distance*math.sin(angle)], position, rotation)
        if math.dist(point, robot_at_scan) > .60:
            points.append(point)
    return points, returns


def match_blocking_arc(scan, position, rotation, hits, center, radius, point_count):
    groups = scan_clusters(scan.ranges, scan.angle_min, scan.angle_increment,
                           scan.range_min, min(scan.range_max, 10.), gap=.16)
    matches = []
    for group in groups:
        if len(group) != point_count:
            continue
        fit = fit_circle(group)
        if (fit is not None and fit["fit_rms_m"] <= .025 and abs(fit["radius_m"]-radius) <= .025 and
                math.dist(transform_xy(fit["center"], position, rotation), center) <= .025):
            matches.append(group)
    if len(matches) != 1:
        return None
    arc = {tuple(transform_xy(point, position, rotation)) for point in matches[0]}
    matched = [point for point in hits if tuple(point) in arc and math.dist(point, center) <= radius+.04]
    return matched if len(matched) == len(hits) else None


class SensorTimeWindow:
    """No goal before 0.5 sim s of valid input with at least three scans."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.since = None
        self.stamps = set()

    def observe(self, now, scan_stamp):
        if self.since is None or now < self.since:
            self.reset()
            self.since = now
        self.stamps.add(scan_stamp)
        return now-self.since >= .5 and len(self.stamps) >= 3


class FusionSensors:
    def __init__(self, node, buffer, output, clock=None, fusion=False, yield_clearance_m=None):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        self.node, self.buffer, self.output = node, buffer, output
        self.clock = clock
        self.yield_clearance_m = yield_clearance_m
        self.input_lock = threading.RLock()
        self.huan_scan = None
        self.huan_scan_stamp = None
        if fusion:
            from huan_fusion_bridge import HuanFusionBridge
            self.huan = HuanFusionBridge()
        else:
            self.huan = None
        self.scans = deque(maxlen=120)
        self.projected_scan = None
        self.blocking_returns = None
        self.scan_wall = 0.
        self.packet, self.packet_wall, self.packets = None, 0., 0
        self.matches, self.positions, self.velocities = set(), set(), set()
        self.current_matches = {}
        self.last_logged = None
        self.route = FablabRouteReader(node, buffer, "/behavior_gate/route_state", "fablab_behavior_guidance", clock=clock)
        self.subscriptions = [
            node.create_subscription(LaserScan, "/scan", self.receive_scan, qos_profile_sensor_data),
            node.create_subscription(String, "/vision/tracks", self.receive_vision, 1)]

    def receive_scan(self, message):
        with self.input_lock:
            self.scan_wall = time.monotonic()
            self.scans.append((self.scan_wall, message))

    def receive_vision(self, message):
        try:
            packet = json.loads(message.data)
            if not isinstance(packet, dict) or not isinstance(packet.get("tracks"), list):
                return
            with self.input_lock:
                self.packet, self.packet_wall = packet, time.monotonic()
                self.current_matches = {}
                self.packets += 1
                self.output.write(json.dumps({"received_sim": self.now(), "packet": packet})+"\n")
                self.output.flush()
        except (ValueError, TypeError):
            return

    def now(self):
        return self.clock() if self.clock is not None else self.node.get_clock().now().nanoseconds/1e9

    def transform(self, target, child, seconds=None):
        from rclpy.time import Time
        t = Time() if seconds is None else Time(nanoseconds=round(seconds*1e9))
        value = self.buffer.lookup_transform(target, child, t)
        p, q = value.transform.translation, value.transform.rotation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[3:])-1.) > .001:
            raise ValueError("INVALID_TF")
        actual = stamp(value.header.stamp)
        if seconds is not None and abs(actual-seconds) > .001:
            raise ValueError("NO_EXACT_SENSOR_TIME_TF")
        if seconds is None and not -.02 <= self.now()-actual <= .3:
            raise ValueError("STALE_CURRENT_ROBOT_TF frame=%s now=%.6f stamp=%.6f age=%.6f" %
                             (target, self.now(), actual, self.now()-actual))
        return [p.x, p.y], [q.x, q.y, q.z, q.w], actual

    def robot(self, frame="odom"):
        p, q, t = self.transform(frame, "base_footprint")
        return {"x": p[0], "y": p[1], "yaw": math.atan2(2*(q[3]*q[2]+q[0]*q[1]),
                1-2*(q[1]*q[1]+q[2]*q[2])), "stamp": t}

    def intent(self):
        robot = self.robot()
        plan, error = self.route.candidate(robot, self.now())
        if plan is None:
            raise ValueError(error)
        return release_intent(plan), robot

    def raw_corridor(self, intent, robot):
        from tf2_ros import TransformException
        with self.input_lock:
            history, scan_wall = tuple(self.scans), self.scan_wall
        if not history or time.monotonic()-scan_wall > .6:
            raise ValueError("SCAN_MISSING_OR_WALL_STALE")
        scan, p, q, r, received_wall, skipped = select_transformed_scan(
            history, self.now(), time.monotonic(), self.transform, (ValueError, TransformException))
        t = stamp(scan.header.stamp)
        geometry = (tuple(p), tuple(q), tuple(r))
        cached = self.projected_scan
        # Reuse only the projection. Freshness, exact-time TF and the current route are checked each tick.
        if cached is None or cached[0] is not scan or cached[1] != geometry:
            cached = self.projected_scan = (scan, geometry, project_scan(scan, p, q, r))
        points, returns = cached[2]
        blocking_hits = []
        result = raw_route_corridor(points, [robot["x"], robot["y"]], intent["route_corridor_xy_m"], returns,
                                    yield_clearance_m=self.yield_clearance_m, blocking_hits=blocking_hits)
        self.blocking_returns = (t, blocking_hits)
        result.update(stamp=t, received_age_wall_s=time.monotonic()-received_wall,
                      skipped_newer_scans=skipped, exact_sensor_time_tf=True)
        if result.get("blocked") and "map_to_odom" in intent:
            result["blocking_evidence"]["map_to_odom"] = intent["map_to_odom"]
        if self.huan is not None and t != self.huan_scan_stamp:
            from huan_fusion_bridge import measured_scan
            if abs(q[0]) > .001 or abs(q[1]) > .001:
                raise ValueError("HUAN_FUSION_REQUIRES_PLANAR_LASER")
            yaw = math.atan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]*q[1]+q[2]*q[2]))
            self.huan_scan = measured_scan(scan.ranges, scan.angle_min, scan.angle_increment,
                                           scan.range_min, scan.range_max, [p[0], p[1], yaw], t, received_wall)
            self.huan_scan_stamp = t
        return result

    def matched_blocking_returns(self, scan_stamp, center, radius, point_count):
        """Return exact projected hits only when a single measured arc matches."""
        if (self.blocking_returns is None or self.projected_scan is None or
                self.blocking_returns[0] != scan_stamp):
            return None
        scan, (position, rotation, _), _ = self.projected_scan
        return match_blocking_arc(scan, position, rotation, self.blocking_returns[1], center, radius, point_count)

    def fused(self, intent, robot):
        if self.huan is None:
            return None
        with self.input_lock:
            packet, receipt = self.packet, self.packet_wall
        return self.huan.update(packet, receipt, self.huan_scan, robot, intent, self.now(), time.monotonic())

    def observations(self):
        from check_stereo_input import inspect_track
        from tf2_ros import TransformException
        with self.input_lock:
            packet = self.packet
        if packet is None:
            return []
        result = []
        for track in packet["tracks"]:
            position_ok, velocity_ok, reasons = inspect_track(packet, track, self.now())
            item = {"packet": packet, "track": track, "position_ok": position_ok,
                    "velocity_ok": velocity_ok, "match": None, "reasons": reasons}
            if position_ok:
                key = (packet["timestamp"], track["id"])
                self.positions.add(key)
                if velocity_ok:
                    self.velocities.add(key)
                match = self.current_matches.get(key)
                if match is None:
                    try:
                        with self.input_lock:
                            history = tuple(self.scans)
                        if not history:
                            raise ValueError("NO_SCAN")
                        scan = min((s for received, s in history),
                                   key=lambda s: abs(stamp(s.header.stamp)-packet["timestamp"]))
                        if abs(stamp(scan.header.stamp)-packet["timestamp"]) > .12:
                            raise ValueError("NO_TIME_MATCHED_SCAN")
                        p, q, _ = self.transform("odom", scan.header.frame_id, stamp(scan.header.stamp))
                        clusters = scan_clusters(scan.ranges, scan.angle_min, scan.angle_increment,
                                                 scan.range_min, scan.range_max)
                        fitted, status = associate(clusters, track["xyz_odom_m"], p, q)
                        match = dict(fitted, matched=True, image_stamp=packet["timestamp"],
                                     scan_stamp=stamp(scan.header.stamp)) if fitted else {"matched": False, "reason": status}
                        if fitted:
                            self.matches.add(key)
                            self.current_matches[key] = match
                    except (TransformException, ValueError) as error:
                        match = {"matched": False, "reason": str(error)}
                item["match"] = match
            result.append(item)
        return result
