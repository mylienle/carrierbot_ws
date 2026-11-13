"""Read-only camera/LiDAR checks for a stationary robot; never routes commands."""
from collections import Counter, deque
import json
import math
from statistics import median
import time

from sensor_pair_geometry import associate, scan_clusters


def readiness_report(status, status_age, scan_age, wheel_age, motion_seen,
                     input_publishers, wheel_publishers):
    blockers = []
    if status is None:
        blockers.append("NO_FIXTURE_STATUS")
    else:
        if status.get("ready") is not True:
            blockers.append("ACTOR_PROXY_NOT_COUPLED")
        if status_age is None or status_age > 0.6:
            blockers.append("FIXTURE_STATUS_STALE")
    if scan_age is None or scan_age > 0.6:
        blockers.append("SCAN_STALE_OR_MISSING")
    if wheel_age is None or wheel_age > 0.6:
        blockers.append("WHEEL_COMMAND_STALE_OR_MISSING")
    if motion_seen:
        blockers.append("NONZERO_OR_INVALID_WHEEL_COMMAND_SEEN")
    if input_publishers:
        blockers.append("GUARD_INPUT_HAS_PUBLISHER")
    if len(wheel_publishers) != 1 or wheel_publishers[0].rstrip("/").split("/")[-1] != "carrierbot_safety_guard":
        blockers.append("UNEXPECTED_WHEEL_PUBLISHERS")
    return {"ready": not blockers, "blockers": blockers,
            "fixture_status_age_wall_s": status_age, "scan_age_wall_s": scan_age,
            "wheel_command_age_wall_s": wheel_age, "motion_command_seen": motion_seen,
            "guard_input_publishers": input_publishers, "wheel_publishers": wheel_publishers,
            "fixture_status": status}


class AlignmentAvailability:
    """Allow a bounded observation gap at rest; never mark the gap's data ready."""
    FATAL = {"NONZERO_OR_INVALID_WHEEL_COMMAND_SEEN", "GUARD_INPUT_HAS_PUBLISHER",
             "UNEXPECTED_WHEEL_PUBLISHERS"}

    def __init__(self, pause_limit=5.0):
        self.pause_limit = pause_limit
        self.since = None
        self.origin = None
        self.events = []

    def update(self, snapshot, now):
        if self.origin is None:
            self.origin = now
        blockers = snapshot["blockers"]
        if self.FATAL.intersection(blockers):
            self.events.append({"event": "abort_route_or_motion", "wall_s": now-self.origin,
                                "snapshot": snapshot})
            return "abort"
        if snapshot["ready"]:
            if self.since is not None:
                self.events.append({"event": "resumed", "wall_s": now-self.origin,
                                    "gap_wall_s": now-self.since, "snapshot": snapshot})
            self.since = None
            return "ready"
        if self.since is None:
            self.since = now
            self.events.append({"event": "paused", "wall_s": now-self.origin,
                                "snapshot": snapshot})
        if now-self.since >= self.pause_limit:
            self.events.append({"event": "abort_gap_timeout", "wall_s": now-self.origin,
                                "gap_wall_s": now-self.since, "snapshot": snapshot})
            return "abort"
        return "pause"


class AlignmentObserver:
    def __init__(self, node, buffer, verbose=True):
        from geometry_msgs.msg import Twist
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        self.node, self.buffer = node, buffer
        self.verbose = verbose
        self.scans = deque(maxlen=120)
        self.scan_wall = 0.0
        self.status, self.status_wall = None, 0.0
        self.wheel_wall, self.wheel_samples = 0.0, 0
        self.motion_command_seen = False
        self.start_robot_xy = None
        self.max_robot_displacement = 0.0
        self.start_robot_yaw = None
        self.max_robot_rotation = 0.0
        self.eligible = 0
        self.matches = []
        self.coupling_errors = []
        self.reasons = Counter()
        self.output = None
        self.subscriptions = [
            node.create_subscription(LaserScan, "/scan", self.receive_scan, qos_profile_sensor_data),
            node.create_subscription(String, "/test_fixture/person_lidar_status", self.receive_status, 1),
            node.create_subscription(Twist, "/carrierbot_controller/cmd_vel_unstamped", self.receive_wheel, 1),
        ]

    def receive_scan(self, message):
        self.scans.append(message)
        self.scan_wall = time.monotonic()

    def receive_status(self, message):
        try:
            value = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if isinstance(value, dict):
            self.status, self.status_wall = value, time.monotonic()

    def receive_wheel(self, message):
        self.wheel_wall = time.monotonic()
        self.wheel_samples += 1
        values = (message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z)
        if any(not math.isfinite(v) or abs(v) > 1e-6 for v in values):
            self.motion_command_seen = True

    def snapshot(self):
        now = time.monotonic()
        sources = self.node.get_publishers_info_by_topic("/carrierbot_controller/cmd_vel_unstamped")
        inputs = self.node.get_publishers_info_by_topic("/safety/input_cmd_vel")
        def names(endpoints):
            return [p.node_namespace.rstrip("/") + "/" + p.node_name for p in endpoints]
        result = readiness_report(
            self.status, now - self.status_wall if self.status_wall else None,
            now - self.scan_wall if self.scans else None,
            now - self.wheel_wall if self.wheel_wall else None,
            self.motion_command_seen, names(inputs), names(sources))
        result["wheel_samples"] = self.wheel_samples
        result["node_time_sim_s"] = self.node.get_clock().now().nanoseconds / 1e9
        return result

    def ready(self):
        return self.snapshot()["ready"]

    def observe(self, packet, track, position_ok):
        if not position_ok:
            return
        from rclpy.time import Time
        from tf2_ros import TransformException
        self.eligible += 1
        stamp = packet["timestamp"]
        row = {"timestamp": stamp, "id": track.get("id"),
               "stereo_xy_odom_m": track["xyz_odom_m"][:2], "matched": False}
        try:
            readiness = self.snapshot()
            if not readiness["ready"]:
                row["status"] = "FIXTURE_OR_STATIONARY_ROUTE_NOT_READY"
                row["readiness"] = readiness
                return row
            coupling = self.status.get("coupling_error_m")
            if not isinstance(coupling, (float, int)) or not math.isfinite(coupling):
                row["status"] = "NO_COUPLING_MEASUREMENT"
                return row
            self.coupling_errors.append(coupling)
            scan = min(self.scans, key=lambda s: abs(self.stamp(s.header.stamp) - stamp))
            scan_stamp = self.stamp(scan.header.stamp)
            row["scan_timestamp"] = scan_stamp
            row["scan_frame_id"] = scan.header.frame_id
            row["sensor_time_difference_s"] = abs(scan_stamp - stamp)
            if abs(scan_stamp - stamp) > 0.12:
                row["status"] = "NO_TIME_MATCHED_SCAN"
                return row
            transform = self.buffer.lookup_transform("odom", scan.header.frame_id,
                                                     Time.from_msg(scan.header.stamp))
            p, q = transform.transform.translation, transform.transform.rotation
            row["scan_transform_timestamp"] = self.stamp(transform.header.stamp)
            if abs(row["scan_transform_timestamp"] - scan_stamp) > 0.001:
                row["status"] = "INVALID_SCAN_TRANSFORM_TIMESTAMP"
                return row
            # Query the robot at image time as well; do not silently substitute latest TF.
            robot = self.buffer.lookup_transform("odom", "base_footprint",
                                                 Time(nanoseconds=round(stamp * 1e9)))
            r = robot.transform.translation
            rq = robot.transform.rotation
            yaw = math.atan2(2*(rq.w*rq.z + rq.x*rq.y), 1-2*(rq.y*rq.y + rq.z*rq.z))
            row["robot_pose_at_image"] = {"x": r.x, "y": r.y, "yaw": yaw, "stamp": stamp}
            if self.start_robot_xy is None:
                self.start_robot_xy = (r.x, r.y)
                self.start_robot_yaw = yaw
            self.max_robot_displacement = max(self.max_robot_displacement,
                math.hypot(r.x - self.start_robot_xy[0], r.y - self.start_robot_xy[1]))
            self.max_robot_rotation = max(self.max_robot_rotation,
                abs(math.atan2(math.sin(yaw-self.start_robot_yaw), math.cos(yaw-self.start_robot_yaw))))
            clusters = scan_clusters(scan.ranges, scan.angle_min, scan.angle_increment,
                                     scan.range_min, scan.range_max)
            match, status = associate(clusters, track["xyz_odom_m"],
                                      (p.x, p.y), (q.x, q.y, q.z, q.w))
            row["status"] = status
            if match is not None:
                row.update(match, matched=True)
                self.matches.append(row.copy())
        except TransformException as error:
            row.update(status="NO_SENSOR_TIME_TF", tf_error=str(error))
        finally:
            self.reasons[row.get("status", "UNKNOWN")] += 1
            if self.output is not None:
                self.output.write(json.dumps(row) + "\n")
                self.output.flush()
            if self.verbose and (self.eligible <= 3 or self.eligible % 10 == 0):
                print("DOI CHIEU: " + json.dumps(row), flush=True)
        return row

    @staticmethod
    def stamp(value):
        return value.sec + value.nanosec / 1e9

    def finish(self, total_tracks):
        errors = sorted(row["center_difference_m"] for row in self.matches)
        span = (max(r["center_odom_m"][1] for r in self.matches) -
                min(r["center_odom_m"][1] for r in self.matches)) if self.matches else 0.0
        fraction = len(self.matches) / self.eligible if self.eligible else 0.0
        position_fraction = self.eligible / total_tracks if total_tracks else 0.0
        checks = {
            "at_least_20_position_samples": self.eligible >= 20,
            "at_least_20_matches": len(self.matches) >= 20,
            "position_fraction_at_least_0_8": position_fraction >= 0.8,
            "match_fraction_at_least_0_8": fraction >= 0.8,
            "center_error_p95_within_0_30_m": bool(errors) and errors[min(len(errors)-1, math.ceil(.95*len(errors))-1)] <= .30,
            "observed_lateral_span_at_least_0_8_m": span >= 0.8,
            "coupling_within_0_05_m": bool(self.coupling_errors) and max(self.coupling_errors) <= 0.05,
            "stationary_robot": self.max_robot_displacement <= 0.05 and self.max_robot_rotation <= 0.05
                                and not self.motion_command_seen,
            "fixture_and_guard_route_still_ready": bool(self.ready()),
        }
        return {"passed": all(checks.values()), "checks": checks,
                "eligible_positions": self.eligible, "matched_positions": len(self.matches),
                "match_fraction": fraction, "position_fraction": position_fraction,
                "center_error_median_m": median(errors) if errors else None,
                "center_error_p95_m": errors[min(len(errors)-1, math.ceil(.95*len(errors))-1)] if errors else None,
                "center_error_max_m": max(errors) if errors else None,
                "observed_lidar_lateral_span_m": span,
                "max_actor_proxy_error_m": max(self.coupling_errors) if self.coupling_errors else None,
                "max_robot_displacement_m": self.max_robot_displacement,
                "max_robot_rotation_rad": self.max_robot_rotation,
                "reasons": dict(self.reasons),
                "scope": "Position/time agreement for the controlled cylinder proxy; velocity accuracy and avoidance are not scored."}
