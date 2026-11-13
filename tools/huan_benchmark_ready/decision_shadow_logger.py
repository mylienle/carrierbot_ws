"""Log decision proposals against live data without publishing any ROS message."""
from collections import Counter
import json
import hashlib
import math
from pathlib import Path

from dynamic_decision_policy import DecisionPolicy, straight_candidate


class DecisionShadowLogger:
    def __init__(self, buffer, output, route_reader=None):
        here = Path(__file__).resolve().parent
        source = json.loads((here / "huan_risk_source.json").read_text(encoding="utf-8"))
        if hashlib.sha256((here / "huan_dynamic_obstacles.py").read_bytes()).hexdigest() != source["sha256"]:
            raise RuntimeError("The reused Huan risk module differs from its source manifest")
        self.buffer = buffer
        self.route_reader = route_reader
        self.output = output
        self.policy = DecisionPolicy()
        self.actions = Counter()
        self.reasons = Counter()
        self.records = 0
        self.matched_records = 0
        self.velocity_records = 0
        self.last_proposal = None
        self.valid_route_records = 0
        self.route_geometries = set()
        self.last_route_snapshot = None

    def observe(self, packet, track, match, now, position_ok, velocity_ok):
        from rclpy.time import Time
        from tf2_ros import TransformException
        robot, plan = None, None
        tf_error = None
        route_error = None
        try:
            transform = self.buffer.lookup_transform("odom", "base_footprint", Time())
            latest_stamp = transform.header.stamp.sec+transform.header.stamp.nanosec/1e9
            if latest_stamp > now:
                transform = self.buffer.lookup_transform("odom", "base_footprint",
                                                         Time(nanoseconds=round(now*1e9)))
            p, q = transform.transform.translation, transform.transform.rotation
            robot = {"x": p.x, "y": p.y,
                     "yaw": math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
                     "stamp": transform.header.stamp.sec+transform.header.stamp.nanosec/1e9}
            if self.route_reader is None:
                plan = straight_candidate(robot, now)
            else:
                plan, route_error = self.route_reader.candidate(robot, now)
        except TransformException as error:
            tf_error = str(error)
        result = self.policy.evaluate(packet, track, match, robot, plan, now,
                                      position_ok=position_ok, velocity_ok=velocity_ok)
        result["robot_pose_odom"] = robot
        result["alignment_status"] = match.get("status") if isinstance(match, dict) else "NOT_AVAILABLE"
        if tf_error:
            result["tf_error"] = tf_error
        if route_error:
            result["route_error"] = route_error
        self.valid_route_records += int(plan is not None)
        if self.route_reader is not None and plan is not None:
            self.last_route_snapshot = plan.get("route_snapshot")
            self.route_geometries.add(json.dumps([
                plan.get("route_segment"), self.last_route_snapshot.get("waypoints_map")]))
        self.output.write(json.dumps(result) + "\n")
        self.output.flush()
        self.records += 1
        self.matched_records += int(isinstance(match, dict) and match.get("matched") is True)
        self.velocity_records += int(velocity_ok)
        self.actions[result["proposal"]] += 1
        self.reasons[result["reason"]] += 1
        if self.last_proposal != result["proposal"] or self.records <= 3 or self.records % 10 == 0:
            print("DE XUAT: %s | %s | risk=%s | motion=%s | chi ghi log" %
                  (result["proposal"], result["reason"], result["risk"], result["motion_status"]), flush=True)
        self.last_proposal = result["proposal"]

    def finish(self):
        real_route = self.route_reader is not None
        complete = self.records >= 20 and self.matched_records >= 20
        if real_route:
            complete = complete and self.valid_route_records >= max(20, .8*self.records)
        return {"recording_complete": complete,
                "records": self.records, "matched_records": self.matched_records,
                "valid_velocity_records": self.velocity_records,
                "actions": dict(self.actions), "reasons": dict(self.reasons),
                "valid_route_records": self.valid_route_records,
                "route_geometry_revisions_observed": len(self.route_geometries),
                "last_route_snapshot": self.last_route_snapshot,
                "route_geometry_source": "fablab_internal_waypoints_v1" if real_route else "hypothetical_straight",
                "candidate": ("Actual remaining Fablab waypoint geometry with nominal speed timing; no motion is executed."
                              if real_route else "Hypothetical straight motion at 0.12 m/s for 2 s; not an executed or Fablab-planned route."),
                "risk_source": "Huan dynamic_obstacles.assess, reused unchanged with robot_radius=0.52 m",
                "actuation": False, "velocity_accuracy_evaluated": False,
                "route_timing_accuracy_evaluated": False,
                "closed_loop_avoidance_evaluated": False,
                "scope": "Tests data flow and proposal recording on one paired-person fixture, not avoidance performance."}
