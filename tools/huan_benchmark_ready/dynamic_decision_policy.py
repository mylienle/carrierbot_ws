"""Provisional per-person proposals; no actuator, path editing or ROS publisher.

Caller supplies validated Vision position/velocity flags and a time-matched
LiDAR association. Huan's existing CPA screening is reused unchanged.
"""
from collections import deque
import math

from huan_dynamic_obstacles import assess, number, vector
from fablab_route import intersects_route_corridor


def straight_candidate(robot, now, speed=0.12, horizon=2.0):
    """A hypothetical straight path, explicitly not the running Fablab route."""
    x, y, yaw = robot["x"], robot["y"], robot["yaw"]
    return {"frame_id": "odom", "stamp": now,
            "kind": "hypothetical_straight_not_executed", "speed_mps": speed,
            "positions": [{"dt_s": 0.0, "xy_m": [x, y]},
                          {"dt_s": horizon, "xy_m": [x+speed*horizon*math.cos(yaw),
                                                     y+speed*horizon*math.sin(yaw)]}]}


class DecisionPolicy:
    def __init__(self):
        self.still_history = {}

    def evaluate(self, packet, track, match, robot, plan, now,
                 position_ok=False, velocity_ok=False):
        ident = track.get("id") if isinstance(track, dict) else None
        result = {"timestamp": packet.get("timestamp"), "received_sim": now,
                  "id": ident, "proposal": "WAIT", "reason": "INPUT_NOT_READY",
                  "risk": "UNKNOWN", "motion_status": "UNKNOWN",
                  "actuation": False, "robot_radius_m": .52,
                  "scope": "Single matched person; proposal under the recorded candidate timing model.",
                  "robot_plan": plan, "velocity_valid": bool(velocity_ok),
                  "velocity_source": "huan_vision_odom" if velocity_ok else None}

        def choose(action, reason):
            result.update(proposal=action, reason=reason)
            return result

        if not position_ok or not isinstance(ident, int) or isinstance(ident, bool) or ident < 0:
            if isinstance(ident, int):
                self.still_history.pop(ident, None)
            return choose("WAIT", "VISION_POSITION_UNAVAILABLE")
        if (not isinstance(match, dict) or match.get("matched") is not True or
                not vector(match.get("center_odom_m"), 2) or not number(match.get("radius_m"))):
            self.still_history.pop(ident, None)
            return choose("WAIT", "LIDAR_ASSOCIATION_UNAVAILABLE")
        if (not isinstance(robot, dict) or not all(number(robot.get(k)) for k in ("x", "y", "yaw", "stamp"))
                or not number(now) or not -.001 <= now-robot["stamp"] <= .3):
            return choose("WAIT", "CURRENT_ROBOT_TF_UNAVAILABLE")
        if not isinstance(plan, dict):
            return choose("WAIT", "ROBOT_PLAN_UNAVAILABLE")
        center = match["center_odom_m"]
        radius = match["radius_m"]
        if not .15 <= radius <= .35:
            return choose("WAIT", "UNSUPPORTED_FIXTURE_GEOMETRY")
        dx, dy = center[0]-robot["x"], center[1]-robot["y"]
        forward = dx*math.cos(robot["yaw"])+dy*math.sin(robot["yaw"])
        lateral = -dx*math.sin(robot["yaw"])+dy*math.cos(robot["yaw"])
        in_corridor = 0 < forward <= 3.0 and abs(lateral) <= .52+radius+.1
        if plan.get("kind") == "fablab_route_nominal_timing":
            in_corridor = intersects_route_corridor(center, plan.get("route_corridor_xy_m"), .52+radius+.1)
            if in_corridor is None:
                return choose("WAIT", "ROBOT_ROUTE_CORRIDOR_UNAVAILABLE")
            if plan.get("guidance_hold") is True:
                return choose("WAIT", "GUIDANCE_STOP_OR_ALIGNMENT")
        clearance = math.hypot(dx, dy)-.52-radius
        result.update(lidar_center_odom_m=center, obstacle_radius_m=radius,
                      forward_m=forward, lateral_m=lateral, current_clearance_m=clearance,
                      within_candidate_corridor=in_corridor)
        if clearance <= .25:
            self.still_history.pop(ident, None)
            return choose("WAIT", "CURRENT_CLEARANCE_SMALL")
        if not velocity_ok:
            self.still_history.pop(ident, None)
            return choose("SLOW" if in_corridor else "FOLLOW_LIDAR_ONLY", "MOTION_UNKNOWN")

        # Keep original timestamps and provenance. Do not substitute LiDAR velocity.
        one_person = dict(packet, tracks=[track])
        risk = assess(one_person, plan, now, horizon=2.0, robot_radius=.52,
                      person_radius=max(.30, radius), margin=.20, uncertainty_rate=.15)
        result["assessment"] = risk
        result["risk"] = risk["risk"]
        if not risk["valid"]:
            self.still_history.pop(ident, None)
            return choose("SLOW", "PREDICTION_UNAVAILABLE")
        velocity = track["velocity_odom_mps"]
        speed = math.hypot(velocity[0], velocity[1])
        stamp = packet["timestamp"]
        history = self.still_history.setdefault(ident, deque(maxlen=20))
        if history and (stamp <= history[-1][0] or stamp-history[-1][0] > .6):
            history.clear()
        if speed > .05:
            history.clear()
        history.append((stamp, speed))
        while history and stamp-history[0][0] > 2.0:
            history.popleft()
        still = (len(history) >= 3 and stamp-history[0][0] >= 1.0
                 and all(s <= .05 for t, s in history))
        result["motion_status"] = "STATIONARY_CONFIRMED" if still else (
            "MOVING" if speed > .05 else "LOW_SPEED_UNCONFIRMED")
        if still and in_corridor:
            return choose("CHECK_DETOUR", "PERSISTENT_STATIONARY_BLOCKER")
        if risk["risk"] == "POTENTIAL_COLLISION":
            return choose("WAIT", "PREDICTED_SPATIOTEMPORAL_CONFLICT")
        if result["motion_status"] == "LOW_SPEED_UNCONFIRMED" and in_corridor:
            return choose("SLOW", "STATIONARY_NOT_CONFIRMED")
        return choose("FOLLOW_CANDIDATE", "NO_CONFLICT_WITHIN_MODEL_HORIZON")
