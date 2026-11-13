"""Huan sensor fusion restricts the established Fablab/guard chain only."""
import math

from fablab_route import finite, intersects_route_corridor, xy
from fusion_yield_policy import FusionYieldPolicy, within_slow_zone
from huan_fusion_bridge import fusion_risk


class IntegratedFusionPolicy:
    MODES = ("huan", "legacy", "lidar-only")

    def __init__(self, mode="huan", *, resume_fresh_scans=False, nearby_slow=False):
        if mode not in self.MODES:
            raise ValueError("Unsupported sensor comparison mode")
        self.mode = mode
        self.base = FusionYieldPolicy(resume_fresh_scans=resume_fresh_scans, nearby_slow=nearby_slow)

    def evaluate(self, now, intent, raw, observations, fusion=None, *, transient_gap=False):
        decision = self.base.evaluate(now, intent, raw, [] if self.mode == "lidar-only" else observations,
                                      transient_gap=transient_gap)
        decision.update(integration_mode=self.mode, fusion_status="NOT_SELECTED", fusion_observations=[])
        if self.mode != "huan":
            return decision
        packet = fusion.get("packet") if isinstance(fusion, dict) else None
        risk = fusion_risk(packet, intent, now)
        decision.update(fusion_assessment=risk,
                        fusion_status="FUSION_VALID" if risk["valid"] else "LIDAR_FALLBACK_FUSION_UNAVAILABLE")
        if not risk["valid"]:
            return decision
        caution = False
        for obstacle in risk["obstacles"]:
            position = obstacle["position_m"]
            velocity = obstacle["velocity_mps"]
            current = [position[i]+obstacle["age_s"]*velocity[i] for i in range(2)]
            near = within_slow_zone(current, intent["route_corridor_xy_m"], obstacle["radius_m"],
                                    self.base.slow_clearance)
            caution = caution or near is True
            decision["fusion_observations"].append(dict(
                id=obstacle["id"], source=obstacle.get("source", "vision_rgbd_tf"),
                stamp=obstacle["stamp"], position_m=current, velocity_mps=velocity,
                within_caution_corridor=near, risk=obstacle["risk"]))
        # A fused model cannot override missing raw data, raw obstacles or
        # intrinsic Fablab/guard stops. Missing velocity is never zero velocity.
        if risk["risk"] == "POTENTIAL_COLLISION":
            self.base.reset_clearance()
            if decision["action"] != "WAIT" or decision["reason"] == "CONFIRMING_CLEAR_CORRIDOR":
                decision.pop("clear_duration_sim_s", None)
                decision.pop("resume_confirmation", None)
                decision.update(action="WAIT", action_code=0, speed_cap_mps=0.,
                                reason="HUAN_FUSION_PREDICTED_CONFLICT")
        elif caution and decision["action"] == "BASELINE":
            decision.update(action="SLOW", action_code=1, speed_cap_mps=.06,
                            reason="HUAN_FUSION_OBSTACLE_NEAR_ROUTE")
        return decision


def stabilize_standing_detour_release(decision, intent):
    """Confirm one permitted speed while releasing a stopped, speed-limited detour."""
    if decision["action"] != "BASELINE" or intent is None:
        return decision
    route = intent["route_snapshot"]
    capped_detour = (route.get("standing_detour_speed_limited") is True
                    or route.get("standing_detour_speed_cap_mps") == .08)
    if capped_detour and route.get("behavior_gate", {}).get("action") == 0:
        # Resume slowly before considering the higher cap for a clear rejoin.
        return dict(decision, action="SLOW", action_code=1, speed_cap_mps=min(.06, decision["speed_cap_mps"]),
                    standing_detour_release_cap=True)
    return decision


def person_approach_limit(corridor, center, radius, velocity):
    """Reserve room on the current leg while a matched person can enter it."""
    start, end = corridor[:2]
    dx, dy = end[0]-start[0], end[1]-start[1]
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return None
    ux, uy = dx/length, dy/length
    x, y = center[0]-start[0], center[1]-start[1]
    along, lateral = x*ux+y*uy, y*ux-x*uy
    if not 0 < along <= min(length, 2.85) or abs(lateral) > 2.45:
        return None
    lateral_speed = 0. if velocity is None else velocity[1]*ux-velocity[0]*uy
    toward_route = (abs(lateral_speed) >= .12 and lateral*lateral_speed < 0 and
                    abs(lateral) <= .52+radius+4.*abs(lateral_speed))
    if along <= 2.30 and (abs(lateral) <= .52+radius or toward_route):
        return "WAIT"
    return "SLOW"


class StationaryBlocker:
    """Reserve room, then match fresh positions before allowing stopped planning."""

    def __init__(self):
        self.last = None
        self.reservation = None

    def keep_reservation(self, now, decision):
        if self.reservation is not None:
            revision, until = self.reservation
            if decision["route_revision"] == revision and now <= until:
                decision["standing_approach_reserved"] = True
                if decision["action"] != "WAIT" or decision["reason"] == "CONFIRMING_CLEAR_CORRIDOR":
                    decision.update(action="WAIT", action_code=0, speed_cap_mps=0.,
                                    reason="RESERVING_SPACE_FOR_PERSON")
            else:
                self.reservation = None

    def observe(self, now, intent, decision, observations):
        # Processing latency and the gap between image timestamps are distinct.
        if self.last is not None and not 0 <= now-self.last["stamp"] <= .85:
            self.last = None
        raw = decision["raw_corridor"]
        if not (intent and raw.get("valid") is True and raw.get("exact_sensor_time_tf") is True
                and decision["reason"] in ("CONFIRMING_CLEAR_CORRIDOR", "LIDAR_CORRIDOR_BLOCKED",
                    "VISION_PREDICTED_CONFLICT", "HUAN_FUSION_PREDICTED_CONFLICT",
                    "VISION_PERSON_NEAR_REFERENCE_ROUTE", "HUAN_FUSION_OBSTACLE_NEAR_ROUTE",
                    "LIDAR_CLEAR_WITH_MATCHED_VISION", "LIDAR_CLEAR_VISION_UNAVAILABLE_OR_OUT_OF_VIEW")):
            return None
        route = intent["route_snapshot"]
        if route.get("avoidance_state") != 0 or route.get("standing_detour_active") is True:
            self.last = self.reservation = None
            return None
        if len(observations) != 1:
            self.last = None
            self.keep_reservation(now, decision)
            return None
        sample = observations[0]
        match = sample.get("match") or {}
        track = sample["track"]
        stamp = sample["packet"].get("timestamp")
        center, radius = match.get("center_odom_m"), match.get("radius_m")
        if not (sample["position_ok"] and match.get("matched") is True and
                finite(stamp) and 0 <= now-stamp <= .5 and match.get("image_stamp") == stamp and
                xy(center) and finite(radius) and .15 <= radius <= .35 and track.get("id") is not None):
            self.keep_reservation(now, decision)
            return None
        corridor = intent["route_corridor_xy_m"]
        velocity = track.get("velocity_odom_mps")
        velocity = (velocity[:2] if sample["velocity_ok"] and isinstance(velocity, (tuple, list))
                    and xy(velocity[:2]) else None)
        limit = person_approach_limit(corridor, center, radius, velocity)
        if limit == "WAIT":
            self.reservation = intent["route_revision"], stamp+.75
            self.keep_reservation(now, decision)
        else:
            self.reservation = None
            if limit == "SLOW" and decision["action"] == "BASELINE":
                decision.update(action="SLOW", action_code=1, speed_cap_mps=.06,
                                reason="SLOW_NEAR_PERSON_APPROACH")
        if not intersects_route_corridor(center, corridor, .52+radius):
            self.last = None
            return None
        if velocity is not None and math.hypot(*velocity) > .12:
            self.last = None
            return None
        key = (intent["route_revision"], track["id"])
        previous = self.last
        if (previous is None or previous["key"] != key or not 0 <= stamp-previous["stamp"] <= .5 or
                math.dist(center, previous["anchor"]) > .12):
            self.last = {"key": key, "stamp": stamp, "since": stamp, "anchor": center}
        elif stamp > previous["stamp"]:
            previous["stamp"] = stamp
        duration = stamp-self.last["since"]
        center_distance = math.dist(center, corridor[0])
        if duration < 3. or center_distance-radius >= 2.5:
            return None
        transform = intent["map_to_odom"]
        dx, dy = center[0]-transform["x"], center[1]-transform["y"]
        c, s = math.cos(transform["yaw"]), math.sin(transform["yaw"])
        return {"center_map": [c*dx+s*dy, -s*dx+c*dy], "image_stamp": stamp,
                "stationary_duration_sim_s": duration, "center_distance_m": center_distance,
                "track_id": track["id"]}


def identify_input_wait(decision, stage, error):
    """Improve diagnostics without releasing any input-induced stop."""
    if error is not None:
        if decision["action"] != "WAIT":
            raise RuntimeError("Invalid input must not produce motion")
        decision["reason"] = {
            "map_pose": "ROBOT_POSE_UNAVAILABLE",
            "route_intent": "ROUTE_INTENT_UNAVAILABLE",
            "scan_tf": "SCAN_OR_EXACT_TF_UNAVAILABLE",
            "fixture": "SCORING_FIXTURE_UNAVAILABLE",
            "vision_stream": "VISION_STREAM_UNAVAILABLE",
            "fusion": "FUSION_INPUT_ERROR",
            "clock": "SIM_CLOCK_UNAVAILABLE",
        }.get(stage, "INPUT_UNAVAILABLE")
    return decision
