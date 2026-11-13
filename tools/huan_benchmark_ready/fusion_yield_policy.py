"""One-person Gazebo yield experiment. Sensor inputs only; no ground truth.

BASELINE always means allow the existing LiDAR/Fablab controller, not certify a
collision-free trajectory. The nominal CPA estimate can only restrict motion.
"""
import math

from fablab_route import finite, intersects_route_corridor, nominal_positions, point_in_corridor, route_segments, xy
from huan_dynamic_obstacles import assess


NEARBY_SLOW_CLEARANCE_M = 2.0


def within_slow_zone(center, corridor, radius, max_clearance=None):
    body_separation = .52+radius
    near = intersects_route_corridor(center, corridor, body_separation+.75)
    if near is not True or max_clearance is None:
        return near
    # The corridor starts at the current robot center in odom. This bound
    # limits caution only; predicted conflicts are evaluated independently.
    return math.dist(center, corridor[0])-body_separation <= max_clearance


def release_intent(plan):
    """Counterfactual intent if released now; never change the source flags."""
    if not isinstance(plan, dict) or plan.get("kind") != "fablab_route_nominal_timing":
        raise ValueError("NO_FABLAB_ROUTE")
    snapshot = plan["route_snapshot"]
    revision = snapshot.get("route_revision")
    if not isinstance(revision, int) or isinstance(revision, bool) or revision < 0:
        raise ValueError("NO_STABLE_ROUTE_REVISION")
    speed = min(snapshot["linear_speed_mps"], snapshot["linear_speed_max_mps"])
    legacy_slow = snapshot.get("standing_detour_speed_limited", snapshot.get("standing_detour_active")) is True
    standing_cap = snapshot.get("standing_detour_speed_cap_mps", .06 if legacy_slow else 0.)
    if not finite(standing_cap) or standing_cap not in (0., .06, .08):
        raise ValueError("INVALID_STANDING_DETOUR_SPEED_CAP")
    if standing_cap > 0.:
        speed = min(speed, standing_cap)
    result = dict(plan)
    result.update(kind="fablab_release_intent_nominal_timing", route_revision=revision,
                  positions=nominal_positions(plan["route_geometry_odom_m"], speed),
                  speed_assumption_mps=speed, counterfactual_release_intent=True,
                  standing_detour_speed_cap_mps=standing_cap,
                  timing_model="nominal_motion_if_released_now_not_a_command",
                  scope="Reference intent only; intrinsic Fablab stops and safety_guard remain enforced.")
    return result


def raw_route_corridor(points_odom, robot_xy, corridor, valid_returns, yield_clearance_m=None):
    """A forward point corridor, not a general obstacle shape estimator."""
    if (not xy(robot_xy) or not isinstance(corridor, list) or len(corridor) < 2 or
            not all(xy(p) for p in corridor) or valid_returns < 30):
        return {"valid": False, "reason": "INSUFFICIENT_SCAN_GEOMETRY"}
    if yield_clearance_m is not None and (not finite(yield_clearance_m) or not .6 <= yield_clearance_m <= 3.):
        return {"valid": False, "reason": "INVALID_YIELD_CLEARANCE"}
    segments = route_segments(corridor)
    _, _, dx, dy, _ = segments[0]
    if math.hypot(dx, dy) < 1e-8:
        return {"valid": False, "reason": "DEGENERATE_FORWARD_CORRIDOR"}
    hits = 0
    corridor_hits = 0
    minimum_clearance = None
    for point in points_odom:
        if not xy(point):
            return {"valid": False, "reason": "INVALID_SCAN_POINT"}
        if (point[0]-robot_xy[0])*dx+(point[1]-robot_xy[1])*dy <= 0:
            continue
        if point_in_corridor(point, corridor[0], segments, .52+.30):
            corridor_hits += 1
            clearance = math.dist(point, robot_xy)-.52
            minimum_clearance = clearance if minimum_clearance is None else min(minimum_clearance, clearance)
            # 0.6m is measured from the modelled body to an actual scan return,
            # not from the robot centre, and not as the full corridor width.
            if yield_clearance_m is None or clearance <= yield_clearance_m+1e-9:
                hits += 1
    return {"valid": True, "blocked": hits >= 3, "blocking_points": hits,
            "valid_returns": valid_returns, "robot_radius_m": .52,
            "point_corridor_margin_m": .30,
            "yield_clearance_m": yield_clearance_m,
            "yield_zone_profile": "legacy_3m_corridor" if yield_clearance_m is None else "forward_body_clearance",
            "nearest_forward_corridor_clearance_m": minimum_clearance,
            "forward_corridor_returns": corridor_hits,
            "centre_to_point_stop_radius_m": None if yield_clearance_m is None else .52+yield_clearance_m,
            "scope": "Forward reference corridor; configured clearance from 0.52m circular body to measured LiDAR surface."
                     if yield_clearance_m is not None else "Forward 3 m reference corridor in the controlled crossing room."}


class FusionYieldPolicy:
    def __init__(self, resume_fresh_scans=False, *, nearby_slow=False):
        self.resume_fresh_scans = resume_fresh_scans
        self.slow_clearance = NEARBY_SLOW_CLEARANCE_M if nearby_slow else None
        self.revision = None
        self.last_time = None
        self.reset_clearance()

    def reset_clearance(self):
        self.clear_since = self.released_at = self.gap_started = self.first_recovery_scan = None

    def _input_gap(self, now, transient):
        # Only an already released route can qualify. A second gap during
        # confirmation or any other stop returns to the full confirmation.
        if not (self.resume_fresh_scans and transient):
            self.reset_clearance()
            return
        if self.released_at is not None and 0 <= now-self.released_at <= .15:
            self.gap_started = now
        elif self.clear_since is not None or self.gap_started is None or now-self.gap_started > .18:
            self.gap_started = self.first_recovery_scan = None
        self.clear_since = self.released_at = None

    def _fresh_scan_confirmation(self, now, intent, raw):
        if self.gap_started is None:
            return False
        if (self.clear_since-self.gap_started > .18 or now-self.gap_started > .45
                or intent.get("guidance_hold") is not False or raw.get("exact_sensor_time_tf") is not True):
            self.gap_started = self.first_recovery_scan = None
            return False
        scan = raw["stamp"]
        # Neither a scan buffered before the dropout nor the same scan on two
        # consecutive policy ticks provides independent evidence of clearance.
        if not self.gap_started < scan <= now:
            return False
        if self.first_recovery_scan is None:
            self.first_recovery_scan = scan
            return False
        return scan > self.first_recovery_scan

    def evaluate(self, now, intent, raw, observations, *, transient_gap=False):
        result = {"stamp": now, "action": "WAIT", "action_code": 0, "speed_cap_mps": 0.,
                  "reason": "INPUT_UNAVAILABLE", "route_revision": None,
                  "vision_observations": [], "raw_corridor": raw,
                  "risk_model": "nominal_CPA_restriction_only", "sensor_based": True}
        def choose(action, reason):
            result.update(action=action, action_code={"WAIT": 0, "SLOW": 1, "BASELINE": 2}[action],
                          speed_cap_mps={"WAIT": 0., "SLOW": .06, "BASELINE": .12}[action], reason=reason)
            return result
        if not finite(now) or (self.last_time is not None and now < self.last_time-.001):
            self.reset_clearance()
            return choose("WAIT", "INVALID_OR_REVERSED_CLOCK")
        self.last_time = now
        if (not isinstance(intent, dict) or intent.get("kind") != "fablab_release_intent_nominal_timing" or
                not finite(intent.get("stamp")) or not 0 <= now-intent["stamp"] <= .3 or
                not isinstance(intent.get("route_revision"), int)):
            self._input_gap(now, transient_gap)
            return choose("WAIT", "ROUTE_INTENT_UNAVAILABLE")
        revision = intent["route_revision"]
        result["route_revision"] = revision
        if revision != self.revision:
            self.revision = revision
            self.reset_clearance()
        if (not isinstance(raw, dict) or raw.get("valid") is not True or
                not isinstance(raw.get("blocked"), bool) or not finite(raw.get("stamp")) or
                not -.02 <= now-raw["stamp"] <= .3):
            self._input_gap(now, transient_gap)
            return choose("WAIT", "LIDAR_UNAVAILABLE")
        caution, prediction_conflict = False, False
        for observation in observations or []:
            packet, track, match = (observation.get(key) for key in ("packet", "track", "match"))
            stamp = packet.get("timestamp") if isinstance(packet, dict) else None
            if (observation.get("position_ok") is not True or not finite(stamp) or
                    not 0 <= now-stamp <= .5 or not isinstance(track, dict) or
                    not isinstance(match, dict) or match.get("matched") is not True or
                    not xy(match.get("center_odom_m")) or not finite(match.get("radius_m")) or
                    not .15 <= match["radius_m"] <= .35):
                continue
            near = within_slow_zone(match["center_odom_m"], intent["route_corridor_xy_m"],
                                    match["radius_m"], self.slow_clearance)
            if near is None:
                self.reset_clearance()
                return choose("WAIT", "ROUTE_CORRIDOR_UNAVAILABLE")
            item = {"id": track.get("id"), "image_stamp": stamp, "matched": True,
                    "within_caution_corridor": near, "velocity_status": "UNKNOWN", "risk": "UNKNOWN"}
            caution = caution or near
            if observation.get("velocity_ok") is True:
                risk = assess(dict(packet, tracks=[track]), intent, now, horizon=2.,
                              robot_radius=.52, person_radius=max(.30, match["radius_m"]),
                              margin=.20, uncertainty_rate=.15)
                item["assessment"] = risk
                if risk["valid"]:
                    item.update(velocity_status="VALID_HUAN_VISION", risk=risk["risk"])
                    prediction_conflict = prediction_conflict or risk["risk"] == "POTENTIAL_COLLISION"
            result["vision_observations"].append(item)
        if raw["blocked"] or prediction_conflict:
            self.reset_clearance()
            return choose("WAIT", "LIDAR_CORRIDOR_BLOCKED" if raw["blocked"] else "VISION_PREDICTED_CONFLICT")
        if self.clear_since is None:
            self.clear_since = now
        result["clear_duration_sim_s"] = now-self.clear_since
        recovered = self._fresh_scan_confirmation(now, intent, raw)
        if now-self.clear_since < .5 and self.released_at is None and not recovered:
            return choose("WAIT", "CONFIRMING_CLEAR_CORRIDOR")
        if recovered:
            result["resume_confirmation"] = "TWO_NEW_CLEAR_EXACT_TF_SCANS"
        self.gap_started = self.first_recovery_scan = None
        if self.resume_fresh_scans and intent.get("guidance_hold") is False and raw.get("exact_sensor_time_tf") is True:
            self.released_at = now
        else:
            self.released_at = None
        if caution:
            return choose("SLOW", "VISION_PERSON_NEAR_REFERENCE_ROUTE")
        return choose("BASELINE", "LIDAR_CLEAR_WITH_MATCHED_VISION" if result["vision_observations"] else
                      "LIDAR_CLEAR_VISION_UNAVAILABLE_OR_OUT_OF_VIEW")
