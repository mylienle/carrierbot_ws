"""Huan sensor fusion restricts the established Fablab/guard chain only."""
import math

from fablab_route import finite, intersects_route_corridor, nominal_positions, point_in_corridor, route_segments, xy
from fusion_yield_policy import RELEASE_PLAN_LOOKAHEAD_S, FusionYieldPolicy, within_slow_zone
from huan_fusion_bridge import apply_standing_laser_reserve, fusion_risk, stationary_laser_ids


CRAWL_COMMAND_CAP_MPS = .06
CRAWL_PREDICTION_SPEED_MPS = .07
STANDING_MOTION_LIMIT = .12
STANDING_ANCHOR_SHIFT_M = .12
STANDING_CONFIRMATION_S = .40
STANDING_MAX_SCAN_GAP_S = .15
CLOSE_PASS_RESUME = "TWO_NEW_MATCHED_CLEAR_SCANS"


def standing_required_separation(radius, motion_margin=0.):
    return .36 + max(.30, radius) + .30 + .04 + motion_margin


def standing_route_has_clearance(intent, position, radius, motion_margin=0.):
    points = intent.get("route_geometry_odom_m")
    if not isinstance(points, list) or len(points) < 2 or not all(xy(p) for p in points):
        return False
    # The simulated collision outline fits inside 0.36 m. Include a 0.30 m
    # edge gap and 0.04 m extra, with the same minimum person radius as Fablab.
    required = standing_required_separation(radius, motion_margin)
    segments = route_segments(points)
    # Fablab can switch to the next leg up to 0.50 m before a waypoint.
    for before, corner, after in zip(points, points[1:], points[2:]):
        incoming, outgoing = math.dist(corner, before), math.dist(corner, after)
        if min(incoming, outgoing) <= 1e-6:
            continue
        chord = [[corner[i] + (endpoint[i]-corner[i])*min(.50, length)/length for i in range(2)]
                 for endpoint, length in ((before, incoming), (after, outgoing))]
        segments.extend(route_segments(chord))
    return not point_in_corridor(position, points[0], segments, required)


class IntegratedFusionPolicy:
    MODES = ("huan", "legacy", "lidar-only")

    def __init__(self, mode="huan", *, resume_fresh_scans=False, nearby_slow=False,
                 reduced_standing_laser_reserve=False, crawl_on_predicted_conflict=False):
        if mode not in self.MODES:
            raise ValueError("Unsupported sensor comparison mode")
        self.mode = mode
        self.base = FusionYieldPolicy(resume_fresh_scans=resume_fresh_scans, nearby_slow=nearby_slow)
        self.reduced_standing_laser_reserve = reduced_standing_laser_reserve
        self.crawl_on_predicted_conflict = crawl_on_predicted_conflict

    def evaluate(self, now, intent, raw, observations, fusion=None, *, transient_gap=False,
                 measured_speed_mps=None, close_monitor=None, match_blocking_returns=None):
        packet = fusion.get("packet") if isinstance(fusion, dict) else None
        risk = fusion_risk(packet, intent, now) if self.mode == "huan" else None
        route = intent.get("route_snapshot", {}) if isinstance(intent, dict) else {}
        detour = route.get("avoidance_state") == 1 or route.get("standing_detour_active") is True
        if risk is not None and self.reduced_standing_laser_reserve and detour:
            apply_standing_laser_reserve(risk, packet)
        standing_check = None
        standing_status = None
        if risk is not None and close_monitor is not None:
            standing_check, standing_status = close_monitor.checked_stationary_route(
                now, intent, raw, risk, fusion, measured_speed_mps)
        if standing_check is not None:
            apply_checked_stationary_forecast(risk, standing_check)
            if raw["blocked"]:
                matched = (match_blocking_returns(raw["stamp"], standing_check["center_odom_m"],
                                                  standing_check["radius_m"], standing_check["cluster_points"])
                           if match_blocking_returns is not None else None)
                if matched and len(matched) == raw["blocking_points"]:
                    raw = dict(raw, blocked=False, blocking_points=0, baseline_blocking_points=len(matched))
                    standing_check["matched_blocking_returns"] = len(matched)
                else:
                    standing_status = "UNCONFIRMED_BLOCKING_RETURNS"
        decision = self.base.evaluate(now, intent, raw, [] if self.mode == "lidar-only" else observations,
                                      transient_gap=transient_gap, stationary_check=standing_check)
        decision.update(integration_mode=self.mode, fusion_status="NOT_SELECTED", fusion_observations=[])
        if risk is None:
            return decision
        if standing_status is not None:
            decision["standing_route_check_status"] = standing_status
        if standing_check is not None:
            decision["standing_route_check"] = standing_check
        if self.crawl_on_predicted_conflict and detour:
            risk = self.crawl_assessment(now, intent, raw, decision, risk, packet, measured_speed_mps)
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
        elif decision.get("crawl_forecast", {}).get("selected") and decision["action"] != "WAIT":
            decision.update(action="SLOW", action_code=1, speed_cap_mps=CRAWL_COMMAND_CAP_MPS,
                            reason="HUAN_FUSION_CLEAR_AT_CRAWL")
        elif standing_check is not None and decision["action"] != "WAIT":
            decision.update(action="SLOW", action_code=1, speed_cap_mps=.06,
                            reason="CHECKED_STANDING_ROUTE")
        elif caution and decision["action"] == "BASELINE":
            decision.update(action="SLOW", action_code=1, speed_cap_mps=.06,
                            reason="HUAN_FUSION_OBSTACLE_NEAR_ROUTE")
        return decision

    def check_close_pass(self, monitor, now, decision, fusion, intent):
        if monitor.restrict(now, decision, fusion, intent):
            self.base.reset_clearance()
        checked_release = monitor.confirm_checked_release(decision)
        gap_recovery = monitor.recovery.observe(now, decision, intent, monitor.blocker)
        if checked_release or gap_recovery:
            self.base.confirm_clearance()

    def crawl_assessment(self, now, intent, raw, decision, risk, packet, speed):
        """Try a positive, enforceable speed only after the robot has slowed enough."""
        if not (risk["valid"] and risk["risk"] == "POTENTIAL_COLLISION"
                and (decision["action"] != "WAIT" or decision["reason"] == "CONFIRMING_CLEAR_CORRIDOR")
                and raw.get("exact_sensor_time_tf") is True and raw.get("blocked") is False
                and intent.get("guidance_hold") is False
                and intent["route_snapshot"].get("behavior_gate", {}).get("action") in (0, 1)
                and intent["speed_assumption_mps"] > CRAWL_PREDICTION_SPEED_MPS
                and finite(speed) and 0 <= speed <= CRAWL_PREDICTION_SPEED_MPS):
            return risk
        stationary = stationary_laser_ids(packet)
        if not all(item["id"] in stationary and item.get("lidar_confirmed") is True
                   for item in risk["obstacles"]):
            return risk

        slow_intent = dict(intent, positions=nominal_positions(
            intent["route_geometry_odom_m"], CRAWL_PREDICTION_SPEED_MPS, horizon=RELEASE_PLAN_LOOKAHEAD_S))
        slow_risk = fusion_risk(packet, slow_intent, now)
        if self.reduced_standing_laser_reserve:
            apply_standing_laser_reserve(slow_risk, packet)
        clear = slow_risk["valid"] and slow_risk["risk"] == "CLEAR_WITHIN_MODEL"
        decision["crawl_forecast"] = {
            "selected": clear, "risk": slow_risk["risk"], "measured_speed_mps": speed,
            "prediction_speed_mps": CRAWL_PREDICTION_SPEED_MPS, "command_cap_mps": CRAWL_COMMAND_CAP_MPS,
            "nominal_risk": risk["risk"], "nominal_baseline_risk": risk.get("baseline_risk"),
        }
        return slow_risk if clear else risk


def apply_checked_stationary_forecast(risk, proof):
    for obstacle in risk["obstacles"]:
        if obstacle["id"] != proof["track_id"] or obstacle.get("source") != proof["source"]:
            continue
        if obstacle.get("source") == "vision_rgbd_tf" and obstacle["closest_approach_m"] <= proof["required_separation_m"]:
            continue
        gap = obstacle["closest_approach_m"] - proof["required_separation_m"]
        proof["original_fusion_risk"] = obstacle["risk"]
        obstacle.update(standing_original_required_separation_m=obstacle["required_separation_m"],
                        required_separation_m=proof["required_separation_m"], clearance_bound_m=gap,
                        risk="CLEAR_WITHIN_MODEL" if gap > 0. else "POTENTIAL_COLLISION")
    risk["risk"] = ("POTENTIAL_COLLISION" if any(o["risk"] == "POTENTIAL_COLLISION" for o in risk["obstacles"])
                    else "CLEAR_WITHIN_MODEL")


class ClosePassRecovery:
    """Reconfirm a previously permitted slow detour after a brief track gap."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.last_permitted = self.gap = None

    def observe(self, now, decision, intent, blocker):
        raw = decision["raw_corridor"]
        scan = raw.get("stamp")
        if not (blocker is not None and intent is not None and
                intent["route_snapshot"].get("standing_detour_active") is True and
                intent.get("guidance_hold") is False and intent.get("standing_detour_speed_cap_mps") == .06 and
                intent["route_revision"] == decision["route_revision"] and
                raw.get("valid") is True and raw.get("blocked") is False and
                raw.get("exact_sensor_time_tf") is True and finite(scan) and 0 <= now-scan <= .30):
            self.reset()
            return False
        key = (intent["route_revision"], blocker["track_id"])
        reason = decision["reason"]
        risk = decision.get("fusion_assessment", {})
        if reason == "CLOSE_PASS_TRACK_UNAVAILABLE" and risk.get("risk") != "POTENTIAL_COLLISION":
            self.observe_gap(now, key)
            return False
        evidence = decision.get("close_pass_evidence", {})
        support_scan = evidence.get("scan_stamp")
        if not (decision.get("close_pass_monitor") == "CLOSE_PASS_STATIONARY_SUPPORTED" and
                evidence.get("route_clearance_ok") is True and finite(support_scan) and
                abs(scan-support_scan) <= .001 and risk.get("valid") is True and
                risk.get("risk") == "CLEAR_WITHIN_MODEL" and
                (decision["action"] in ("SLOW", "BASELINE") or reason == "CONFIRMING_CLEAR_CORRIDOR")):
            self.reset()
            return False
        recovered = reason == "CONFIRMING_CLEAR_CORRIDOR" and self.confirm(now, key, scan, decision)
        if decision["action"] in ("SLOW", "BASELINE"):
            self.last_permitted = (key, now, scan)
            self.gap = None
        return recovered

    def observe_gap(self, now, key):
        gap = self.gap
        if gap is not None:
            if gap["key"] != key or not 0 <= now-gap["started"] <= .18 or gap["first_scan"] is not None:
                self.reset()
            return
        previous = self.last_permitted
        self.last_permitted = None
        if previous is not None and previous[0] == key and 0 <= now-previous[1] <= .15:
            self.gap = {"key": key, "started": now, "previous_scan": previous[2], "first_scan": None}

    def confirm(self, now, key, scan, decision):
        gap = self.gap
        if gap is None:
            return False
        elapsed = now-gap["started"]
        first = gap["first_scan"]
        if gap["key"] != key or not 0 <= elapsed <= (.18 if first is None else .45):
            self.reset()
            return False
        if scan <= max(gap["started"], gap["previous_scan"]):
            return False
        if first is None:
            gap["first_scan"] = scan
            return False
        if scan <= first:
            return False
        decision.update(action="SLOW", action_code=1, speed_cap_mps=.06,
                        reason="CLOSE_PASS_RESUMED_AFTER_TRACK_GAP", close_pass_resume=CLOSE_PASS_RESUME,
                        close_pass_resume_evidence={"gap_started_sim_s": gap["started"], "scan_stamps": [first, scan]})
        return True


class ClosePassMonitor:
    """Require fresh evidence from the same person during a close detour."""

    def __init__(self):
        self.blocker = None
        self.motion_detected = False
        self.motion_check_pending = False
        self.evidence = None
        self.track_gap = False
        self.reacquired_scan = None
        self.checked_release_scan = None
        self.recovery = ClosePassRecovery()

    def arm(self, blocker):
        if not self.motion_check_pending:
            self.blocker = blocker
            self.evidence = None
            self.track_gap = False
            self.reacquired_scan = self.checked_release_scan = None
            self.recovery.reset()

    def measurement(self, now, risk, fusion):
        if self.blocker is None or risk.get("valid") is not True or fusion is None:
            return None
        track_id = self.blocker["track_id"]
        matched = [item for item in risk["obstacles"] if item["id"] == track_id
                   and item.get("lidar_confirmed") is True]
        if len(matched) != 1:
            return None
        packet = fusion.get("packet") or {}
        for track in packet.get("tracks", []):
            if track.get("id") != track_id or track.get("source") not in ("vision_rgbd_tf", "laser_scan_tf"):
                continue
            support = track.get("lidar_support") or {}
            stamp, position, velocity, radius = (support.get(key) for key in (
                "stamp", "position", "measured_velocity", "radius"))
            if (finite(stamp) and 0 <= now-stamp <= .30 and xy(position) and xy(velocity)
                    and finite(radius) and .20 <= radius <= .40):
                return stamp, position, velocity, radius, track["source"], support.get("points")
        return None

    def checked_stationary_route(self, now, intent, raw, risk, fusion, robot_speed):
        if intent is None:
            return None, "ROUTE_INTENT_UNAVAILABLE"
        if intent["route_snapshot"].get("standing_detour_active") is not True:
            return None, "NO_ACTIVE_STANDING_DETOUR"
        if (intent.get("guidance_hold") is not False or intent.get("standing_detour_speed_cap_mps") != .06 or
                not 0 <= now-intent["stamp"] <= .30):
            return None, "SLOW_DETOUR_NOT_READY"
        if not (isinstance(raw, dict) and raw.get("valid") is True and raw.get("exact_sensor_time_tf") is True and
                isinstance(raw.get("blocked"), bool) and finite(raw.get("stamp"))):
            return None, "SCAN_NOT_READY"
        if not finite(robot_speed):
            return None, "WHEEL_ODOMETRY_UNAVAILABLE"
        if abs(robot_speed) > .07:
            return None, "WHEEL_SPEED_OVER_LIMIT"
        sample = self.measurement(now, risk, fusion)
        if sample is None:
            return None, "MATCHED_TRACK_UNAVAILABLE"
        stamp, position, velocity, radius, source, point_count = sample
        if not (isinstance(point_count, int) and point_count >= 6 and
                abs(raw["stamp"]-stamp) <= .001):
            return None, "TRACK_SCAN_NOT_MATCHED"
        if source == "vision_rgbd_tf":
            camera = next(item for item in risk["obstacles"] if item["id"] == self.blocker["track_id"])
            if math.hypot(*camera["velocity_mps"]) > STANDING_MOTION_LIMIT:
                return None, "VISION_REPORTS_MOTION"
            if math.dist(camera["position_m"], position) > STANDING_ANCHOR_SHIFT_M:
                return None, "VISION_LIDAR_POSITION_DISAGREEMENT"
        previous = self.evidence
        speed = math.hypot(*velocity)
        if not (previous is not None and not self.motion_check_pending and
                0 <= stamp-previous["stamp"] <= STANDING_MAX_SCAN_GAP_S and not previous["moving"] and
                previous["count"] >= 3 and stamp-previous["since"] >= STANDING_CONFIRMATION_S and
                (not self.track_gap or (self.reacquired_scan is not None and stamp > self.reacquired_scan)) and
                speed <= STANDING_MOTION_LIMIT and math.dist(position, previous["position"]) <= .08 and
                math.dist(position, self.blocker["center_odom"]) <= STANDING_ANCHOR_SHIFT_M):
            return None, "PERSON_NOT_STABLE"
        # Account for drift during the scan's age and the directive's 0.30 s lifetime.
        motion_margin = speed * (now-stamp+.30)
        if not standing_route_has_clearance(intent, position, radius, motion_margin):
            return None, "ROUTE_CLEARANCE_TOO_SMALL"
        proof = dict(track_id=self.blocker["track_id"], source=source, center_odom_m=position,
                     radius_m=radius, scan_stamp=stamp,
                     cluster_points=point_count,
                     required_separation_m=standing_required_separation(radius, motion_margin),
                     motion_margin_m=motion_margin)
        return proof, "CHECKED"

    def can_preserve_scan_history(self, now, decision):
        raw = decision.get("raw_corridor", {})
        stamp = raw.get("stamp")
        return (self.evidence is not None and not self.evidence["moving"] and not self.motion_check_pending and
                self.reacquired_scan is None and decision.get("route_revision") is not None and
                self.evidence["revision"] == decision["route_revision"] and
                raw.get("valid") is True and raw.get("exact_sensor_time_tf") is True and finite(stamp) and
                0 <= now-stamp <= .30 and 0 <= stamp-self.evidence["stamp"] <= STANDING_MAX_SCAN_GAP_S and
                all(item["id"] == self.blocker["track_id"]
                    for item in decision.get("fusion_assessment", {}).get("obstacles", [])))

    def confirm_checked_release(self, decision):
        proof = decision.get("standing_route_check")
        if not (proof is not None and decision["reason"] == "CONFIRMING_CLEAR_CORRIDOR" and
                decision["close_pass_monitor"] == "CLOSE_PASS_STATIONARY_SUPPORTED" and
                decision.get("standing_route_check_status") == "CHECKED" and
                decision["raw_corridor"]["blocked"] is False and
                decision["fusion_assessment"]["risk"] == "CLEAR_WITHIN_MODEL"):
            self.checked_release_scan = None
            return False
        key = (decision["route_revision"], proof["track_id"])
        scan = proof["scan_stamp"]
        previous = self.checked_release_scan
        if previous is None or key != previous[0] or not 0 <= scan-previous[1] <= STANDING_MAX_SCAN_GAP_S:
            self.checked_release_scan = (key, scan)
            return False
        if scan == previous[1]:
            return False
        decision.update(action="SLOW", action_code=1, speed_cap_mps=.06,
                        reason="CHECKED_STANDING_ROUTE",
                        standing_route_resume_scans=[previous[1], scan])
        self.checked_release_scan = None
        return True

    def restrict(self, now, decision, fusion, intent=None):
        sample = self.measurement(now, decision.get("fusion_assessment", {}), fusion)
        if sample is None or (self.evidence is not None and sample[0] < self.evidence["stamp"]):
            if sample is None and self.can_preserve_scan_history(now, decision):
                self.track_gap = True
                decision["standing_history_preserved"] = True
            else:
                self.evidence = self.reacquired_scan = None
                self.track_gap = False
            status = "CLOSE_PASS_TRACK_UNAVAILABLE"
        else:
            stamp, position, velocity, radius, source, _ = sample
            if source == "vision_rgbd_tf":
                camera = next(o for o in decision["fusion_assessment"]["obstacles"] if o["id"] == self.blocker["track_id"])
                if camera.get("velocity_mps") is not None and math.hypot(*camera["velocity_mps"]) > STANDING_MOTION_LIMIT:
                    velocity = camera["velocity_mps"]
            speed, offset = math.hypot(*velocity), math.dist(position, self.blocker["center_odom"])
            moving = speed > STANDING_MOTION_LIMIT
            clearance = None
            raw = decision.get("raw_corridor", {})
            if (intent is not None and raw.get("valid") is True and raw.get("exact_sensor_time_tf") is True
                    and decision.get("route_revision") == intent["route_revision"]
                    and intent["route_snapshot"].get("standing_detour_active") is True):
                clearance = standing_route_has_clearance(intent, position, radius, speed*(now-stamp+.30))
            if offset > STANDING_ANCHOR_SHIFT_M or clearance is False:
                self.motion_check_pending = True
            previous = self.evidence
            returned_with_current_scan = (raw.get("exact_sensor_time_tf") is True and finite(raw.get("stamp")) and
                                          abs(raw["stamp"]-stamp) <= .001)
            if (previous is None or moving != previous["moving"] or stamp-previous["stamp"] > .30
                    or (self.track_gap and (stamp-previous["stamp"] > STANDING_MAX_SCAN_GAP_S or
                                            previous["revision"] != decision.get("route_revision") or
                                            not returned_with_current_scan))
                    or (not moving and math.dist(position, previous["position"]) > .08)):
                self.evidence = {"since": stamp, "stamp": stamp, "count": 1, "moving": moving,
                                 "position": position, "revision": decision.get("route_revision")}
                self.track_gap = False
                self.reacquired_scan = None
            elif stamp > previous["stamp"]:
                previous["stamp"] = stamp
                previous["count"] += 1
                previous["revision"] = decision.get("route_revision")
                if self.track_gap:
                    if self.reacquired_scan is None:
                        self.reacquired_scan = stamp
                    else:
                        self.track_gap = False
                        self.reacquired_scan = None
            count = self.evidence["count"]
            duration = stamp - self.evidence["since"]
            decision["close_pass_evidence"] = dict(source=source, scan_stamp=stamp, speed_mps=speed,
                                                   offset_m=offset, scan_count=count, duration_sim_s=duration,
                                                   route_clearance_ok=clearance)
            status = self.motion_status(moving, count, duration)
            if status == "CLOSE_PASS_REVALIDATING_ROUTE":
                status = self.revalidate_route(intent, decision, position, clearance)
        decision["close_pass_monitor"] = status
        if status == "CLOSE_PASS_STATIONARY_SUPPORTED":
            return False
        if decision["action"] != "WAIT" or decision["reason"] == "CONFIRMING_CLEAR_CORRIDOR":
            decision.pop("clear_duration_sim_s", None)
            decision.pop("resume_confirmation", None)
            decision.update(action="WAIT", action_code=0, speed_cap_mps=0., reason=status)
        return True

    def motion_status(self, moving, count, duration):
        if moving:
            self.motion_check_pending = True
            if count >= 3 and duration >= .15:
                self.motion_detected = True
            return "CLOSE_PASS_PERSON_MOVED" if self.motion_detected else "CLOSE_PASS_MOTION_UNCONFIRMED"
        if self.motion_check_pending:
            if count < 3 or duration < STANDING_CONFIRMATION_S:
                return "CLOSE_PASS_RECHECKING_STATIONARITY"
            return "CLOSE_PASS_REVALIDATING_ROUTE"
        return "CLOSE_PASS_STATIONARY_SUPPORTED"

    def revalidate_route(self, intent, decision, position, clearance):
        if clearance is None:
            return "CLOSE_PASS_REVALIDATING_ROUTE"
        if clearance:
            self.blocker = dict(self.blocker, center_odom=position)
            self.motion_check_pending = self.motion_detected = False
            decision["close_pass_route_revalidated"] = True
            return "CLOSE_PASS_STATIONARY_SUPPORTED"
        transform = intent["map_to_odom"]
        dx, dy = position[0]-transform["x"], position[1]-transform["y"]
        c, s = math.cos(transform["yaw"]), math.sin(transform["yaw"])
        decision["close_pass_replan_request"] = {"center_map": [c*dx+s*dy, -s*dx+c*dy]}
        return "CLOSE_PASS_REPLAN_REQUIRED"


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
        return {"center_map": [c*dx+s*dy, -s*dx+c*dy], "center_odom": center, "image_stamp": stamp,
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
