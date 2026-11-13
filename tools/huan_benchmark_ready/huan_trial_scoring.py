"""Explicit comparison criteria; the original nine results are always retained."""
import math

from fusion_fixture import STANDING_BLOCK_X


def comparison_score(metrics, mode, integration_checks=None, *, early_rejoin=False,
                     early_rejoin_observed=False):
    original = dict(metrics["checks"])
    applicable = dict(original)
    excluded = []
    if mode == "lidar-only":
        excluded = ["vision_actually_limited_moving_speed"]
        applicable.pop(excluded[0])
    elif mode not in ("huan", "legacy"):
        raise ValueError("Unknown comparison mode")
    if early_rejoin:
        stop_checks = ("lidar_driven_wait_observed", "resumed_after_wait", "moved_before_block")
        excluded.extend(stop_checks)
        for name in stop_checks:
            applicable.pop(name, None)
        # Require evidence even when re-scoring an older or incomplete record.
        for name in ("one_goal", "arrival", "person_crossed_during_mission",
                     "circular_clearance_at_least_20cm"):
            applicable[name] = original.get(name) is True
        applicable["early_rejoin_observed"] = early_rejoin_observed is True
    extra = {}
    if mode == "huan":
        for name in ("fusion_ready_before_goal", "huan_fusion_used_during_mission"):
            extra[name] = (integration_checks or {}).get(name) is True
        applicable.update(extra)
    return dict(metrics, checks=applicable, applicable_checks=dict(applicable),
                original_9_checks=original, original_9_checks_passed=all(original.values()),
                not_applicable_checks=excluded, integration_checks=extra,
                passed=all(applicable.values()),
                score_objective="early_rejoin" if early_rejoin else "lidar_yield_stop",
                score_contract=("Early rejoin requires an observed rejoin, arrival and clearance. "
                                "Three LiDAR-stop checks are outside this objective. Original nine results preserved."
                                if early_rejoin else
                                "Original nine results preserved; only lidar-only excludes camera-effect; "
                                "Huan mode additionally requires actual fusion input use"))


def encounter_score(metrics, integration_checks=None, *, scenario="head-on-wide"):
    original = dict(metrics["checks"])
    checks = {name: original.get(name) is True
              for name in ("one_goal", "arrival", "circular_clearance_at_least_20cm")}
    checks.update(integration_checks or {})
    return dict(metrics, checks=checks, applicable_checks=dict(checks),
                original_9_checks=original, passed=all(checks.values()),
                score_objective=scenario.replace("-", "_"),
                score_contract="One goal, arrival and clearance. Detour, wait and rejoin are diagnostics.")


def observe_occlusion(report, row, initial_robot, actor_motion, actor_exit):
    """Record sensor and command milestones for scoring after policy evaluation."""
    now = row["sim"]
    robot = row["scoring_only"]["robot_truth"]
    if actor_motion is None or now < actor_motion:
        report["robot_advance_before_actor_motion_m"] = max(
            report["robot_advance_before_actor_motion_m"], math.dist(initial_robot, robot))
    if row.get("input_error") is None and any(item["position_ok"] for item in row["matches"]):
        report.setdefault("first_stereo_position_sim_s", now)
    decision = row["decision"]
    reason = decision["reason"]
    v, w = row["guard_command"]
    stopped = (decision["action"] == "WAIT" and abs(v) <= .001 and abs(w) <= .001
               and reason in ("LIDAR_CORRIDOR_BLOCKED", "VISION_PREDICTED_CONFLICT", "HUAN_FUSION_PREDICTED_CONFLICT"))
    slowed = decision["action"] == "SLOW" and .01 < v <= .061
    if (actor_exit is None or now < actor_exit) and (stopped or slowed):
        event = {"sim": now, "action": decision["action"], "reason": reason,
                 "robot_pose_map": row["route"]["robot_pose_map"]}
        report.setdefault("first_obstacle_response", event)
        if stopped:
            report.setdefault("first_obstacle_wait", event)
    if actor_exit is not None and now >= actor_exit and v > .01:
        report.setdefault("first_forward_after_actor_exit_sim_s", now)


def observe_standing_blocker(report, row, initial_robot, actor_motion, center_entry):
    """Verify the blocker remains in the lane while the robot passes it."""
    now = row["sim"]
    robot, person = (row["scoring_only"][key] for key in ("robot_truth", "person_truth"))
    if actor_motion is None or now < actor_motion:
        report["robot_advance_before_actor_motion_m"] = max(
            report["robot_advance_before_actor_motion_m"], math.dist(initial_robot, robot))
    if row["route"].get("avoidance_state") == 1:
        report.setdefault("first_detour_sim_s", now)
    # Allow the last few centimetres of the scripted entrance to finish.
    if center_entry is None or now < center_entry+.8:
        return
    standing_in_lane = abs(person[0]-STANDING_BLOCK_X) <= .15 and abs(person[1]) <= .20
    key = "samples_standing_in_lane" if standing_in_lane else "samples_outside_lane_after_entry"
    report[key] += 1
    if standing_in_lane and robot[0] >= STANDING_BLOCK_X+.60:
        report.setdefault("bypassed_standing_person_sim_s", now)
