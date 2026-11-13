"""Read a trial trace to distinguish obstacle stops from missing input stops."""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import statistics

from fusion_fixture import HEAD_ON_SCENARIOS, OCCLUDED_SCENARIO, STANDING_SCENARIO


def analyze(directory):
    directory = Path(directory)
    summary = json.loads((directory / "summary.json").read_text(encoding="utf-8"))
    records = []
    with (directory / "trace.jsonl").open(encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                records.append(json.loads(line))
    wait_sim, wait_wall = defaultdict(float), defaultdict(float)
    after_three = defaultdict(float)
    baseline_commands_after_three = []
    first_block_pose = None
    head_on = summary.get("scenario") in HEAD_ON_SCENARIOS
    occluded = summary.get("scenario") == OCCLUDED_SCENARIO
    standing = summary.get("scenario") == STANDING_SCENARIO
    clear_stamp = summary.get("head_on_timing", {}).get("actor_clearance_sim_s")
    wait_after_clear = defaultdict(float)
    first_detour = first_forward_after_clear = None
    first_slow = None
    slow_duration = 0.
    cruise_duration = cruise_after_clear = 0.
    detour_deferred_duration = 0.
    stopped_for_obstacle_before_clear = 0.
    max_lateral = None
    planned_max_lateral = None
    standing_release_wait = 0.
    for i, row in enumerate(records):
        decision = row["decision"]
        route = row.get("route", {})
        pose = route.get("robot_pose_map")
        command = row.get("guard_command")
        if (head_on or occluded) and first_detour is None and route.get("avoidance_state") == 1:
            first_detour = row["sim"]
        if (head_on or standing) and isinstance(pose, list) and len(pose) >= 2:
            max_lateral = max(max_lateral or 0., abs(pose[1]))
        if standing and route.get("standing_detour_active") is True and route.get("waypoints_map"):
            planned_max_lateral = max(planned_max_lateral or 0.,
                                      max(abs(point[1]) for point in route["waypoints_map"]))
        if head_on:
            if first_slow is None and decision.get("action") == "SLOW":
                first_slow = {"sim": row["sim"], "reason": decision.get("reason")}
                truth = row.get("scoring_only", {})
                if "robot_truth" in truth and "person_truth" in truth:
                    first_slow["circular_clearance_m"] = math.dist(truth["robot_truth"], truth["person_truth"])-.77
            if (first_forward_after_clear is None and clear_stamp is not None and row["sim"] >= clear_stamp
                    and isinstance(command, list) and command and command[0] > .01):
                first_forward_after_clear = row["sim"]
        if decision.get("reason") == "LIDAR_CORRIDOR_BLOCKED" and first_block_pose is None:
            first_block_pose = pose
        if (isinstance(pose, list) and pose[0] >= 3. and decision.get("action") == "BASELINE"
                and isinstance(command, list)):
            baseline_commands_after_three.append(abs(command[0]))
        if i+1 == len(records):
            continue
        nxt = records[i+1]
        dt = nxt["sim"]-row["sim"]
        if (standing and dt >= 0. and route.get("standing_detour_speed_limited") is True
                and route.get("behavior_gate", {}).get("reason") == "release_hysteresis"
                and decision.get("action") in ("SLOW", "BASELINE")
                and isinstance(command, list) and len(command) == 2
                and all(abs(value) <= .001 for value in command)):
            standing_release_wait += dt
        if route.get("detour_deferred_by_yield") is True and dt >= 0:
            detour_deferred_duration += dt
        if route.get("clear_cruise_active") is True and dt >= 0:
            cruise_duration += dt
            if clear_stamp is not None and nxt["sim"] > clear_stamp:
                cruise_after_clear += nxt["sim"]-max(clear_stamp, row["sim"])
        if head_on and decision.get("action") == "SLOW" and dt >= 0:
            slow_duration += dt
        wall_dt = nxt["wall"]-row["wall"] if "wall" in nxt and "wall" in row else None
        if decision.get("action") == "WAIT" and dt >= 0:
            reason = decision["reason"]
            wait_sim[reason] += dt
            if wall_dt is not None and wall_dt >= 0:
                wait_wall[reason] += wall_dt
            if isinstance(pose, list) and pose[0] >= 3.:
                after_three[reason] += dt
            if head_on and clear_stamp is not None and nxt["sim"] > clear_stamp:
                wait_after_clear[reason] += nxt["sim"]-max(clear_stamp, row["sim"])
            if (head_on and clear_stamp is not None and row["sim"] < clear_stamp
                    and reason in ("LIDAR_CORRIDOR_BLOCKED", "VISION_PREDICTED_CONFLICT",
                                   "HUAN_FUSION_PREDICTED_CONFLICT")
                    and isinstance(command, list) and len(command) == 2
                    and all(abs(value) <= .001 for value in command)):
                stopped_for_obstacle_before_clear += min(nxt["sim"], clear_stamp)-row["sim"]
    elapsed_wall, elapsed_sim = summary.get("elapsed_wall_s"), summary.get("elapsed_sim_s")
    report = dict(
        source="trace decision intervals; recorded commands, not measured motor speed",
        yield_clearance_m=summary.get("yield_clearance_m"),
        yield_clearance_reference=summary.get("yield_clearance_reference"),
        rows=len(records), first_lidar_block_pose_map=first_block_pose,
        resume_profile=summary.get("resume_profile", "standard"),
        caution_max_clearance_m=summary.get("caution_max_clearance_m"),
        clear_cruise_enabled=summary.get("clear_cruise_enabled", False),
        clear_cruise_active_sim_s=cruise_duration,
        yield_before_detour=summary.get("yield_before_detour", False),
        detour_deferred_by_yield_sim_s=detour_deferred_duration,
        fast_sensor_gap_releases=sum(row["decision"].get("resume_confirmation") == "TWO_NEW_CLEAR_EXACT_TF_SCANS"
                                     for row in records),
        wait_duration_sim_s_by_reason=dict(wait_sim), wait_duration_wall_s_by_reason=dict(wait_wall),
        wait_after_x3_sim_s_by_reason=dict(after_three),
        baseline_guard_command_after_x3_median_mps=(statistics.median(baseline_commands_after_three)
                                                   if baseline_commands_after_three else None),
        observed_rtf=elapsed_sim/elapsed_wall if elapsed_wall and elapsed_sim is not None else None,
        limitations=["Intervals use sample-and-hold attribution until the next recorded decision.",
                     "This cannot establish contact, motor stop latency or real vehicle safety.",
                     "Raw field validity is still determined by the trial; this report never changes its score."])
    if head_on:
        report["head_on"] = dict(
            actor_clearance_sim_s=clear_stamp,
            stopped_command_for_obstacle_before_actor_clearance_sim_s=stopped_for_obstacle_before_clear,
            first_slow=first_slow, slow_decision_duration_sim_s=slow_duration,
            clear_cruise_after_actor_clearance_sim_s=cruise_after_clear,
            wait_after_actor_clearance_sim_s_by_reason=dict(wait_after_clear),
            first_detour_sim_s=first_detour,
            first_forward_guard_command_delay_sim_s=(first_forward_after_clear-clear_stamp
                                                      if first_forward_after_clear is not None else None),
            max_abs_lateral_map_m=max_lateral,
            last_trace_robot_pose_map=records[-1].get("route", {}).get("robot_pose_map") if records else None,
        )
    if occluded:
        timing = summary.get("occlusion_timing", {})
        evidence = summary.get("occlusion", {})
        actor_exit = timing.get("actor_clearance_sim_s")
        forward = evidence.get("first_forward_after_actor_exit_sim_s")
        report["occlusion"] = dict(evidence, actor_screen_edge_sim_s=timing.get("actor_screen_edge_sim_s"),
                                   actor_clearance_sim_s=actor_exit,
                                   first_detour_sim_s=first_detour,
                                   first_forward_guard_command_delay_sim_s=(forward-actor_exit
                                       if forward is not None and actor_exit is not None else None))
    if standing:
        timing = summary.get("standing_blocker_timing", {})
        evidence = summary.get("standing_blocker", {})
        center, bypass = timing.get("blocker_center_entry_sim_s"), evidence.get("bypassed_standing_person_sim_s")
        arrival = summary.get("arrival_sim_s")
        rejoin_cruise = next((row["sim"] for row in records
            if row["route"].get("standing_detour_active") is True
            and row["route"].get("standing_detour_speed_limited") is False
            and row["guard_command"][0] > .061), None)
        report["standing_blocker"] = dict(evidence, blocker_center_entry_sim_s=center,
            standing_detour_enabled=summary.get("standing_detour_enabled", False),
            first_standing_approach_hold_sim_s=summary.get("first_standing_approach_hold_sim_s"),
            first_standing_detour_request_sim_s=summary.get("first_standing_detour_request_sim_s"),
            first_standing_detour_request_center_distance_m=summary.get(
                "first_standing_detour_request_center_distance_m"),
            first_standing_detour_seen_sim_s=summary.get("first_standing_detour_seen_sim_s"),
            first_standing_rejoin_cruise_sim_s=rejoin_cruise,
            zero_command_release_confirmation_sim_s=standing_release_wait,
            bypass_delay_sim_s=(bypass-center if bypass is not None and center is not None else None),
            bypass_to_arrival_sim_s=(arrival-bypass if arrival is not None and bypass is not None else None),
            max_abs_lateral_map_m=max_lateral, planned_max_abs_lateral_map_m=planned_max_lateral,
            last_trace_robot_pose_map=records[-1].get("route", {}).get("robot_pose_map") if records else None)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    report = analyze(args.run)
    (args.run / "performance_review.json").write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
