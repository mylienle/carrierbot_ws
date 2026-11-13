#!/usr/bin/env python3
"""Sensor-driven Gazebo trials with truth restricted to fixture verification and scoring."""
from collections import Counter
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import threading

from analyze_huan_run import outline_gaps
from fablab_route import finite
from fusion_fixture import (ACTOR, GOAL_LATE_LIMIT, HEAD_ON_SCENARIOS, MODE, NARROW_DETOUR_WALL_MARGIN_M,
                            OCCLUDED_SCENARIO, PROXY, SCENARIOS, STANDING_CENTER_GAPS_M,
                            STANDING_CLOSE_PASS_GAP_M, STANDING_GOAL_X, STANDING_SCENARIO, TOOLSET,
                            CrossingTiming, factory_arguments, insertion_report)
from huan_fusion_bridge import STANDING_LASER_RESERVE_M
from huan_fusion_policy import (CRAWL_COMMAND_CAP_MPS, ClosePassMonitor, IntegratedFusionPolicy,
                                StationaryBlocker, identify_input_wait, stabilize_standing_detour_release)
from fusion_yield_policy import NEARBY_SLOW_CLEARANCE_M
from huan_trial_scoring import comparison_score, encounter_score, observe_occlusion, observe_standing_blocker
from huan_cpu_metadata import configure_runtime_threads


ROBOT_SCORING_RADIUS_M = .52
PERSON_PROXY_RADIUS_M = .25


def fresh_wheel_speed(sample, sim, wall):
    if sample is None:
        return None
    stamp, received, speed = sample
    if 0 <= sim-stamp <= .25 and 0 <= wall-received <= .4 and finite(speed) and speed >= 0.:
        return speed
    return None


def synchronized_goal_ready(target, clock_snapshot, inputs_ready, route_ready, report):
    """Keep the sensor history live and dispatch only inside the fixture's existing window."""
    stamp_ns, _, fresh = clock_snapshot()
    report["clock_fresh"] = fresh
    if not fresh:
        return False
    sim = stamp_ns * 1e-9
    if sim > target + GOAL_LATE_LIMIT:
        raise RuntimeError("Missed the synchronized goal slot; see goal_slot_preflight in summary.json")
    # A preflight that passed several seconds ago can no longer supply a live sensor history.
    ready = inputs_ready()
    if sim < target:
        return False
    route_ok = route_ready()
    stamp_ns, _, fresh = clock_snapshot()
    checked_at = stamp_ns * 1e-9
    report.update(clock_fresh=fresh, sensor_inputs_ready=ready, command_route_ready=route_ok,
                  checked_at_sim_s=checked_at)
    if fresh and checked_at > target + GOAL_LATE_LIMIT:
        raise RuntimeError("Missed the synchronized goal slot; see goal_slot_preflight in summary.json")
    return fresh and target <= checked_at and ready and route_ok


def prepare_vision(vision, tools, output, velocity_profile="original", vision_threads=1, *,
                   reliable_rgbd=False):
    import yaml
    entry = vision/"main_vm_cpu.py"
    for path in (entry, vision/"config.vm.cpu.yaml", vision/".venv-vm/bin/python",
                 vision/"yolo11n.pt", vision/"yolo11s-pose.pt"):
        if not path.is_file():
            raise RuntimeError("Missing previously tested Vision input: " + str(path))
    core = json.loads((tools/"huan_vision_core.json").read_text(encoding="utf-8"))
    for relative, expected in core["normalized_sha256"].items():
        path = vision/relative
        if not path.is_file() or hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest() != expected:
            raise RuntimeError("Vision core differs from reviewed Huan source: " + relative)
    source = entry.read_text(encoding="utf-8")
    old_import = "from src.torso_depth import PoseTorsoDetector, associate_poses, measure_torso_depth"
    old_call = "poses = pose_detector.detect(frame) if pose_detector is not None else []"
    if 'device="cpu"' not in source or source.count(old_import) != 1 or source.count(old_call) != 1:
        raise RuntimeError("CPU Vision source differs from the reviewed version")
    source = source.replace(old_import, "from src.torso_depth import associate_poses, measure_torso_depth\n"
                                      "from carrierbot_roi_pose import PoseTorsoDetector")
    source = source.replace(old_call, "poses = pose_detector.detect(frame, tracks) if pose_detector is not None else []")
    source = configure_runtime_threads(source, vision_threads)
    source = ("import sys\nsys.path.insert(0, %r)\nsys.path.insert(0, %r)\n"
              "import torch, cv2\ntorch.set_num_threads(%d)\ntorch.set_num_interop_threads(1)\ncv2.setNumThreads(1)\n"
              "from huan_cpu_metadata import install as install_huan_metadata\n"
              "install_huan_metadata(%r)\n" % (str(vision), str(tools), vision_threads, velocity_profile)) + source
    compile(source, "main_fusion_roi_cpu.py", "exec")
    target = output/"main_fusion_roi_cpu.py"
    target.write_text(source, encoding="utf-8")
    config = yaml.safe_load((vision/"config.vm.cpu.yaml").read_text(encoding="utf-8"))
    if config.get("detection", {}).get("class_id") != 0:
        raise RuntimeError("This trial expects HuÃ¢n's existing person-only detector")
    config["input"]["reference_frame"] = "odom"
    if reliable_rgbd:
        config["input"]["reliable_rgbd"] = True
    config["motion_compensation"] = {"enabled": True, "reference_source": "tf2_ekf"}
    config["prediction"]["enabled"] = True
    config["output"].update(show_window=False, save_csv=False, save_video=False)
    config_path = output/"vision_config.yaml"
    config_path.write_text(yaml.safe_dump(config), encoding="utf-8")
    return [str(vision/".venv-vm/bin/python"), "-u", str(target), "--config", str(config_path), "--max-frames", "0"]


class TrialScore:
    """Test scoring only. Ground truth never enters FusionYieldPolicy."""
    def __init__(self):
        self.minimum_clearance = float("inf")
        self.ymin, self.ymax = float("inf"), -float("inf")
        self.slow_images = set()
        self.lidar_wait_samples = 0
        self.wait_origin = None
        self.wait_max_drift = 0.
        self.resume_origin = None
        self.resume_displacement = 0.
        self.first_block_xy = None
        self.maximum_speed = 0.
        self.last_action, self.since = None, None
        self.wait_route = None
        self.wait_planning_revision = None

    def observe(self, now, decision, route, command, robot_truth, person_truth):
        if not all(math.isfinite(v) for point in (robot_truth, person_truth) for v in point):
            raise RuntimeError("Non-finite scoring position")
        separation = math.dist(robot_truth, person_truth)-ROBOT_SCORING_RADIUS_M-PERSON_PROXY_RADIUS_M
        self.minimum_clearance = min(self.minimum_clearance, separation)
        self.ymin, self.ymax = min(self.ymin, person_truth[1]), max(self.ymax, person_truth[1])
        if separation < .20:
            raise RuntimeError("Circular-model clearance fell below the trial's 0.20 m threshold")
        v, w = command
        if not all(math.isfinite(x) for x in command) or v < -.001 or abs(v) > .121 or abs(w) > .301:
            raise RuntimeError("Unexpected guard output in this forward-only 0.12 m/s trial")
        self.maximum_speed = max(self.maximum_speed, abs(v))
        action = decision["action"]
        if action != self.last_action:
            self.since, self.last_action = now, action
            self.wait_origin, self.wait_route = None, None
            self.wait_planning_revision = None
        blocked = decision["reason"] == "LIDAR_CORRIDOR_BLOCKED"
        if blocked and self.first_block_xy is None:
            self.first_block_xy = list(robot_truth)
        if action == "WAIT" and now-self.since >= .4:
            upstream = route.get("command_preview", {})
            if any(abs(x) > .001 for x in [v, w, upstream.get("linear_x", float("inf")),
                                           upstream.get("angular_z", float("inf"))]):
                raise RuntimeError("Robot command remained nonzero after a sensor-driven WAIT")
            observed = (route.get("route_revision"), route.get("current_segment"), route.get("waypoints_map"))
            if self.wait_origin is None:
                self.wait_origin = list(robot_truth)
            elif self.wait_route != observed:
                old_revision, _, old_points = self.wait_route
                new_revision, _, new_points = observed
                if not (isinstance(old_revision, int) and old_revision == self.wait_planning_revision
                        and new_revision == old_revision+1 and route.get("standing_detour_active") is True
                        and old_points and new_points and new_points != old_points
                        and new_points[-1] == old_points[-1]):
                    raise RuntimeError("Route changed while the sensor gate held WAIT")
                self.wait_planning_revision = None
            self.wait_route = observed
            self.wait_max_drift = max(self.wait_max_drift, math.dist(robot_truth, self.wait_origin))
            if self.wait_max_drift > .03:
                raise RuntimeError("Robot drifted over 3 cm during a settled WAIT")
            self.lidar_wait_samples += int(blocked)
        if action == "WAIT" and (decision.get("standing_detour_request") is not None
                                 or decision.get("close_pass_replan_request") is not None):
            self.wait_planning_revision = decision.get("route_revision")
        if action == "SLOW" and now-self.since >= .4:
            uv = route.get("command_preview", {}).get("linear_x", float("inf"))
            if abs(v) > .061 or abs(uv) > .061:
                raise RuntimeError("Measured commands exceeded the sensor-driven SLOW cap")
            if v > .01:
                self.slow_images.update(item["image_stamp"] for item in decision["vision_observations"]
                                        if item["within_caution_corridor"])
        if self.lidar_wait_samples and action != "WAIT" and v > .01:
            if self.resume_origin is None:
                self.resume_origin = list(robot_truth)
            self.resume_displacement = max(self.resume_displacement, math.dist(robot_truth, self.resume_origin))

    def finish(self, start_truth, matched_positions, arrival, error, goal_count):
        checks = {"one_goal": goal_count == 1, "arrival": arrival and error <= .18,
                  "matched_vision_positions": matched_positions >= 10,
                  "vision_actually_limited_moving_speed": len(self.slow_images) >= 2,
                  "lidar_driven_wait_observed": self.lidar_wait_samples >= 3,
                  "resumed_after_wait": self.resume_displacement >= .10,
                  "moved_before_block": self.first_block_xy is not None and math.dist(start_truth, self.first_block_xy) >= .10,
                  "person_crossed_during_mission": self.ymin <= -1.8 and self.ymax >= 2.,
                  "circular_clearance_at_least_20cm": self.minimum_clearance >= .20}
        return {"passed": all(checks.values()), "checks": checks,
                "minimum_clearance_m": self.minimum_clearance,
                "clearance_model": "robot circle 0.52 m, person collision proxy circle 0.25 m",
                "slow_unique_vision_images": len(self.slow_images), "lidar_wait_samples": self.lidar_wait_samples,
                "max_settled_wait_drift_m": self.wait_max_drift, "resumed_displacement_m": self.resume_displacement,
                "max_guard_linear_command_mps": self.maximum_speed,
                "person_y_range_during_mission": [self.ymin, self.ymax]}


def main():
    parser = argparse.ArgumentParser(description="Huan fusion -> Fablab -> existing safety_guard, Gazebo only")
    parser.add_argument("--mode", choices=IntegratedFusionPolicy.MODES, default="huan")
    parser.add_argument("--scenario", choices=SCENARIOS, default="crossing")
    parser.add_argument("--resume-profile", choices=("standard", "fresh-scans"), default="standard",
                        help="Experimental short-gap release after two new clear scans with exact-time TF")
    parser.add_argument("--nearby-slow", action="store_true",
                        help="Experimental 2.0 m body-to-obstacle clearance bound for caution only")
    parser.add_argument("--standing-detour", action="store_true",
                        help="Allow Fablab to prepare a detour during WAIT for a matched stationary person")
    parser.add_argument("--reduced-standing-laser-reserve", action="store_true",
                        help="Test a nominal 0.50 m reserve for continuously tracked slow laser obstacles during a detour")
    parser.add_argument("--crawl-on-predicted-conflict", action="store_true",
                        help="Test a capped slow release if Huân predicts the detour is clear at crawl speed")
    parser.add_argument("--velocity-profile", choices=("original", "huan-responsive", "cpu-sparse"), default="original")
    parser.add_argument("--vision-threads", type=int, choices=(1, 2), default=1)
    parser.add_argument("--reliable-rgbd", action="store_true",
                        help="Use Huan's reliable RGB/depth subscriptions in the Gazebo trial")
    parser.add_argument("--input-only", action="store_true", help="Check sensor initialization, send no waypoint")
    args = parser.parse_args()
    if not args.input_only and not args.reliable_rgbd:
        parser.error("The synchronized benchmark requires --reliable-rgbd")
    if args.nearby_slow and args.mode == "lidar-only":
        parser.error("--nearby-slow requires camera-based caution")
    if args.standing_detour and (args.mode != "huan" or args.input_only):
        parser.error("--standing-detour requires an active --mode huan trial")
    if args.reduced_standing_laser_reserve and (args.scenario != STANDING_SCENARIO or not args.standing_detour):
        parser.error("--reduced-standing-laser-reserve requires --scenario standing-blocker --standing-detour")
    if args.crawl_on_predicted_conflict and (args.scenario != STANDING_SCENARIO or not args.standing_detour):
        parser.error("--crawl-on-predicted-conflict requires --scenario standing-blocker --standing-detour")
    head_on = args.scenario in HEAD_ON_SCENARIOS
    occluded = args.scenario == OCCLUDED_SCENARIO
    standing = args.scenario == STANDING_SCENARIO
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from gazebo_msgs.msg import ModelStates
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from std_msgs.msg import Bool, Float64MultiArray, String
    from tf2_ros import Buffer, TransformException, TransformListener
    from fusion_sensor_adapter import FusionSensors, SensorTimeWindow
    from huan_fusion_vendor.sim_clock_feed import SimulationClockFeed

    rclpy.init()
    node = rclpy.create_node("fusion_yield_trial")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    clock_feed = SimulationClockFeed(node)
    crossing_timing = CrossingTiming(args.scenario)
    root = Path.home()/"carrierbot_test_runs"
    root.mkdir(exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix=time.strftime("huan_fusion_%Y%m%d_%H%M%S_"), dir=str(root)))
    tracks = (output/"tracks.jsonl").open("w", encoding="utf-8")
    trace = (output/"trace.jsonl").open("w", encoding="utf-8")
    sensors = FusionSensors(node, buffer, tracks, clock=clock_feed.seconds, fusion=args.mode == "huan")
    state = {"config": None, "fixture": None, "fixture_wall": 0., "command": [0., 0.],
             "command_wall": 0., "truth_wall": 0., "truth": {}, "arrived": False, "wheel_odometry": None}
    def dictionary(message, key):
        try:
            value = json.loads(message.data)
            if isinstance(value, dict):
                state[key], state[key+"_wall"] = value, time.monotonic()
        except ValueError:
            pass
    def wheel(message):
        state.update(command=[message.linear.x, message.angular.z], command_wall=time.monotonic())
    def wheel_odometry(message):
        linear = message.twist.twist.linear
        sample = message.header.stamp
        state["wheel_odometry"] = (sample.sec + sample.nanosec * 1e-9, time.monotonic(),
                                    math.hypot(linear.x, linear.y))
    def truth(message):
        positions = {name: [p.position.x, p.position.y] for name, p in zip(message.name, message.pose)}
        sim_stamp, _, fresh = clock_feed.snapshot()
        if fresh and ACTOR in positions:
            crossing_timing.observe(sim_stamp*1e-9, positions[ACTOR])
        state["truth"] = positions
        state["truth_wall"] = time.monotonic()
    subscriptions = [node.create_subscription(String, "/test_fixture/fusion_config", lambda m: dictionary(m, "config"),
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(String, "/test_fixture/person_lidar_status", lambda m: dictionary(m, "fixture"), 1),
        node.create_subscription(Twist, "/carrierbot_controller/cmd_vel_unstamped", wheel, 1),
        node.create_subscription(Odometry, "/odom", wheel_odometry, qos_profile_sensor_data),
        node.create_subscription(ModelStates, "/gazebo/model_states", truth, qos_profile_sensor_data),
        node.create_subscription(Bool, "/behavior_gate/arrival", lambda m: state.update(arrived=bool(m.data)), 1)]
    directive = node.create_publisher(Float64MultiArray, "/dynamic_behavior/directive", 1)
    goals = node.create_publisher(PoseStamped, "/behavior_gate/waypoints", 1)
    sequence = 0
    children, logs = [], []
    result = {"stage": "huan_input_readiness" if args.input_only else "huan_fablab_crossing", "toolset": TOOLSET,
              "scenario": args.scenario, "resume_profile": args.resume_profile,
              "standing_detour_enabled": args.standing_detour,
              "caution_max_clearance_m": NEARBY_SLOW_CLEARANCE_M if args.nearby_slow else None,
              "integration_mode": args.mode, "velocity_profile": args.velocity_profile,
              "velocity_profile_experimental": args.velocity_profile == "cpu-sparse",
              "input_readiness_only": args.input_only, "navigation_evaluated": False,
              "velocity_window_sim_s": {"original": .8, "huan-responsive": .6, "cpu-sparse": 1.4}[args.velocity_profile],
              "vision_threads": args.vision_threads, "reliable_rgbd_requested": args.reliable_rgbd,
              "passed": False, "actuation": False,
              "actuator_route_connected": True, "phase": "preflight",
              "sensor_decision_evaluated": False, "vision_used": False, "goal_sent_count": 0,
              "scope": "one Gazebo human/cylinder crossing; yielding, not a general navigation benchmark",
              "truth_use": "fixture verification and scoring only; never passed to decision policy"}
    if head_on:
        if not args.input_only:
            result["stage"] = "huan_fablab_" + args.scenario.replace("-", "_")
        location = "the narrow corridor" if args.scenario == "head-on-narrow" else "the wide room"
        result["scope"] = "One head-on encounter in " + location + " followed by a lateral exit"
    if occluded:
        if not args.input_only:
            result["stage"] = "huan_fablab_occluded_crossing"
        result["scope"] = "One lateral crossing from behind a mapped side screen"
        result["occlusion"] = {"robot_advance_before_actor_motion_m": 0.}
    if standing:
        if not args.input_only:
            result["stage"] = "huan_fablab_standing_blocker"
        result["scope"] = "One person enters and remains standing in the middle of the route"
        result["standing_blocker"] = {"robot_advance_before_actor_motion_m": 0.,
                                      "samples_standing_in_lane": 0, "samples_outside_lane_after_entry": 0}
    if args.reduced_standing_laser_reserve:
        result["standing_laser_reserve"] = dict(nominal_m=STANDING_LASER_RESERVE_M,
                                                applied_samples=0, original_conflict_cleared_samples=0)
    if args.crawl_on_predicted_conflict:
        result["crawl_forecast"] = dict(command_cap_mps=CRAWL_COMMAND_CAP_MPS, checked_samples=0,
                                       clear_samples=0, slow_directive_samples=0)
    score = TrialScore()
    def now():
        return clock_feed.seconds()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    callback_thread = threading.Thread(target=executor.spin, name="fusion-ros-inputs", daemon=True)
    callback_thread.start()
    def pump(seconds):
        if not callback_thread.is_alive():
            raise RuntimeError("ROS input executor stopped; motion permission denied")
        time.sleep(seconds)
    def names(topic, subscribers=False):
        peers = node.get_subscriptions_info_by_topic(topic) if subscribers else node.get_publishers_info_by_topic(topic)
        return [p.node_name for p in peers]
    def graph():
        expected = {"/safety/input_cmd_vel": ["fablab_behavior_guidance"],
            "/carrierbot_controller/cmd_vel_unstamped": ["carrierbot_safety_guard"],
            "/dynamic_behavior/directive": [node.get_name()], "/behavior_gate/waypoints": [node.get_name()],
            "/behavior_gate/route_state": ["fablab_behavior_guidance"],
            "/test_fixture/fusion_config": ["lien_test_startup"]}
        return (all(names(topic) == nodes for topic, nodes in expected.items()) and
                names("/safety/input_cmd_vel", True) == ["carrierbot_safety_guard"] and
                names("/dynamic_behavior/directive", True) == ["fablab_behavior_guidance"] and
                names("/behavior_gate/waypoints", True) == ["fablab_behavior_guidance"] and
                names("/carrierbot_controller/cmd_vel_unstamped", True).count("carrierbot_controller") == 1)
    def send(action=0, cap=0., revision=None, standing_blocker=None):
        nonlocal sequence
        snapshot = sensors.route.latest
        if snapshot is None:
            return
        revision = snapshot.get("route_revision") if revision is None else revision
        if not isinstance(revision, int):
            return
        sequence += 1
        message = Float64MultiArray()
        message.data = [1., now(), float(sequence), float(action), cap, float(revision)]
        if standing_blocker is not None:
            message.data = [2., now(), float(sequence), 0., 0., float(revision), *standing_blocker]
        directive.publish(message)
    def launch(label, arguments, cwd=None):
        log = (output/(label+".log")).open("w", encoding="utf-8")
        logs.append(log)
        process = subprocess.Popen(arguments, cwd=str(cwd) if cwd else None, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        children.append((label, process))
        return process
    def spin_wait(predicate, seconds, label, *, poll_seconds=.05):
        deadline, sent = time.monotonic()+seconds, 0.
        while time.monotonic() < deadline:
            pump(poll_seconds)
            if time.monotonic()-sent >= .1:
                send()
                sent = time.monotonic()
            for name, process in children:
                if process.poll() is not None:
                    raise RuntimeError(name+" stopped; see its log")
            if predicate():
                return
        raise RuntimeError("Timed out: " + label)
    print("TOOLSET: "+TOOLSET+"\n"
          "CA THU: "+args.scenario+" | LiDAR + stereo -> Fablab -> safety_guard.\nLOG: "+str(output), flush=True)
    print("MODE: %s | VELOCITY FILTER: %s | CPU THREADS: %d | same Fablab/guard limits 0.12/0.06 m/s" %
          (args.mode, args.velocity_profile, args.vision_threads), flush=True)
    print("RESUME PROFILE: "+args.resume_profile, flush=True)
    if args.reduced_standing_laser_reserve:
        print("DU PHONG DU BAO KHI NE: %.2f m + bu tuoi scan; chi track LiDAR gan dung yen." %
              STANDING_LASER_RESERVE_M, flush=True)
    if args.nearby_slow:
        print("VUNG GIAM TOC: toi da %.1f m khoang ho theo mo hinh." % NEARBY_SLOW_CLEARANCE_M, flush=True)
    if args.input_only:
        print("CHE DO KIEM TRA DAU VAO: giu WAIT, khong gui waypoint; khong cham dieu huong.", flush=True)
    started_wall = started_sim = None
    try:
        spin_wait(lambda: graph() and sensors.route.message_ready() and state["config"] is not None
                  and time.monotonic()-state["command_wall"] <= .6 and "carrierbot" in state["truth"],
                  20., "start start_fusion_yield.sh, wait for SAN SANG THU LIDAR + STEREO")
        config = state["config"]
        if (config.get("mode") != MODE or config.get("source") != "lien_test_startup"
                or config.get("integration_toolset") != TOOLSET):
            raise RuntimeError("Wrong startup version. Start the synchronized benchmark from huan_benchmark_ready")
        if config.get("scenario", "crossing") != args.scenario:
            raise RuntimeError("Use the same --scenario in startup and trial; no mission was sent")
        clearance = config.get("yield_clearance_m")
        if clearance is not None and (not isinstance(clearance, (float, int)) or isinstance(clearance, bool)
                                      or not math.isfinite(clearance) or not .6 <= clearance <= 3.):
            raise RuntimeError("Invalid startup LiDAR yield clearance")
        sensors.yield_clearance_m = clearance
        result["yield_clearance_m"] = clearance
        result["yield_clearance_reference"] = config["yield_clearance_reference"]
        result["early_rejoin_enabled"] = config.get("early_rejoin_enabled") is True
        result["final_approach_ramp_enabled"] = config.get("final_approach_ramp_enabled") is True
        result["early_rejoin_observed"] = False
        print("VUNG NHUONG LIDAR: " + ("legacy 3m corridor" if clearance is None else
              "%.2f m tu be mat than xe" % clearance), flush=True)
        tools = Path(__file__).resolve().parent
        (output/"integration_manifest.json").write_bytes((tools/"integration_manifest.json").read_bytes())
        (output/"huan_source_manifest.json").write_bytes((tools/"huan_fusion_vendor/manifest.json").read_bytes())
        session = json.loads(Path(config["startup_session_file"]).read_text(encoding="utf-8"))
        if session.get("sim_lidar_rate_hz") is not None:
            result["sim_lidar_rate_hz"] = session["sim_lidar_rate_hz"]
        result["clear_cruise_enabled"] = session.get("clear_cruise_enabled") is True
        result["yield_before_detour"] = session.get("yield_before_detour") is True
        result["compact_standing_detour"] = session.get("compact_standing_detour") is True
        center_gap = session.get("planned_standing_center_gap_m")
        if center_gap is not None and (center_gap not in STANDING_CENTER_GAPS_M or result["compact_standing_detour"]):
            raise RuntimeError("Invalid standing centre gap profile in the startup session")
        result["planned_standing_center_gap_m"] = center_gap
        post_pass_cruise = session.get("standing_post_pass_cruise") is True
        if post_pass_cruise and (center_gap != 1.65 or "standing_detour_speed_cap_mps" not in sensors.route.latest):
            raise RuntimeError("Restart from the shared launcher for the 1.65 m post-pass cruise trial")
        result["standing_post_pass_cruise"] = post_pass_cruise
        if (result["compact_standing_detour"] or center_gap is not None) and (not standing or not args.standing_detour):
            raise RuntimeError("Use --scenario standing-blocker --standing-detour for the selected standing detour profile")
        if args.standing_detour and not result["yield_before_detour"]:
            raise RuntimeError("--standing-detour requires a session started with --yield-before-detour")
        if args.standing_detour and "standing_detour_active" not in sensors.route.latest:
            raise RuntimeError("Restart from the shared launcher to build standing-detour support")
        close_pass = center_gap == STANDING_CLOSE_PASS_GAP_M
        if close_pass:
            result["phase"] = "wheel_odometry_preflight"
            result["standing_close_pass"] = {
                "reference_outline_gap_m": .30,
                "minimum_sampled_outline_gap_m": None,
                "valid_outline_samples": 0,
                "detour_monitor_counts": Counter(),
                "route_revalidations": 0,
                "replan_request_samples": 0,
                "replans_observed": 0,
                "resumes_after_short_track_gap": 0,
                "standing_history_preserved_samples": 0,
                "standing_route_resumes_on_two_scans": 0,
                "checked_standing_route_samples": 0,
                "standing_route_check_status_counts": Counter(),
                "fresh_wheel_odometry_samples": 0,
                "matched_cluster_cleared_raw_stop_samples": 0,
                "stationary_geometry_cleared_fusion_conflict_samples": 0,
            }
            def wheel_odometry_ready():
                sample = state["wheel_odometry"]
                sim, wall = now(), time.monotonic()
                speed = fresh_wheel_speed(sample, sim, wall)
                if speed is None:
                    return False
                result["first_fresh_wheel_odometry"] = dict(sim=sim, speed_mps=speed)
                return True
            spin_wait(wheel_odometry_ready, 6., "fresh wheel odometry on /odom; no waypoint sent")
            print("DA NHAN VAN TOC BANH XE TU /odom.", flush=True)
        if standing and config.get("goal_map") != [STANDING_GOAL_X, 0.]:
            raise RuntimeError("Restart from the shared launcher for the standing-blocker goal with rejoin space")
        if args.scenario == "head-on-narrow":
            result["detour_wall_margin_m"] = session.get("detour_wall_margin_m")
            if any(value != NARROW_DETOUR_WALL_MARGIN_M for value in
                   (result["detour_wall_margin_m"], config.get("detour_wall_margin_m"))):
                raise RuntimeError("Restart head-on-narrow from the shared launcher to apply the detour wall margin")
        (output/"startup_session.json").write_text(json.dumps(session, indent=2), encoding="utf-8")
        (output/"fixture_config.json").write_text(json.dumps(config, indent=2), encoding="utf-8")
        if (sensors.route.latest.get("waypoints_map") or state["arrived"] or
                any(name in state["truth"] for name in (ACTOR, PROXY))):
            raise RuntimeError("Use a fresh fusion startup session; do not repeat a trial in the same world")
        if any(abs(v) > .001 for v in state["command"]) or names("/vision/tracks"):
            raise RuntimeError("Robot must be waiting, with no other Vision publisher")
        vision = Path.home()/"carrierbot-vm.q09FUG/Robot_Vision_Training"
        vision_args = prepare_vision(vision, tools, output, args.velocity_profile, args.vision_threads,
                                     reliable_rgbd=args.reliable_rgbd)
        if args.reliable_rgbd:
            import yaml
            camera = yaml.safe_load((output/"vision_config.yaml").read_text(encoding="utf-8"))["input"]
            camera_topics = [camera[key] for key in ("rgb_topic", "depth_topic", "camera_info_topic")]

            def reliable_publishers_ready():
                policies = {}
                for topic in camera_topics:
                    peers = node.get_publishers_info_by_topic(topic)
                    policies[topic] = [ReliabilityPolicy(peer.qos_profile.reliability).name for peer in peers]
                result["camera_publisher_qos"] = policies
                return all(values == [ReliabilityPolicy.RELIABLE.name] for values in policies.values())

            spin_wait(reliable_publishers_ready, 10., "one RELIABLE publisher on each camera topic")
            print("CAMERA INPUT: RELIABLE RGB/DEPTH. Ghep cap theo dau thoi gian trung nhau.", flush=True)
        launch("vision", vision_args, vision)
        result.update(vision_used=args.mode != "lidar-only", vision_process_running=True,
                      vision_used_for_decision=args.mode != "lidar-only", phase="vision_warmup")
        print("DANG KHOI DONG VISION CPU. Xe cho; chua gui dich.", flush=True)
        spin_wait(lambda: sensors.packets >= 3 and len(names("/vision/tracks")) == 1,
                  70., "Vision warm-up packets")
        with sensors.input_lock:
            runtime_cpu = dict(sensors.packet.get("runtime_cpu") or {})
        result["runtime_cpu_measured"] = runtime_cpu
        if runtime_cpu.get("torch_threads_actual") != args.vision_threads:
            raise RuntimeError("Measured Torch threads differ from requested choice: "+str(runtime_cpu))
        print("CPU THUC TE: %d luong Torch | profile %s" %
              (runtime_cpu["torch_threads_actual"], runtime_cpu.get("velocity_profile_requested")), flush=True)
        result["phase"] = "sensor_time_preflight"
        time_window = SensorTimeWindow()
        time_check = {"passed": False, "failures": 0, "latest_error": None, "next_check_wall": 0.}
        result["sensor_time_preflight"] = time_check
        def sensor_time_ready():
            if time.monotonic() < time_check["next_check_wall"]:
                return False
            time_check["next_check_wall"] = time.monotonic()+.1
            try:
                sensors.robot("map")
                robot = sensors.robot()
                start_xy = [robot["x"], robot["y"]]
                # Probe availability only, before an actor or a mission exists.
                # This provisional geometry is never sent to Fablab or the policy.
                end_xy = [robot["x"]+3*math.cos(robot["yaw"]), robot["y"]+3*math.sin(robot["yaw"])]
                scan = sensors.raw_corridor({"route_corridor_xy_m": [start_xy, end_xy]}, robot)
                if scan.get("valid") is not True:
                    raise ValueError("INVALID_RAW_SCAN_GEOMETRY")
                time_check["selected_scan"] = scan
                ready = time_window.observe(now(), scan["stamp"])
                time_check["passed"] = ready
                return ready
            except (ValueError, TransformException) as error:
                time_window.reset()
                time_check.update(failures=time_check["failures"]+1, latest_error=str(error))
                return False
        print("DANG KIEM TRA DONG BO SCAN + TF. Xe cho; chua them nguoi hay gui dich.", flush=True)
        spin_wait(sensor_time_ready, 15., "scan/TF preflight; see sensor_time_preflight in summary.json")
        print("DAT DONG BO SCAN + TF: co nhieu scan moi, giu nguyen nguong tuoi du lieu.", flush=True)
        for key, expected in (("proxy_sdf", "model"), ("actor_sdf", "actor")):
            import xml.etree.ElementTree as ET
            path = Path(config[key])
            if path.parent != Path(config["actor_sdf"]).parent or ET.parse(str(path)).find(expected) is None:
                raise RuntimeError("Invalid fixture asset")
            result["phase"] = "spawn_"+expected
            name = PROXY if expected == "model" else ACTOR
            print("DANG THEM %s: gui noi dung SDF truc tiep." % name, flush=True)
            process = launch("spawn_"+expected, factory_arguments(path))
            # Do not resend an actor request: insertion can succeed after a delayed CLI response.
            deadline = time.monotonic()+12.
            while time.monotonic() < deadline and (process.poll() is None or name not in state["truth"]):
                pump(.05)
                send()
            logs[-1].flush()
            report = insertion_report(name, process.poll(), state["truth"].keys(),
                time.monotonic()-state["truth_wall"],
                (output/("spawn_"+expected+".log")).read_text(encoding="utf-8", errors="replace")[-3000:])
            result.setdefault("factory_insertions", []).append(report)
            (output/("spawn_"+expected+"_status.json")).write_text(json.dumps(report, indent=2), encoding="utf-8")
            if not report["confirmed"]:
                raise RuntimeError("Factory insertion not confirmed for %s (exit=%s, present=%s); "
                                   "see spawn_%s_status.json. Restart, do not resend." %
                                   (name, report["process_returncode"], report["observed_in_model_states"], expected))
            children.remove(("spawn_"+expected, process))
            print("DA THEM VA XAC NHAN: "+name, flush=True)
        launch("pair_fixture", ["/usr/bin/python3", str(tools/"pair_person_lidar.py")])
        result["phase"] = "pair_warmup"
        fusion_ready_images = set()
        hidden_frames = set()
        warmup_print_wall = [0.]
        def paired_ready():
            fixture = state["fixture"]
            fixture_ready = (fixture is not None and fixture.get("ready") is True
                             and time.monotonic()-state["fixture_wall"] <= .6)
            if occluded:
                with sensors.input_lock:
                    packet, received = sensors.packet, sensors.packet_wall
                stamp = packet.get("timestamp") if packet else None
                fresh_frame = (finite(stamp) and crossing_timing.epoch is not None
                               and stamp >= crossing_timing.epoch and -.001 <= now()-stamp <= .5
                               and time.monotonic()-received <= 1.)
                if fresh_frame and packet.get("tracks") == []:
                    hidden_frames.add(stamp)
                elif fresh_frame:
                    hidden_frames.clear()
                ready = fixture_ready and fresh_frame and len(hidden_frames) >= 2
                result["occlusion"]["fresh_empty_frames_before_goal"] = len(hidden_frames)
                return ready
            sensors.observations()
            if time.monotonic()-warmup_print_wall[0] >= 3.:
                warmup_print_wall[0] = time.monotonic()
                print("DAU VAO: %d vi tri khop | %d van toc hop le | %d anh Huan fusion hop le" %
                      (len(sensors.matches), len(sensors.velocities), len(fusion_ready_images)), flush=True)
            basic = fixture_ready and len(sensors.matches) >= 3
            if args.mode != "huan" or not basic:
                return basic
            try:
                from huan_fusion_vendor.base_handoff import still_plan
                robot = sensors.robot()
                x, y = robot["x"], robot["y"]
                raw = sensors.raw_corridor({"route_corridor_xy_m": [[x, y], [x+3., y]]}, robot)
                if raw.get("valid") is not True:
                    raise ValueError("INVALID_RAW_SCAN_GEOMETRY: "+str(raw.get("reason")))
                plan = still_plan([x, y, robot["yaw"]], now())
                fused = sensors.fused(plan, robot)
                ready_now = fused["assessment"]["valid"] and "vision_rgbd_tf" in fused["sources"]
                if ready_now:
                    fusion_ready_images.add(fused["camera_image_stamp"])
                result["fusion_preflight"] = dict(valid_camera_images=len(fusion_ready_images),
                    last_status=fused["status"], diagnostic=fused["diagnostic"],
                    valid_velocity_tracks=len(sensors.velocities), sources=fused["sources"],
                    camera_image_stamp=fused["camera_image_stamp"], camera_age_sim_s=fused["camera_age_sim_s"],
                    scan_stamp=fused["scan_stamp"])
                return ready_now and len(fusion_ready_images) >= 2
            except (ValueError, TransformException) as exception:
                result["fusion_preflight"] = dict(valid_camera_images=len(fusion_ready_images), error=str(exception))
                return False
        if occluded:
            print("DANG XAC NHAN NGUOI SAU VACH VA 2 GOI CAMERA MOI CHUA CO TRACK.", flush=True)
        else:
            print("DANG DOI 3 VI TRI KHOP" + (" VA 2 ANH CO VAN TOC HOP LE CHO HUAN FUSION." if args.mode == "huan" else "."), flush=True)
        spin_wait(paired_ready, 35., "paired positions / Huan input readiness; inspect fusion_preflight")
        missed_hold = (not crossing_timing.at_start(state["truth"][ACTOR]) if head_on or occluded or standing
                       else state["truth"][ACTOR][1] > -2.0)
        if missed_hold:
            raise RuntimeError("Warm-up missed the initial fixture hold; no mission was sent")
        if not graph():
            raise RuntimeError("Command routing changed during warm-up")
        if args.input_only:
            result.update(passed=True, phase="input_ready", input_readiness_passed=True)
            print("DAT KHOI TAO DAU VAO. Chua danh gia do chinh xac van toc hay dieu huong.", flush=True)
        else:
            result["phase"] = "scheduled_goal"
            spin_wait(lambda: crossing_timing.goal_target is not None, 4., "initial actor pose and clock")
            target = crossing_timing.goal_target
            goal = config["goal_map"]
            message = PoseStamped()
            message.header.frame_id = "map"
            message.pose.orientation.w = 1.
            message.pose.position.x, message.pose.position.y = goal
            goal_preflight = dict(target_sim_s=target, latest_allowed_sim_s=target+GOAL_LATE_LIMIT)
            result["goal_slot_preflight"] = goal_preflight
            print("DANG CHO MOC GUI DICH DONG BO: %.3f s mo phong; tiep tuc cap nhat cam bien." % target,
                  flush=True)
            spin_wait(lambda: synchronized_goal_ready(target, clock_feed.snapshot, paired_ready, graph,
                                                       goal_preflight),
                      35., "synchronized simulation goal slot", poll_seconds=.02)
            start = sensors.robot("map")
            initial_truth = list(state["truth"]["carrierbot"])
            if math.hypot(start["x"], start["y"]) > .20:
                raise RuntimeError("The trial expects a newly spawned robot at the origin")
            goal_stamp_ns, _, fresh = clock_feed.snapshot()
            if not fresh:
                raise RuntimeError("Simulation clock unavailable at the goal slot")
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(goal_stamp_ns, 10**9)
            crossing_timing.record_goal(goal_stamp_ns*1e-9)
            goals.publish(message)
            result.update(goal_sent_count=1, goal_map=goal, start_pose_map=start)
            spin_wait(lambda: sensors.route.message_ready() and sensors.route.latest.get("active") is True
                      and bool(sensors.route.latest.get("waypoints_map")), 10., "one waypoint acknowledgment")
            print("DA GUI 1 DICH (%g, %g). Bat dau quyet dinh bang cam bien." % tuple(goal), flush=True)
            setup_matches = set(sensors.matches)
            policy = IntegratedFusionPolicy(args.mode, resume_fresh_scans=args.resume_profile == "fresh-scans",
                                            nearby_slow=args.nearby_slow,
                                            reduced_standing_laser_reserve=args.reduced_standing_laser_reserve,
                                            crawl_on_predicted_conflict=args.crawl_on_predicted_conflict)
            blocker_tracker = StationaryBlocker() if args.standing_detour else None
            close_monitor = ClosePassMonitor() if close_pass else None
            pending_replan = None
            fusion_start = sensors.huan.summary() if sensors.huan is not None else {}
            result["phase"] = "mission"
            result["navigation_evaluated"] = True
            started_wall, started_sim = time.monotonic(), now()
            last_control = last_graph = last_print = 0.
            previous_time, gap_since, last_reason = now(), None, None
            input_errors = Counter()
            max_sim, max_wall = (210., 720.) if standing else (150., 600.)
            while time.monotonic()-started_wall < max_wall:
                pump(.03)
                wall, sim = time.monotonic(), now()
                if sim < previous_time-.001 or sim-previous_time > 2.:
                    raise RuntimeError("Simulation clock jumped")
                previous_time = sim
                if sim-started_sim > max_sim:
                    raise RuntimeError("Trial exceeded %g simulation seconds" % max_sim)
                for label, process in children:
                    if process.poll() is not None:
                        raise RuntimeError(label+" stopped; see its log")
                if wall-last_graph >= .5:
                    last_graph = wall
                    if not graph() or len(names("/vision/tracks")) != 1:
                        raise RuntimeError("Sensor or actuator routing changed")
                if wall-last_control < .10:
                    continue
                last_control = wall
                snapshot = sensors.route.latest
                if (wall-state["command_wall"] > .8 or wall-state["truth_wall"] > .8 or
                        "carrierbot" not in state["truth"] or ACTOR not in state["truth"]):
                    raise RuntimeError("Command output or scoring measurements lost")
                if not isinstance(snapshot, dict) or snapshot.get("behavior_gate", {}).get("enabled") is not True:
                    raise RuntimeError("Behavior gate unavailable")
                if snapshot.get("early_rejoin_used") is True and not result["early_rejoin_observed"]:
                    result.update(early_rejoin_observed=True,
                                  early_rejoin_pose_map=snapshot.get("robot_pose_map"),
                                  early_rejoin_elapsed_sim_s=sim-started_sim)
                    print("DA NHAP LAI TUYEN SOM. Tiep tuc bam dich cu.", flush=True)
                points = snapshot.get("waypoints_map", [])
                if not points or math.dist(points[-1], goal) > .001:
                    raise RuntimeError("Original mission goal lost or replaced")
                if pending_replan is not None and snapshot["route_revision"] > pending_replan[0]:
                    if (snapshot["route_revision"] == pending_replan[0]+1
                            and snapshot["standing_detour_active"] is True and points != pending_replan[1]):
                        result["standing_close_pass"]["replans_observed"] += 1
                    pending_replan = None
                current, error, fusion = None, None, None
                intent, raw, observations, input_error, input_stage = None, {"valid": False}, [], None, "clock"
                try:
                    clock_feed.rebind_if_stale()
                    if not clock_feed.snapshot()[2]:
                        raise ValueError("CLOCK_RECEIPT_STALE")
                    input_stage = "map_pose"
                    # Transient missing/current-time TF must hold WAIT, never escape
                    # the input-gap handler before the scan/TF recovery can run.
                    current = sensors.robot("map")
                    error = math.hypot(current["x"]-goal[0], current["y"]-goal[1])
                    if error <= .5:
                        result.setdefault("last_half_metre_entry_elapsed_sim_s", sim-started_sim)
                    if state["arrived"] and snapshot.get("finished") is True and error <= .18:
                        result.update(arrival=True, arrival_sim_s=sim, goal_error_m=error, final_pose_map=current)
                        result["last_half_metre_elapsed_sim_s"] = (
                            sim-started_sim-result["last_half_metre_entry_elapsed_sim_s"])
                        break
                    input_stage = "route_intent"
                    intent, robot = sensors.intent()
                    input_stage = "scan_tf"
                    raw = sensors.raw_corridor(intent, robot)
                    if raw.get("valid") is not True:
                        raise ValueError("INVALID_RAW_SCAN_GEOMETRY: "+str(raw.get("reason")))
                    input_stage = "vision_stream"
                    observations = sensors.observations()
                    if wall-sensors.packet_wall > 3.:
                        raise ValueError("VISION_STREAM_GAP")
                    input_stage = "fixture"
                    if wall-state["fixture_wall"] > .6 or not state["fixture"].get("ready"):
                        raise ValueError("SCORING_FIXTURE_STREAM_GAP")
                    input_stage = "fusion"
                    fusion = sensors.fused(intent, robot)
                    gap_since = None
                except (ValueError, TransformException) as exception:
                    input_error = str(exception)
                    input_errors[input_stage+":"+type(exception).__name__] += 1
                    result["input_errors"] = dict(input_errors)
                    raw = {"valid": False, "reason": input_error}
                    if gap_since is None:
                        gap_since = wall
                odom = state["wheel_odometry"]
                decision_sim, decision_wall = now(), time.monotonic()
                transient_gap = ((input_stage == "route_intent" and input_error in
                                  ("ROUTE_STREAM_NOT_READY", "ROUTE_OR_SOURCE_POSE_NOT_READY"))
                                 or (input_stage == "scan_tf" and input_error is not None
                                     and input_error.startswith("NO_FRESH_TIME_MATCHED_SCAN:")))
                measured_speed = fresh_wheel_speed(odom, decision_sim, decision_wall)
                decision = policy.evaluate(decision_sim, intent, raw, observations, fusion,
                                           transient_gap=transient_gap, measured_speed_mps=measured_speed,
                                           close_monitor=close_monitor,
                                           match_blocking_returns=sensors.matched_blocking_returns)
                risk = decision.get("fusion_assessment", {})
                crawl = decision.get("crawl_forecast")
                baseline_risk = crawl["nominal_baseline_risk"] if crawl else risk.get("baseline_risk")
                nominal_risk = crawl["nominal_risk"] if crawl else risk.get("risk")
                if baseline_risk is not None:
                    evidence = result["standing_laser_reserve"]
                    evidence["applied_samples"] += 1
                    if baseline_risk == "POTENTIAL_COLLISION" and nominal_risk == "CLEAR_WITHIN_MODEL":
                        evidence["original_conflict_cleared_samples"] += 1
                decision = identify_input_wait(decision, input_stage, input_error)
                blocker = None
                if blocker_tracker is not None:
                    blocker = blocker_tracker.observe(decision_sim, intent, decision, observations)
                    if decision.get("standing_approach_reserved"):
                        result.setdefault("first_standing_approach_hold_sim_s", decision_sim)
                        policy.base.reset_clearance()
                    if blocker is not None:
                        if close_monitor is not None:
                            close_monitor.arm(blocker)
                        result.setdefault("first_standing_detour_request_sim_s", decision_sim)
                        result.setdefault("first_standing_detour_request_center_distance_m",
                                          blocker["center_distance_m"])
                        decision.update(action="WAIT", action_code=0, speed_cap_mps=0.,
                                        reason="PLANNING_AROUND_STANDING_PERSON",
                                        standing_detour_request=blocker)
                    decision_route = intent["route_snapshot"] if intent is not None else snapshot
                    detour_active = (snapshot["standing_detour_active"] is True
                                     or decision_route.get("standing_detour_active") is True)
                    if detour_active:
                        result.setdefault("first_standing_detour_seen_sim_s", decision_sim)
                    if close_monitor is not None and (detour_active or close_monitor.motion_check_pending):
                        policy.check_close_pass(close_monitor, decision_sim, decision, fusion, intent)
                        evidence = result["standing_close_pass"]
                        evidence["detour_monitor_counts"][decision["close_pass_monitor"]] += 1
                        evidence["route_revalidations"] += int(decision.get("close_pass_route_revalidated") is True)
                        evidence["resumes_after_short_track_gap"] += int("close_pass_resume" in decision)
                        evidence["standing_history_preserved_samples"] += int("standing_history_preserved" in decision)
                        evidence["standing_route_resumes_on_two_scans"] += int("standing_route_resume_scans" in decision)
                        evidence["fresh_wheel_odometry_samples"] += int(measured_speed is not None)
                        status = decision.get("standing_route_check_status")
                        if status is not None:
                            evidence["standing_route_check_status_counts"][status] += 1
                        checked = decision.get("standing_route_check")
                        if checked is not None:
                            evidence["checked_standing_route_samples"] += 1
                            evidence["matched_cluster_cleared_raw_stop_samples"] += int("matched_blocking_returns" in checked)
                            evidence["stationary_geometry_cleared_fusion_conflict_samples"] += int(
                                checked.get("original_fusion_risk") == "POTENTIAL_COLLISION" and
                                decision["fusion_assessment"]["risk"] == "CLEAR_WITHIN_MODEL")
                        replan = decision.get("close_pass_replan_request")
                        if replan is not None:
                            pending_replan = (decision["route_revision"], decision_route["waypoints_map"])
                            evidence["replan_request_samples"] += 1
                decision = stabilize_standing_detour_release(decision, intent)
                if post_pass_cruise and decision["action"] == "BASELINE" and intent is not None:
                    standing_cap = intent["standing_detour_speed_cap_mps"]
                    if standing_cap > 0.:
                        # Fablab can accelerate only after this speed has been evaluated and sent back.
                        decision["speed_cap_mps"] = min(decision["speed_cap_mps"], standing_cap)
                if post_pass_cruise and snapshot.get("standing_detour_speed_cap_mps") == .08:
                    guard_speed = state["command"][0]
                    result["post_pass_cruise_max_guard_command_mps"] = max(
                        result.get("post_pass_cruise_max_guard_command_mps", 0.), guard_speed)
                    if guard_speed > .061:
                        result.setdefault("first_post_pass_cruise_sim_s", decision_sim)
                if crawl:
                    evidence = result["crawl_forecast"]
                    evidence["checked_samples"] += 1
                    evidence["clear_samples"] += int(crawl["selected"])
                    evidence["slow_directive_samples"] += int(decision["reason"] == "HUAN_FUSION_CLEAR_AT_CRAWL")
                result["sensor_decision_evaluated"] = True
                result["actuation"] = result["actuation"] or any(abs(v) > .001 for v in state["command"])
                planning_request = blocker or decision.get("close_pass_replan_request")
                send(decision["action_code"], decision["speed_cap_mps"], decision["route_revision"],
                     planning_request["center_map"] if planning_request is not None else None)
                score.observe(decision_sim, decision, snapshot, state["command"], state["truth"]["carrierbot"], state["truth"][ACTOR])
                row = {"sim": decision_sim, "wall": decision_wall, "tick_started_wall": wall,
                       "tick_compute_wall_s": decision_wall-wall, "decision": decision, "fusion": fusion,
                       "clock_age_wall_s": clock_feed.snapshot()[1],
                       "route": snapshot, "guard_command": state["command"],
                       "matches": [{"position_ok": v["position_ok"], "velocity_ok": v["velocity_ok"],
                                    "match": v["match"], "id": v["track"].get("id")} for v in observations],
                       "input_error": input_error, "input_stage": input_stage if input_error else None,
                       "scoring_only": {"robot_truth": state["truth"]["carrierbot"],
                                                                    "person_truth": state["truth"][ACTOR]}}
                if close_pass:
                    row["wheel_odometry"] = {"speed_mps": measured_speed,
                                             "age_sim_s": decision_sim-odom[0] if odom is not None else None,
                                             "age_wall_s": decision_wall-odom[1] if odom is not None else None}
                trace.write(json.dumps(row)+"\n")
                trace.flush()
                if occluded:
                    observe_occlusion(result["occlusion"], row, initial_truth,
                                      crossing_timing.motion, crossing_timing.exit)
                if standing:
                    observe_standing_blocker(result["standing_blocker"], row, initial_truth,
                                             crossing_timing.motion, crossing_timing.center)
                if close_pass:
                    pose = snapshot.get("robot_pose_map") or []
                    if snapshot.get("pose_tf_valid") is True and len(pose) == 3 and finite(pose[2]):
                        # Simulator positions and the URDF outline are used only for scoring.
                        _, outline_gap = outline_gaps(row["scoring_only"]["robot_truth"], pose[2],
                                                      row["scoring_only"]["person_truth"], PERSON_PROXY_RADIUS_M)
                        evidence = result["standing_close_pass"]
                        evidence["valid_outline_samples"] += 1
                        minimum = evidence["minimum_sampled_outline_gap_m"]
                        if minimum is None or outline_gap < minimum:
                            evidence["minimum_sampled_outline_gap_m"] = outline_gap
                if (decision["reason"] != last_reason and wall-last_print >= 1.) or wall-last_print >= 10.:
                    last_reason, last_print = decision["reason"], wall
                    location = "xe (%.2f, %.2f) con %.2f m" % (current["x"], current["y"], error) if current else "dang cho TF vi tri moi"
                    detail = " | %s: %s" % (input_stage, input_error[:180]) if input_error else ""
                    print("%s | %s | %s | stereo-LiDAR %d mau%s" %
                          (decision["action"], last_reason, location, len(sensors.matches), detail), flush=True)
                if gap_since is not None and wall-gap_since > 5.:
                    raise RuntimeError("Sensor/fixture gap lasted more than 5 wall seconds: "+str(input_error))
            else:
                raise RuntimeError("Trial exceeded %g wall seconds" % max_wall)
            result["matched_positions_during_mission"] = len(sensors.matches-setup_matches)
            metrics = score.finish(initial_truth, result["matched_positions_during_mission"], result.get("arrival", False),
                                   result.get("goal_error_m", float("inf")), result["goal_sent_count"])
            extra_checks = None
            if sensors.huan is not None:
                result["huan_fusion"] = sensors.huan.summary()
                result["huan_fusion_mission"] = {k: result["huan_fusion"][k]-fusion_start.get(k, 0)
                                                for k in ("updates", "valid_risk_updates", "laser_source_updates", "valid_camera_images")}
                extra_checks = {"fusion_ready_before_goal": len(fusion_ready_images) >= 2,
                    "huan_fusion_used_during_mission": result["huan_fusion_mission"]["valid_risk_updates"] >= 3}
            if occluded:
                evidence = result["occlusion"]
                response = evidence.get("first_obstacle_response")
                extra_checks = {
                    "person_hidden_at_goal_send": evidence.get("fresh_empty_frames_before_goal", 0) >= 2,
                    "robot_moved_before_actor": evidence["robot_advance_before_actor_motion_m"] >= .25,
                    "obstacle_response_before_actor_exit": response is not None and crossing_timing.exit is not None
                                                         and response["sim"] < crossing_timing.exit,
                    "forward_command_after_actor_exit": "first_forward_after_actor_exit_sim_s" in evidence,
                }
                if args.mode != "lidar-only":
                    first_stereo = evidence.get("first_stereo_position_sim_s")
                    extra_checks["stereo_position_during_encounter"] = (
                        first_stereo is not None and crossing_timing.motion is not None
                        and crossing_timing.exit is not None and crossing_timing.motion <= first_stereo < crossing_timing.exit)
                if args.mode == "huan":
                    extra_checks["huan_fusion_used_during_mission"] = result["huan_fusion_mission"]["valid_camera_images"] >= 2
            if standing:
                evidence = result["standing_blocker"]
                extra_checks = dict(extra_checks or {},
                    robot_moved_before_actor=evidence["robot_advance_before_actor_motion_m"] >= .25,
                    person_remained_in_lane=(evidence["samples_standing_in_lane"] >= 3
                                             and evidence["samples_outside_lane_after_entry"] == 0),
                    detour_observed="first_detour_sim_s" in evidence,
                    bypassed_person_while_standing="bypassed_standing_person_sim_s" in evidence)
                if args.standing_detour:
                    extra_checks["standing_detour_applied"] = "first_standing_detour_seen_sim_s" in result
                if post_pass_cruise:
                    extra_checks["post_pass_speed_above_0_06_observed"] = "first_post_pass_cruise_sim_s" in result
                if close_pass:
                    minimum = result["standing_close_pass"]["minimum_sampled_outline_gap_m"]
                    extra_checks["sampled_outline_clearance_at_least_0_30_m"] = minimum is not None and minimum >= .30
            if head_on or occluded or standing:
                result.update(encounter_score(metrics, extra_checks, scenario=args.scenario))
                if standing:
                    result["score_contract"] = "Arrival and clearance require an observed bypass while the person remains in the lane."
            else:
                result.update(comparison_score(metrics, args.mode, extra_checks,
                                               early_rejoin=result["early_rejoin_enabled"],
                                               early_rejoin_observed=result["early_rejoin_observed"]))
    except KeyboardInterrupt:
        result["error"] = "Interrupted by user"
    except Exception as error:
        result["error"] = str(error)
        print("STOP: "+str(error), flush=True)
    finally:
        for _ in range(3):
            try:
                send()
                pump(.05)
            except Exception:
                pass
        # Stop only owned perception/fixture processes. Startup and Gazebo stay open.
        for _, process in reversed(children):
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        deadline = time.monotonic()+5.
        while any(p.poll() is None for _, p in children) and time.monotonic() < deadline:
            pump(.05)
        for _, process in reversed(children):
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        result.update(packets=sensors.packets, valid_position_tracks=len(sensors.positions),
                      valid_velocity_tracks=len(sensors.velocities), matched_positions=len(sensors.matches))
        if sensors.huan is not None:
            result["huan_fusion"] = sensors.huan.summary()
        if started_wall is not None:
            result.update(elapsed_wall_s=time.monotonic()-started_wall, elapsed_sim_s=now()-started_sim)
        if math.isfinite(score.minimum_clearance):
            result["observed_minimum_clearance_m"] = score.minimum_clearance
            result["minimum_center_distance_m"] = score.minimum_clearance + ROBOT_SCORING_RADIUS_M + PERSON_PROXY_RADIUS_M
        executor.shutdown(timeout_sec=2.)
        callback_thread.join(timeout=2.)
        if callback_thread.is_alive():
            result.update(passed=False, error="ROS input executor did not stop cleanly")
        for handle in logs+[tracks, trace]:
            handle.close()
        if not args.input_only:
            timing_key = "head_on_timing" if head_on else "crossing_timing"
            timing_check = "synchronized_head_on" if head_on else "synchronized_crossing"
            if occluded:
                timing_key, timing_check = "occlusion_timing", "synchronized_occluded_crossing"
            elif standing:
                timing_key, timing_check = "standing_blocker_timing", "synchronized_standing_blocker"
            result[timing_key] = crossing_timing.report()
            if "checks" in result:
                result["navigation_checks_passed"] = result["passed"]
                timing_ok = result[timing_key]["valid"]
                result["checks"][timing_check] = timing_ok
                result["applicable_checks"][timing_check] = timing_ok
                result["passed"] = result["passed"] and timing_ok
        (output/"summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        try:
            from analyze_huan_run import analyze
            (output/"performance_review.json").write_text(json.dumps(analyze(output), indent=2), encoding="utf-8")
        except Exception as report_error:
            print("PERFORMANCE REPORT UNAVAILABLE: " + str(report_error), flush=True)
        try:
            from diagnose_huan_input import diagnose
            diagnosis = diagnose(output)
            (output/"input_diagnosis.json").write_text(json.dumps(diagnosis, indent=2), encoding="utf-8")
            print("VELOCITY STATUS: "+json.dumps(diagnosis["velocity_status_counts"]), flush=True)
        except Exception as report_error:
            print("INPUT DIAGNOSIS UNAVAILABLE: "+str(report_error), flush=True)
        outcome = "CHUA DAT: xem summary.json va trace.jsonl."
        if result["passed"]:
            objective = "input_readiness" if args.input_only else result.get("score_objective")
            outcome = {"input_readiness": "DAT KHOI TAO DAU VAO.",
                       "head_on_wide": "DAT CA NGUOI DI NGUOC CHIEU TRONG PHONG RONG.",
                       "head_on_narrow": "DAT TIEU CHI CA NGUOI DI NGUOC CHIEU TRONG HANH LANG HEP.",
                       "occluded_crossing": "DAT TIEU CHI CA NGUOI BUOC RA TU SAU VACH.",
                       "standing_blocker": "DAT TIEU CHI CA NGUOI DUNG CHAN GIUA DUONG.",
                       "early_rejoin": "DAT CA CAT NGANG DONG BO VA NHAP LAI TUYEN."}.get(objective, "DAT CA NGUOI CAT NGANG.")
        print(outcome+"\nKET QUA: "+str(output/"summary.json"), flush=True)
        print(json.dumps(result, indent=2), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
