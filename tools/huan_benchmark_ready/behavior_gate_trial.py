#!/usr/bin/env python3
"""Scripted Gazebo wiring test. This does not evaluate sensor-driven avoidance."""
import json
import math
from pathlib import Path
import tempfile
import time

PHASES = [("INITIAL_WAIT", 1.0, 0), ("BASELINE", 3.0, 2), ("SLOW", 3.0, 1),
          ("WAIT", 2.0, 0), ("RESUME", 2.0, 2), ("DROP_HEARTBEAT", 2.0, None),
          ("FINISH", None, 2)]


class TrialSequence:
    def __init__(self, now, wall):
        self.index = 0
        self.since, self.wall_since, self.previous = now, wall, now

    def step(self, now, wall, arrived=False):
        if now < self.previous-.001 or now-self.previous > 2.0:
            raise RuntimeError("Simulation clock jumped during the motion trial")
        self.previous = now
        name, duration, action = PHASES[self.index]
        if arrived:
            if name != "FINISH":
                raise RuntimeError("Goal completed before all gate phases were tested")
            return "DONE", None, now-self.since, wall-self.wall_since
        if duration is not None and now-self.since >= duration:
            self.index += 1
            self.since, self.wall_since = now, wall
            name, duration, action = PHASES[self.index]
        return name, action, now-self.since, wall-self.wall_since


class TraceVerifier:
    def __init__(self, goal):
        self.goal = goal
        self.phases = {}

    def observe(self, row):
        phase = row["phase"]
        if row["phase_elapsed_sim_s"] < .4:
            return
        if phase == "DROP_HEARTBEAT" and row["phase_elapsed_wall_s"] < 1.0:
            return
        snapshot = row["route"]
        points = snapshot.get("waypoints_map", [])
        if not points or math.dist(points[-1], self.goal) > .001:
            raise RuntimeError("The original mission goal was lost or replaced")
        v, w = row["guard_command"]
        if not math.isfinite(v) or not math.isfinite(w):
            raise RuntimeError("Non-finite command at guard output")
        if v < -.001 or abs(w) > .301:
            raise RuntimeError("Unexpected reverse or excessive turn in this forward-only trial")
        expected = {"INITIAL_WAIT": 0, "BASELINE": 2, "SLOW": 1, "WAIT": 0,
                    "RESUME": 2, "DROP_HEARTBEAT": 0, "FINISH": 2}[phase]
        gate = snapshot.get("behavior_gate", {})
        if gate.get("action") != expected and not (phase == "FINISH" and snapshot.get("finished")):
            raise RuntimeError("Gate action mismatch in %s: %s" % (phase, gate))
        cap = .06 if phase == "SLOW" else .12
        upstream = snapshot.get("command_preview", {})
        uv, uw = upstream.get("linear_x"), upstream.get("angular_z")
        if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in (uv, uw)):
            raise RuntimeError("Missing or invalid Fablab output command")
        if abs(uv) > cap+.001:
            raise RuntimeError("Fablab output exceeds the gate cap before the guard")
        if abs(v) > cap+.001:
            raise RuntimeError("Guard output exceeds the requested speed cap in " + phase)
        if expected == 0 and (abs(v) > .001 or abs(w) > .001 or abs(uv) > .001 or abs(uw) > .001):
            raise RuntimeError("Nonzero guard output during " + phase)
        stats = self.phases.setdefault(phase, {"samples": 0, "positive_samples": 0,
                    "max_guard_linear_mps": 0., "max_guard_angular_rps": 0., "max_fablab_linear_mps": 0.,
                    "first_truth_xy": row["truth_xy"], "last_truth_xy": row["truth_xy"],
                    "max_truth_displacement_m": 0., "route_revision": snapshot.get("route_revision"),
                    "segment": snapshot.get("current_segment"),
                    "waypoints_map": [list(point) for point in points]})
        stats["samples"] += 1
        stats["positive_samples"] += int(abs(v) > .01)
        stats["max_guard_linear_mps"] = max(stats["max_guard_linear_mps"], abs(v))
        stats["max_guard_angular_rps"] = max(stats["max_guard_angular_rps"], abs(w))
        stats["max_fablab_linear_mps"] = max(stats["max_fablab_linear_mps"], abs(uv))
        stats["last_truth_xy"] = row["truth_xy"]
        stats["max_truth_displacement_m"] = max(stats["max_truth_displacement_m"],
                                                math.dist(stats["first_truth_xy"], row["truth_xy"]))
        if expected == 0:
            if stats["max_truth_displacement_m"] > .03:
                raise RuntimeError("Robot moved more than 3 cm after the stop settling interval")
            if (snapshot.get("route_revision") != stats["route_revision"] or
                    snapshot.get("current_segment") != stats["segment"] or
                    points != stats["waypoints_map"]):
                raise RuntimeError("Fablab changed the route while waiting")
        if phase == "DROP_HEARTBEAT" and gate.get("reason") != "directive_timeout":
            raise RuntimeError("Heartbeat-loss stop was not attributed to the gate timeout")

    def finish(self):
        checks = {}
        for phase, _, action in PHASES:
            stats = self.phases.get(phase, {})
            checks[phase+"_observed"] = stats.get("samples", 0) >= 3
            if action in (1, 2):
                checks[phase+"_moved"] = stats.get("positive_samples", 0) >= 3 and stats.get("max_truth_displacement_m", 0) >= .03
        return {"passed": all(checks.values()), "checks": checks, "phases": self.phases}


def main():
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from gazebo_msgs.msg import ModelStates
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, Float64MultiArray, String
    from tf2_ros import Buffer, TransformException, TransformListener

    rclpy.init()
    node = rclpy.create_node("behavior_gate_trial")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    directives = node.create_publisher(Float64MultiArray, "/dynamic_behavior/directive", 1)
    goals = node.create_publisher(PoseStamped, "/behavior_gate/waypoints", 1)
    state = {"scan_wall": 0., "route_wall": 0., "wheel_wall": 0., "truth_wall": 0.,
             "route": None, "guard_command": [0., 0.], "truth_xy": None,
             "wheel_samples": 0, "arrived": False}
    def scan(message):
        state["scan_wall"] = time.monotonic()
    def route(message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict) or value.get("source") != "fablab_internal_waypoints_v1":
                raise ValueError("Invalid route source")
            state.update(route=value, route_wall=time.monotonic())
        except ValueError:
            state.update(route=None)
    def wheel(message):
        values = (message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z)
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError("Non-finite wheel input")
        state.update(guard_command=[message.linear.x, message.angular.z],
                     wheel_wall=time.monotonic(), wheel_samples=state["wheel_samples"]+1)
    def truth(message):
        if "carrierbot" in message.name:
            p = message.pose[message.name.index("carrierbot")].position
            if not all(math.isfinite(value) for value in (p.x, p.y)):
                raise RuntimeError("Non-finite Gazebo robot position")
            state.update(truth_xy=[p.x, p.y], truth_wall=time.monotonic())
    subscriptions = [
        node.create_subscription(LaserScan, "/scan", scan, qos_profile_sensor_data),
        node.create_subscription(String, "/behavior_gate/route_state", route, 1),
        node.create_subscription(Twist, "/carrierbot_controller/cmd_vel_unstamped", wheel, 1),
        node.create_subscription(ModelStates, "/gazebo/model_states", truth, qos_profile_sensor_data),
        node.create_subscription(Bool, "/behavior_gate/arrival", lambda m: state.update(arrived=bool(m.data)), 1),
    ]
    def sources(topic, subscribers=False):
        info = node.get_subscriptions_info_by_topic(topic) if subscribers else node.get_publishers_info_by_topic(topic)
        return [row.node_name for row in info]
    def route_ok():
        return (sources("/safety/input_cmd_vel") == ["fablab_behavior_guidance"] and
                sources("/safety/input_cmd_vel", True) == ["carrierbot_safety_guard"] and
                sources("/carrierbot_controller/cmd_vel_unstamped") == ["carrierbot_safety_guard"] and
                sources("/carrierbot_controller/cmd_vel_unstamped", True).count("carrierbot_controller") == 1 and
                sources("/dynamic_behavior/directive") == [node.get_name()] and
                sources("/dynamic_behavior/directive", True) == ["fablab_behavior_guidance"] and
                sources("/behavior_gate/route_state") == ["fablab_behavior_guidance"] and
                sources("/behavior_gate/arrival") == ["fablab_behavior_guidance"] and
                sources("/behavior_gate/waypoints") == [node.get_name()] and
                sources("/behavior_gate/waypoints", True) == ["fablab_behavior_guidance"])
    def pose():
        transform = buffer.lookup_transform("map", "base_footprint", Time())
        p, q = transform.transform.translation, transform.transform.rotation
        if not all(math.isfinite(value) for value in (p.x, p.y, q.x, q.y, q.z, q.w)):
            raise RuntimeError("Non-finite robot TF")
        stamp = transform.header.stamp.sec+transform.header.stamp.nanosec/1e9
        now = node.get_clock().now().nanoseconds/1e9
        if not -.02 <= now-stamp <= .3:
            raise RuntimeError("Current robot TF is stale")
        return [p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]
    def ready():
        wall = time.monotonic()
        snapshot = state["route"]
        now = node.get_clock().now().nanoseconds/1e9
        return (route_ok() and all(wall-state[key] <= .8 for key in
                ("scan_wall", "wheel_wall", "route_wall", "truth_wall")) and
                snapshot is not None and snapshot.get("pose_tf_valid") is True and
                isinstance(snapshot.get("stamp"), (int, float)) and -.02 <= now-snapshot["stamp"] <= .5 and
                snapshot.get("behavior_gate", {}).get("enabled") is True)

    result = {"stage": "behavior_gate_wiring_motion", "passed": False, "actuation": True,
              "environment": "Gazebo simulation", "sensor_decision_evaluated": False,
              "vision_used": False, "obstacle_avoidance_evaluated": False, "goal_sent_count": 0}
    root = Path.home()/"carrierbot_test_runs"
    root.mkdir(exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix=time.strftime("behavior_gate_%Y%m%d_%H%M%S_"), dir=str(root)))
    trace = (output/"trace.jsonl").open("w", encoding="utf-8")
    print("THU GIAM TOC/CHO CO CHUYEN DONG. Khong dung Vision.\nLOG: " + str(output), flush=True)
    sequence_number = 0
    def send(action):
        nonlocal sequence_number
        if action is None:
            return
        snapshot = state["route"]
        if snapshot is None:
            return
        sequence_number += 1
        message = Float64MultiArray()
        cap = 0. if action == 0 else .06 if action == 1 else .12
        message.data = [1., node.get_clock().now().nanoseconds/1e9, float(sequence_number),
                        float(action), cap, float(snapshot["route_revision"])]
        directives.publish(message)
    verifier = None
    try:
        deadline = time.monotonic()+20.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if ready():
                try:
                    start = pose()
                    break
                except (TransformException, RuntimeError):
                    pass
        else:
            raise RuntimeError("Start start_behavior_gate.sh and wait for SAN SANG THU DIEU PHOI")
        if state["route"].get("waypoints_map") or state["route"].get("finished"):
            raise RuntimeError("Use a fresh gate session; a previous mission already exists")
        if any(abs(v) > .001 for v in state["guard_command"]):
            raise RuntimeError("Robot is not initially waiting")
        goal = [start[0]+1.2*math.cos(start[2]), start[1]+1.2*math.sin(start[2])]
        verifier = TraceVerifier(goal)
        result.update(start_pose_map=start, goal_map=goal, requested_distance_m=1.2)
        initial_truth = list(state["truth_xy"])
        send(0)
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = node.get_clock().now().to_msg()
        message.pose.orientation.w = 1.0
        message.pose.position.x, message.pose.position.y = goal
        goals.publish(message)
        result["goal_sent_count"] = 1
        started_wall = time.monotonic()
        started_sim = node.get_clock().now().nanoseconds/1e9
        controller = TrialSequence(started_sim, started_wall)
        last_send, last_phase, last_sample = 0., None, -1
        while time.monotonic()-started_wall < 360.:
            rclpy.spin_once(node, timeout_sec=.05)
            now, wall = node.get_clock().now().nanoseconds/1e9, time.monotonic()
            if now-started_sim > 60.:
                raise RuntimeError("Trial exceeded 60 simulation seconds")
            if not ready():
                raise RuntimeError("Scan, TF, truth, guard or directive routing became unavailable")
            current = pose()
            phase, action, elapsed, elapsed_wall = controller.step(now, wall, state["arrived"])
            if phase == "DONE":
                break
            if phase != last_phase:
                print("GIAI DOAN: " + phase, flush=True)
                last_phase = phase
                last_send = 0.
            if wall-last_send >= .1:
                send(action)
                last_send = wall
            if state["wheel_samples"] != last_sample:
                last_sample = state["wheel_samples"]
                row = {"phase": phase, "phase_elapsed_sim_s": elapsed, "phase_elapsed_wall_s": elapsed_wall,
                       "stamp_sim": now, "route": state["route"], "guard_command": state["guard_command"],
                       "truth_xy": state["truth_xy"], "pose_map": current}
                trace.write(json.dumps(row)+"\n")
                trace.flush()
                verifier.observe(row)
        else:
            raise RuntimeError("Trial exceeded 360 wall seconds")
        final = pose()
        error = math.dist(final[:2], goal)
        result.update(final_pose_map=final, goal_error_m=error, arrival=state["arrived"],
                      actual_displacement_m=math.dist(initial_truth, state["truth_xy"]),
                      elapsed_sim_s=node.get_clock().now().nanoseconds/1e9-started_sim,
                      elapsed_wall_s=time.monotonic()-started_wall)
        result["verification"] = verifier.finish()
        result["passed"] = (result["verification"]["passed"] and result["arrival"] and error <= .18
                            and result["actual_displacement_m"] >= .6 and result["goal_sent_count"] == 1)
    except (Exception, KeyboardInterrupt) as error:
        result["error"] = str(error) or "User stopped the trial"
        print("STOP: " + result["error"], flush=True)
    finally:
        # Stop first. If this node exits abruptly, the independent C++ watchdog also stops.
        for _ in range(3):
            try:
                send(0)
                rclpy.spin_once(node, timeout_sec=.05)
            except Exception as error:
                result["cleanup_observation_error"] = str(error)
                result["passed"] = False
        if verifier is not None and "verification" not in result:
            result["verification"] = verifier.finish()
        result["last_route_snapshot"] = state["route"]
        trace.close()
        (output/"summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(result, indent=2), flush=True)
        print("KET QUA: " + str(output/"summary.json"), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
