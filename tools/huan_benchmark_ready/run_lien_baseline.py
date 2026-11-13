#!/usr/bin/env python3
"""Exercise Lien's guidance through the guard on an empty or blocked route."""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import time

import rclpy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


def main():
    parser = argparse.ArgumentParser()
    scenarios = parser.add_mutually_exclusive_group()
    scenarios.add_argument("--static-obstacle", action="store_true",
                           help="Place a cylinder at world (3, 0), then send the goal map (5, 0)")
    scenarios.add_argument("--crossing", action="store_true",
                           help="Move a cylinder across the path after the robot starts driving")
    parser.add_argument("--reverse-crossing", action="store_true",
                        help="Reverse the crossing direction, from +Y towards -Y")
    parser.add_argument("--turn-back", action="store_true",
                        help="After the center hold, move the obstacle back to its original side")
    parser.add_argument("--check-only", action="store_true",
                        help="Print readiness details without sending any waypoint")
    args = parser.parse_args()
    if args.reverse_crossing and not args.crossing:
        parser.error("--reverse-crossing requires --crossing")
    if args.turn_back and not args.crossing:
        parser.error("--turn-back requires --crossing")
    crossing_sign = -1.0 if args.reverse_crossing else 1.0
    has_obstacle = args.static_obstacle or args.crossing
    obstacle_name = "lien_crossing_test" if args.crossing else "lien_static_test"
    rclpy.init()
    node = rclpy.create_node("lien_baseline_test")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    command_pub = node.create_publisher(Twist, "/safety/input_cmd_vel", 1)
    goal_pub = node.create_publisher(PoseStamped, "/avoidance_preview/waypoints", 1)
    state = {"enabled": False, "scan": 0.0, "raw": 0.0, "arrived": False,
             "max_wheel_speed": 0.0, "wheel": 0.0,
             "truth": 0.0, "minimum_center_distance_m": float("inf"),
             "maximum_lateral_m": 0.0, "obstacle_present": False,
             "robot_x": 0.0, "obstacle_y_min": float("inf"),
             "obstacle_y_max": -float("inf"), "obstacle_y": None}

    def scan(message):
        state["scan"] = time.monotonic()

    def raw(message):
        state["raw"] = time.monotonic()
        if state["enabled"]:
            if not all(math.isfinite(v) for v in (message.linear.x, message.angular.z)):
                raise RuntimeError("Non-finite guidance command")
            command_pub.publish(message)

    def arrival(message):
        if state["enabled"] and message.data:
            state["arrived"] = True

    def wheel(message):
        state["wheel"] = time.monotonic()
        state["max_wheel_speed"] = max(state["max_wheel_speed"], abs(message.linear.x))

    def truth(message):
        if "carrierbot" not in message.name:
            return
        state["truth"] = time.monotonic()
        robot = message.pose[message.name.index("carrierbot")].position
        state["robot_x"] = robot.x
        if state["enabled"]:
            state["maximum_lateral_m"] = max(state["maximum_lateral_m"], abs(robot.y))
        state["obstacle_present"] = obstacle_name in message.name
        if state["enabled"] and state["obstacle_present"]:
            obstacle = message.pose[message.name.index(obstacle_name)].position
            distance = math.hypot(robot.x - obstacle.x, robot.y - obstacle.y)
            state["minimum_center_distance_m"] = min(state["minimum_center_distance_m"], distance)
            state["obstacle_y"] = obstacle.y
            state["obstacle_y_min"] = min(state["obstacle_y_min"], obstacle.y)
            state["obstacle_y_max"] = max(state["obstacle_y_max"], obstacle.y)

    subscriptions = [
        node.create_subscription(LaserScan, "/scan", scan, qos_profile_sensor_data),
        node.create_subscription(Twist, "/avoidance_preview/cmd_vel", raw, 1),
        node.create_subscription(Bool, "/avoidance_preview/arrival", arrival, 1),
        node.create_subscription(Twist, "/carrierbot_controller/cmd_vel_unstamped", wheel, 1),
        # Ground truth is recorded for scoring, never forwarded to guidance.
        node.create_subscription(ModelStates, "/gazebo/model_states", truth, qos_profile_sensor_data),
    ]

    def pose():
        transform = buffer.lookup_transform("map", "base_footprint", Time())
        p, q = transform.transform.translation, transform.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return p.x, p.y, yaw

    def routing_ok():
        wheel_sources = node.get_publishers_info_by_topic("/carrierbot_controller/cmd_vel_unstamped")
        input_sources = node.get_publishers_info_by_topic("/safety/input_cmd_vel")
        return (len(wheel_sources) == 1 and wheel_sources[0].node_name == "carrierbot_safety_guard"
                and len(input_sources) == 1 and input_sources[0].node_name == node.get_name())

    label = "lien_empty_path_1m"
    if args.static_obstacle:
        label = "lien_static_obstacle"
    if args.crossing:
        label = "lien_crossing_obstacle"
    if args.turn_back:
        label = "lien_turn_back_obstacle"
    if args.check_only:
        label = "lien_readiness"
    result = {"test": label, "passed": False}
    mover = None
    pending_move = None
    move_submitted_at = 0.0
    crossing_started = None
    motion_completed = False
    last_move_sim = -float("inf")
    last_stage = None
    try:
        deadline = time.monotonic() + 15
        ready = False
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                start = pose()
            except TransformException:
                continue
            now = time.monotonic()
            ready = (routing_ok() and goal_pub.get_subscription_count() == 1
                     and command_pub.get_subscription_count() == 1
                     and now - state["scan"] < 0.5 and now - state["raw"] < 0.5
                     and now - state["wheel"] < 0.5)
            if ready:
                break
        if not ready:
            now = time.monotonic()
            try:
                current_pose = list(pose())
                transform_error = None
            except TransformException as error:
                current_pose = None
                transform_error = str(error)
            def endpoints(topic, publishers):
                rows = (node.get_publishers_info_by_topic(topic) if publishers
                        else node.get_subscriptions_info_by_topic(topic))
                return [row.node_namespace.rstrip("/") + "/" + row.node_name for row in rows]
            result["readiness"] = {
                "map_to_base_pose": current_pose,
                "tf_error": transform_error,
                "scan_age_wall_s": now - state["scan"] if state["scan"] else None,
                "guidance_command_age_wall_s": now - state["raw"] if state["raw"] else None,
                "wheel_command_age_wall_s": now - state["wheel"] if state["wheel"] else None,
                "waypoint_subscribers": endpoints("/avoidance_preview/waypoints", False),
                "guard_input_subscribers": endpoints("/safety/input_cmd_vel", False),
                "guard_input_publishers": endpoints("/safety/input_cmd_vel", True),
                "wheel_publishers": endpoints("/carrierbot_controller/cmd_vel_unstamped", True),
                "guidance_publishers": endpoints("/avoidance_preview/cmd_vel", True),
                "nodes": [namespace.rstrip("/") + "/" + name
                          for name, namespace in node.get_node_names_and_namespaces()],
            }
            raise RuntimeError("Map TF, scan, guidance or guard route is not ready. Keep start_lien_test.sh running.")
        if args.check_only:
            result.update(ready=True, passed=True)
            print("SAN SANG: TF, scan, guidance and guard route. Chua gui waypoint.", flush=True)
            return
        parameters = node.create_client(GetParameters, "/fablab_avoidance_preview/get_parameters")
        try:
            if not parameters.wait_for_service(timeout_sec=3):
                raise RuntimeError("Could not read guidance speed parameters")
            request = GetParameters.Request()
            request.names = ["linear_speed", "linear_speed_max", "guidance_log_path"]
            future = parameters.call_async(request)
            deadline = time.monotonic() + 5
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if not future.done():
                raise RuntimeError("Guidance parameter request timed out")
            values = future.result().values
            result["configured_speed_mps"] = values[0].double_value
            result["configured_max_speed_mps"] = values[1].double_value
            result["guidance_csv"] = values[2].string_value
            session_file = Path(values[2].string_value).parent / "session.json"
            if session_file.is_file():
                session = json.loads(session_file.read_text(encoding="utf-8"))
                result["recovery_enabled"] = session.get("recovery_enabled", False)
                result["guidance_executable"] = session.get("guidance_executable")
                result["paired_person"] = session.get("paired_person", False)
                if result["paired_person"]:
                    raise RuntimeError("This paired-person session is for sensor alignment. "
                                       "Run check_stereo_input.sh --roi-pose --paired-person. "
                                       "Restart without --paired-person for the existing cylinder motion trials.")
        finally:
            node.destroy_client(parameters)
        if has_obstacle:
            if args.static_obstacle and not (0.5 <= start[0] <= 1.3 and abs(start[1]) <= 0.3 and abs(start[2]) <= 0.25):
                raise RuntimeError("Static test expects the robot near the end of the 1 m baseline")
            if args.crossing and (math.hypot(start[0], start[1]) > 0.4 or abs(start[2]) > 0.25):
                raise RuntimeError("Restart start_lien_test.sh before the crossing case; robot must be near (0, 0)")
            if args.crossing:
                mover = node.create_client(SetEntityState, "/gazebo/set_entity_state")
                if not mover.wait_for_service(timeout_sec=5):
                    raise RuntimeError("Missing /gazebo/set_entity_state for moving the test obstacle")
            sdf = '''<sdf version="1.6"><model name="%s"><static>true</static>
<link name="pole"><collision name="collision"><geometry><cylinder><radius>0.20</radius>
<length>1.0</length></cylinder></geometry></collision><visual name="visual"><geometry>
<cylinder><radius>0.20</radius><length>1.0</length></cylinder></geometry></visual></link>
</model></sdf>''' % obstacle_name
            spawn = subprocess.run(
                ["ros2", "run", "gazebo_ros", "spawn_entity.py", "-stdin",
                 "-entity", obstacle_name, "-x", "3", "-y", str(-2.2 * crossing_sign) if args.crossing else "0",
                 "-z", "0.5", "-timeout", "20"],
                input=sdf, text=True, timeout=30)
            if spawn.returncode:
                raise RuntimeError("Could not spawn " + obstacle_name + ". Restart the simulation before retrying.")
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if state["obstacle_present"] and time.monotonic() - state["scan"] < 0.5:
                    break
            if not state["obstacle_present"]:
                raise RuntimeError("Missing Gazebo model-state scoring data")
            target = (5.0, 0.0)
        elif math.hypot(start[0], start[1]) > 0.4 or abs(start[2]) > 0.25:
            raise RuntimeError("This baseline expects the newly spawned robot near (0, 0), facing +X.")
        else:
            target = (start[0] + math.cos(start[2]), start[1] + math.sin(start[2]))

        result.update(start=list(start), goal=list(target))
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = target
        goal.pose.orientation.z = math.sin(start[2] / 2)
        goal.pose.orientation.w = math.cos(start[2] / 2)
        state["enabled"] = True
        trial_started_wall = time.monotonic()
        trial_started_sim = node.get_clock().now().nanoseconds / 1e9
        goal_pub.publish(goal)
        result["goal_sent_count"] = 1
        print("DA GUI WAYPOINT: x=%.3f y=%.3f." % target, flush=True)

        deadline = time.monotonic() + (300 if args.crossing else 240 if args.static_obstacle else 90)
        last_report = 0.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - state["raw"] > 0.6 or now - state["scan"] > 0.5:
                raise RuntimeError("Guidance or scan stopped updating; ending this trial")
            if has_obstacle and (now - state["truth"] > 1.0 or not state["obstacle_present"]):
                raise RuntimeError("Ground-truth scoring data or the test obstacle disappeared")
            if args.crossing:
                sim_time = node.get_clock().now().nanoseconds / 1e9
                if crossing_started is None and state["robot_x"] >= 1.0:
                    crossing_started = sim_time
                    print("VAT BAT DAU CAT NGANG khi xe qua x=1 m.", flush=True)
                if pending_move is not None:
                    if pending_move.done():
                        if not pending_move.result().success:
                            raise RuntimeError("Gazebo rejected an obstacle movement")
                        pending_move = None
                    elif now - move_submitted_at > 2.0:
                        raise RuntimeError("Gazebo obstacle movement service stopped responding")
                if crossing_started is not None:
                    elapsed = sim_time - crossing_started
                    if elapsed < 8.8:
                        obstacle_y, stage = -2.2 + 0.25 * elapsed, "DI VAO LOI"
                    elif elapsed < 12.8:
                        obstacle_y, stage = 0.0, "CHAN GIUA LOI 4 GIAY MO PHONG"
                    else:
                        progress = min(2.2, 0.25 * (elapsed - 12.8))
                        obstacle_y = -progress if args.turn_back else progress
                        stage = "QUAY DAU VE PHIA CU" if args.turn_back else "ROI LOI"
                        if progress >= 2.2:
                            stage = "DA RA KHOI LOI"
                    if stage != last_stage:
                        print("VAT CAN: " + stage, flush=True)
                        last_stage = stage
                    if pending_move is None and sim_time - last_move_sim >= 0.1 and not motion_completed:
                        request = SetEntityState.Request()
                        request.state.name = obstacle_name
                        request.state.reference_frame = "world"
                        request.state.pose.position.x = 3.0
                        request.state.pose.position.y = crossing_sign * obstacle_y
                        request.state.pose.position.z = 0.5
                        request.state.pose.orientation.w = 1.0
                        pending_move = mover.call_async(request)
                        move_submitted_at = now
                        last_move_sim = sim_time
                    end_direction = -crossing_sign if args.turn_back else crossing_sign
                    motion_completed = (elapsed >= 21.6 and state["obstacle_y"] is not None
                                        and end_direction * state["obstacle_y"] >= 2.15)
            if now - last_report >= 1.0:
                if not routing_ok():
                    raise RuntimeError("Unexpected velocity publisher; ending this trial")
                last_report = now
                current = pose()
                remaining = math.hypot(current[0] - target[0], current[1] - target[1])
                print("Xe (%.3f, %.3f) | con %.3f m" % (current[0], current[1], remaining), flush=True)
            if state["arrived"]:
                break
        final = pose()
        error = math.hypot(final[0] - target[0], final[1] - target[1])
        result.update(final=list(final), goal_error_m=error, arrival=state["arrived"],
                      max_wheel_speed_mps=state["max_wheel_speed"],
                      elapsed_wall_s=time.monotonic() - trial_started_wall,
                      elapsed_sim_s=node.get_clock().now().nanoseconds / 1e9 - trial_started_sim)
        result["passed"] = state["arrived"] and error <= 0.20 and state["max_wheel_speed"] > 0.01
        if has_obstacle:
            distance = state["minimum_center_distance_m"]
            result.update(minimum_center_distance_m=distance,
                          minimum_clearance_m=distance - 0.52 - 0.20,
                          maximum_lateral_m=state["maximum_lateral_m"],
                          clearance_model="circular robot radius 0.52 m, cylinder radius 0.20 m")
            result["passed"] = result["passed"] and distance >= 0.82
            if args.static_obstacle:
                result["passed"] = result["passed"] and state["maximum_lateral_m"] >= 0.5
        if args.crossing:
            direction = "+Y to -Y" if args.reverse_crossing else "-Y to +Y"
            if args.turn_back:
                direction = "+Y to center to +Y" if args.reverse_crossing else "-Y to center to -Y"
            result.update(crossing_triggered=crossing_started is not None,
                          crossing_completed=motion_completed and not args.turn_back,
                          obstacle_maneuver_completed=motion_completed,
                          turn_back=args.turn_back,
                          obstacle_y_min_m=state["obstacle_y_min"],
                          obstacle_y_max_m=state["obstacle_y_max"],
                          obstacle_speed_mps=0.25, center_hold_sim_s=4.0,
                          crossing_direction=direction)
            required_span = 2.0 if args.turn_back else 4.0
            result["passed"] = (result["passed"] and motion_completed
                                and state["obstacle_y_max"] - state["obstacle_y_min"] >= required_span)
        print("DAT: xe toi dich." if result["passed"] else "CHUA DAT: xem ket qua va log guidance.", flush=True)
    except KeyboardInterrupt:
        result["error"] = "User stopped the trial"
    except Exception as error:
        result["error"] = str(error)
        print("STOP: " + str(error), flush=True)
    finally:
        state["enabled"] = False
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline and rclpy.ok():
            command_pub.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)
        output = Path.home() / "carrierbot_test_runs"
        output.mkdir(exist_ok=True)
        if result.get("guidance_csv"):
            try:
                with Path(result["guidance_csv"]).open(encoding="utf-8") as file:
                    result["recovery_events"] = max(
                        (int(row.get("recovery_count") or 0) for row in csv.DictReader(file)),
                        default=0)
            except (OSError, ValueError) as error:
                result["guidance_log_read_error"] = str(error)
        report = output / (label + "_" + time.strftime("%Y%m%d_%H%M%S") + ".json")
        report.write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(result, indent=2), flush=True)
        print("KET QUA: " + str(report), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
