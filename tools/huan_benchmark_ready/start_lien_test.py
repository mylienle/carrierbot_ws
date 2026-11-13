#!/usr/bin/env python3
"""Start the reviewed Classic simulation and Lien's guidance in preview mode.

Python 3.8 / ROS 2 Foxy. No waypoint is sent. All generated files and logs go
to a new directory. Ctrl+C closes only this script's own process groups.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

import yaml

from gazebo_session import ros_domain_conflicts, simulator_environment
from fusion_fixture import (NARROW_DETOUR_WALL_MARGIN_M, OCCLUDED_SCENARIO, SCENARIOS, STANDING_CENTER_GAPS_M,
                            STANDING_SCENARIO, TOOLSET, configure_narrow_corridor, configure_occlusion_screen)


def guard_input_ready(publishers, guidance_name):
    """Wait for ROS discovery to identify the sole publisher to the guard."""
    observed = [(peer.node_namespace, peer.node_name) for peer in publishers]
    expected = [("/", guidance_name)]
    if observed == expected:
        return True
    if not observed or observed in (
        [("_NODE_NAMESPACE_UNKNOWN_", "_NODE_NAME_UNKNOWN_")],
        [("_CREATED_BY_BARE_DDS_APP_", "_CREATED_BY_BARE_DDS_APP_")],
    ):
        return False
    raise RuntimeError("Unexpected source at /safety/input_cmd_vel. Expected %r; found %r"
                       % (expected, observed))


def pose(element):
    item = element.find("pose")
    if item is None:
        return [0.0] * 6
    if item.get("relative_to"):
        raise ValueError("World uses an unsupported relative pose")
    values = [float(value) for value in item.text.split()]
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError("Invalid SDF pose")
    if any(abs(value) > 1e-8 for value in values[3:]):
        raise ValueError("Map builder requires axis-aligned collision boxes")
    return values


def prepare(world_path, urdf_path, controller_path, ekf_path, amcl_path, output, sim_rate=1.0,
            paired_person=False, sensor_fusion=False, yield_clearance_m=.6, early_rejoin=False,
            final_approach_ramp=False, scenario="crossing", sim_lidar_rate=None):
    tree = ET.parse(str(world_path))
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError("Missing SDF world")
    expected = {"vision_room_14x10", "mission_wall_-1", "mission_wall_1"}
    if {model.get("name") for model in world.findall("model")} != expected:
        raise ValueError("This starter expects the original wide crossing world")
    if scenario == "head-on-narrow":
        configure_narrow_corridor(world)
    elif scenario == OCCLUDED_SCENARIO:
        configure_occlusion_screen(world)
    physics = world.find("physics")
    if physics is None:
        raise ValueError("Missing world physics configuration")
    step = float(physics.findtext("max_step_size", "0"))
    if not math.isfinite(step) or step <= 0:
        raise ValueError("Invalid physics step size")
    for name, value in (("real_time_update_rate", sim_rate / step), ("real_time_factor", sim_rate)):
        setting = physics.find(name)
        if setting is None:
            setting = ET.SubElement(physics, name)
        setting.text = str(value)
    boxes = []
    for model in world.findall("model"):
        if model.findtext("static") != "true":
            raise ValueError("Expected static room geometry")
        for link in model.findall("link"):
            for collision in link.findall("collision"):
                size = collision.findtext("geometry/box/size")
                if size is None:
                    raise ValueError("Unsupported collision shape in room")
                sx, sy, sz = map(float, size.split())
                if not all(math.isfinite(v) and v > 0 for v in (sx, sy, sz)):
                    raise ValueError("Invalid collision box dimensions")
                x, y, z = [sum(pose(e)[i] for e in (model, link, collision))
                           for i in range(3)]
                if z - sz / 2 <= 0.5 <= z + sz / 2:
                    boxes.append((x - sx / 2, y - sy / 2,
                                  x + sx / 2, y + sy / 2))
    expected_box_count = 7 if scenario in ("head-on-narrow", OCCLUDED_SCENARIO) else 6
    if len(boxes) != expected_box_count:
        raise ValueError("Expected %d room collision boxes" % expected_box_count)

    resolution = 0.05
    xmin = min(box[0] for box in boxes)
    ymin = min(box[1] for box in boxes)
    xmax = max(box[2] for box in boxes)
    ymax = max(box[3] for box in boxes)
    ox, oy = math.floor(xmin - 0.5), math.floor(ymin - 0.5)
    width = int(round((math.ceil(xmax + 0.5) - ox) / resolution))
    height = int(round((math.ceil(ymax + 0.5) - oy) / resolution))
    pixels = bytearray([205]) * (width * height)
    for row in range(height):
        y = oy + (height - row - 0.5) * resolution
        for col in range(width):
            x = ox + (col + 0.5) * resolution
            if xmin <= x <= xmax and ymin <= y <= ymax:
                occupied = any(a <= x <= c and b <= y <= d for a, b, c, d in boxes)
                pixels[row * width + col] = 0 if occupied else 254
    (output / "crossing_map.pgm").write_bytes(
        ("P5\n%d %d\n255\n" % (width, height)).encode("ascii") + pixels)
    (output / "crossing_map.yaml").write_text(yaml.safe_dump({
        "image": "crossing_map.pgm", "resolution": resolution,
        "origin": [ox, oy, 0.0], "negate": 0,
        "occupied_thresh": 0.65, "free_thresh": 0.196,
    }), encoding="utf-8")

    for plugin in world.findall("plugin"):
        if plugin.get("name") == "dacn_evaluator_direct_truth":
            world.remove(plugin)
    if paired_person:
        from person_fixture import configure
        fixture = configure(world)
        (output / "person_fixture.json").write_text(json.dumps(fixture, indent=2), encoding="utf-8")
    if sensor_fusion:
        from fusion_fixture import configure
        config = configure(world, output, scenario)
        config["integration_toolset"] = TOOLSET
        config["startup_session_file"] = str(output/"session.json")
        config["yield_clearance_m"] = yield_clearance_m
        config["early_rejoin_enabled"] = early_rejoin
        config["final_approach_ramp_enabled"] = final_approach_ramp
        config["yield_clearance_reference"] = "Distance from the 0.52m circular body model to a measured LiDAR point in the forward route corridor"
        (output / "fusion_fixture.json").write_text(json.dumps(config, indent=2), encoding="utf-8")
    tree.write(str(output / "room.world"), encoding="utf-8", xml_declaration=True)

    robot = ET.parse(str(urdf_path))
    control = robot.find(".//plugin[@filename='libgazebo_ros2_control.so']/parameters")
    if control is None:
        raise ValueError("Missing Gazebo Classic controller configuration")
    control.text = str(controller_path)
    if sim_lidar_rate is not None:
        rate = robot.find(".//sensor[@name='laser'][@type='ray']/update_rate")
        if rate is None:
            raise ValueError("Missing Gazebo laser update rate in the test URDF")
        rate.text = str(sim_lidar_rate)
    robot.write(str(output / "robot.urdf"), encoding="utf-8", xml_declaration=True)
    (output / "rsp.yaml").write_text(yaml.safe_dump({
        "robot_state_publisher": {"ros__parameters": {
            "use_sim_time": True,
            "robot_description": (output / "robot.urdf").read_text(encoding="utf-8"),
        }}
    }), encoding="utf-8")

    ekf = yaml.safe_load(ekf_path.read_text(encoding="utf-8"))
    ekf_params = ekf["ekf_filter_node"]["ros__parameters"]
    ekf_params.pop("imu0", None)
    ekf_params["use_sim_time"] = True
    ekf_params["odom0"] = "/odom"
    (output / "ekf_wheel.yaml").write_text(yaml.safe_dump(ekf), encoding="utf-8")

    amcl = yaml.safe_load(amcl_path.read_text(encoding="utf-8"))
    params = amcl["amcl"]["ros__parameters"]
    params.update(use_sim_time=True, set_initial_pose=True,
                  initial_pose={"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0})
    (output / "amcl.yaml").write_text(yaml.safe_dump(amcl), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=float, choices=(0.12, 0.20), default=0.12,
                        help="Configured guidance speed in m/s")
    parser.add_argument("--recovery", action="store_true",
                        help="Use the separate build with blocked-detour recovery")
    parser.add_argument("--sim-rate", type=float, choices=(0.35, 1.0), default=1.0,
                        help="Target simulation/wall-time ratio, 0.35 for CPU perception experiments")
    parser.add_argument("--sim-lidar-rate", type=float, choices=(10., 20.),
                        help="LaserScan rate in simulated Hz in this Gazebo session")
    parser.add_argument("--paired-person", action="store_true",
                        help="Use one animated person with a synchronized LiDAR collision proxy")
    parser.add_argument("--route-preview", action="store_true",
                        help="Use the separate recovery build that exports internal Fablab route telemetry")
    parser.add_argument("--behavior-gate", action="store_true",
                        help="Wire a separately built directive-gated Fablab node to the existing safety guard")
    parser.add_argument("--sensor-fusion", action="store_true",
                        help="Stage a late-spawned person for the sensor-driven yielding trial")
    parser.add_argument("--scenario", choices=SCENARIOS, default="crossing",
                        help="Person trajectory for the sensor-fusion trial")
    parser.add_argument("--yield-clearance", type=float, default=.6,
                        help="Simulated LiDAR yielding clearance from body surface in metres (default 0.6)")
    parser.add_argument("--legacy-yield-zone", action="store_true",
                        help="Use the previous 3m corridor trigger only for a comparison run")
    parser.add_argument("--early-rejoin", action="store_true",
                        help="Test one early return to the original route in the fusion simulation")
    parser.add_argument("--final-approach-ramp", action="store_true",
                        help="Test a distance-based speed taper on the final leg after early rejoin")
    parser.add_argument("--clear-cruise", action="store_true",
                        help="Use the existing nominal speed on clear aligned stretches of the simulation")
    parser.add_argument("--yield-before-detour", action="store_true",
                        help="Defer new Fablab detours during a sensor-driven SLOW directive")
    standing_gap = parser.add_mutually_exclusive_group()
    standing_gap.add_argument("--compact-standing-detour", action="store_true",
                              help="Try a 1.00 m planned body-to-person gap in the standing-blocker simulation")
    standing_gap.add_argument("--standing-center-gap", type=float, choices=STANDING_CENTER_GAPS_M,
                              help="Set the minimum planned centre separation in the standing-blocker simulation")
    parser.add_argument("--standing-post-pass-cruise", action="store_true",
                        help="Try 0.08 m/s after passing the standing person on a clear rejoin leg")
    args = parser.parse_args()
    if not math.isfinite(args.yield_clearance) or not .6 <= args.yield_clearance <= 3.:
        parser.error("--yield-clearance must be finite and between 0.6 and 3.0 metres")
    if args.route_preview and not args.recovery:
        parser.error("--route-preview requires --recovery")
    if args.behavior_gate and (not args.route_preview or args.paired_person or args.speed != .12):
        parser.error("The first --behavior-gate trial requires --route-preview, --speed 0.12 and no --paired-person")
    if args.sensor_fusion and (not args.behavior_gate or args.paired_person):
        parser.error("--sensor-fusion requires --behavior-gate and no --paired-person")
    if args.scenario != "crossing" and not args.sensor_fusion:
        parser.error("The selected scenario requires --sensor-fusion")
    if args.early_rejoin and not args.sensor_fusion:
        parser.error("--early-rejoin requires --sensor-fusion")
    if args.final_approach_ramp and not args.early_rejoin:
        parser.error("--final-approach-ramp requires --early-rejoin")
    if args.clear_cruise and not args.sensor_fusion:
        parser.error("--clear-cruise requires --sensor-fusion")
    if args.yield_before_detour and not args.sensor_fusion:
        parser.error("--yield-before-detour requires --sensor-fusion")
    if args.compact_standing_detour and (args.scenario != STANDING_SCENARIO or not args.yield_before_detour):
        parser.error("--compact-standing-detour requires --scenario standing-blocker and --yield-before-detour")
    if args.standing_center_gap is not None and (args.scenario != STANDING_SCENARIO or not args.yield_before_detour):
        parser.error("--standing-center-gap requires --scenario standing-blocker and --yield-before-detour")
    if args.standing_post_pass_cruise and args.standing_center_gap != 1.65:
        parser.error("--standing-post-pass-cruise requires --standing-center-gap 1.65")
    detour_wall_margin = NARROW_DETOUR_WALL_MARGIN_M if args.scenario == "head-on-narrow" else None
    import rclpy
    from controller_manager_msgs.srv import ListControllers
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    from lifecycle_msgs.srv import ChangeState
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener, TransformException

    home = Path.home()
    lab = home / "carrierbot-vm.q09FUG"
    vision = lab / "Robot_Vision_Training"
    case = vision / "simulation/dynamic_carrierbot_los_crossing_20261101_01"
    trial = home / "carrierbot-lien-foxy.0bkXPj"
    if args.behavior_gate:
        pointer = home / "carrierbot_test_runs/behavior_huan_workspace.txt"
        trial = Path(pointer.read_text(encoding="utf-8").strip()).resolve()
        if trial.parent != home or not trial.name.startswith("carrierbot-lien-gate."):
            raise RuntimeError("Invalid behavior workspace pointer")
        manifest = json.loads((trial / "gate_build_manifest.json").read_text(encoding="utf-8"))
        tools = Path(__file__).resolve().parent
        for relative in ("behavior_gate_source/fablab_guidance_node.cpp", "behavior_gate_source/route_preview.hpp",
                         "behavior_gate_source/behavior_gate.hpp", "recovery_source/avoidance_recovery.hpp"):
            if hashlib.sha256((tools/relative).read_bytes()).hexdigest() != manifest.get(relative):
                raise RuntimeError("Behavior binary is outdated. Run build_behavior_gate.sh again.")
    elif args.route_preview:
        pointer = home / "carrierbot_test_runs/route_workspace.txt"
        trial = Path(pointer.read_text(encoding="utf-8").strip()).resolve()
        if trial.parent != home or not trial.name.startswith("carrierbot-lien-route."):
            raise RuntimeError("Invalid route workspace pointer")
        manifest_path = trial / "route_build_manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        tools = Path(__file__).resolve().parent
        for relative in ("route_preview_source/fablab_guidance_node.cpp", "route_preview_source/route_preview.hpp",
                         "recovery_source/avoidance_recovery.hpp"):
            if hashlib.sha256((tools/relative).read_bytes()).hexdigest() != manifest.get(relative):
                raise RuntimeError("Route binary is outdated. Run build_lien_route_preview.sh again.")
    elif args.recovery:
        pointer = home / "carrierbot_test_runs/recovery_workspace.txt"
        trial = Path(pointer.read_text(encoding="utf-8").strip()).resolve()
        if trial.parent != home or not trial.name.startswith("carrierbot-lien-recovery."):
            raise RuntimeError("Invalid recovery workspace pointer")
    source = lab / "carrierbot_ws/src"
    files = [case / "room.world", case / "robot_hd720.urdf",
             source / "carrierbot_controller/config/controller_sim.yaml",
             source / "carrierbot_slam/config/ekf.yaml",
             source / "carrierbot_slam/config/amcl.yaml",
             home / "carrierbot_ws_latest/tools/safety_guard.py",
             trial / "install/carrierbot_navigation/lib/carrierbot_navigation/fablab_guidance_node"]
    for path in files:
        if not path.is_file():
            raise RuntimeError("Missing file: " + str(path))
    for binary in ("gzserver", "gzclient", "ros2"):
        if not shutil.which(binary):
            raise RuntimeError("Missing command: " + binary)
    for package in ("gazebo_ros", "controller_manager", "robot_localization",
                    "nav2_map_server", "nav2_amcl", "robot_state_publisher"):
        subprocess.run(["ros2", "pkg", "prefix", package], check=True,
                       stdout=subprocess.DEVNULL, timeout=10)
    if os.environ.get("GAZEBO_MASTER_URI") != "http://127.0.0.1:11349":
        raise RuntimeError("Run start_lien_test.sh to select the simulation environment")
    conflicts = ros_domain_conflicts(os.environ)
    if conflicts:
        details = "\n".join("  PID %s: %s" % (item["pid"], " ".join(item["arguments"]))
                            for item in conflicts)
        raise RuntimeError("Another gzserver is using ROS_DOMAIN_ID=39. "
                           "Close that simulation's startup terminal before retrying. "
                           "No process was stopped.\n" + details)
    try:
        connection = socket.create_connection(("127.0.0.1", 11349), timeout=0.5)
    except OSError:
        pass
    else:
        connection.close()
        raise RuntimeError("Gazebo is already using port 11349. Close that simulation first.")

    runs = home / "carrierbot_test_runs"
    runs.mkdir(exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix=time.strftime("lien_%Y%m%d_%H%M%S_"), dir=str(runs)))
    simulation_env = simulator_environment(os.environ, output)
    print("LOG: " + str(output), flush=True)
    print("KENH CANH RIENG: " + simulation_env["IGN_PARTITION"], flush=True)
    prepare(*files[:5], output, sim_rate=args.sim_rate, paired_person=args.paired_person,
            sensor_fusion=args.sensor_fusion,
            yield_clearance_m=None if args.legacy_yield_zone else args.yield_clearance,
            early_rejoin=args.early_rejoin, final_approach_ramp=args.final_approach_ramp,
            scenario=args.scenario, sim_lidar_rate=args.sim_lidar_rate)
    (output / "session.json").write_text(json.dumps({
        "configured_speed_mps": args.speed,
        "scenario": args.scenario if args.sensor_fusion else None,
        "world_source": str(files[0]),
        "guidance_executable": str(files[6]),
        "guidance_executable_sha256": hashlib.sha256(files[6].read_bytes()).hexdigest(),
        "safety_guard_path": str(files[5]),
        "safety_guard_sha256": hashlib.sha256(files[5].read_bytes()).hexdigest(),
        "integration_toolset": TOOLSET,
        "yield_clearance_m": None if args.legacy_yield_zone else args.yield_clearance,
        "recovery_enabled": args.recovery,
        "early_rejoin_enabled": args.early_rejoin,
        "final_approach_ramp_enabled": args.final_approach_ramp,
        "clear_cruise_enabled": args.clear_cruise,
        "yield_before_detour": args.yield_before_detour,
        "compact_standing_detour": args.compact_standing_detour,
        "planned_standing_center_gap_m": args.standing_center_gap,
        "standing_post_pass_cruise": args.standing_post_pass_cruise,
        "detour_wall_margin_m": detour_wall_margin,
        "target_sim_rate": args.sim_rate,
        "sim_lidar_rate_hz": args.sim_lidar_rate,
        "paired_person": args.paired_person,
        "odometry": "wheel_only",
        "gazebo_master_uri": simulation_env["GAZEBO_MASTER_URI"],
        "ign_partition": simulation_env["IGN_PARTITION"],
        "route_preview_enabled": args.route_preview,
        "behavior_gate_enabled": args.behavior_gate,
        "sensor_fusion_enabled": args.sensor_fusion,
    }, indent=2), encoding="utf-8")

    rclpy.init()
    node = rclpy.create_node("lien_test_startup")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    tf_buffer = Buffer(node=node)
    listener = TransformListener(tf_buffer, node)
    received = {}
    subscriptions = []
    for topic, message_type, qos in (
        ("/clock", Clock, qos_profile_sensor_data),
        ("/scan", LaserScan, qos_profile_sensor_data),
        ("/odom", Odometry, qos_profile_sensor_data),
        ("/odometry/filtered", Odometry, qos_profile_sensor_data),
        ("/amcl_pose", PoseWithCovarianceStamped, 10),
        ("/map", OccupancyGrid, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        ("/carrierbot_controller/cmd_vel_unstamped", Twist, 10),
    ):
        def callback(message, key=topic):
            received[key] = (time.monotonic(), message)
        subscriptions.append(node.create_subscription(message_type, topic, callback, qos))

    children = []
    logs = []

    def launch(label, arguments):
        log = (output / (label + ".log")).open("w", encoding="utf-8")
        logs.append(log)
        process = subprocess.Popen(arguments, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True, cwd=str(output), env=simulation_env)
        children.append((label, process))
        return process

    def wait_for(label, predicate, seconds=40):
        print("DANG KIEM TRA: " + label, flush=True)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            for name, process in children:
                if process.poll() is not None:
                    raise RuntimeError("Process stopped: %s. See %s.log" % (name, name))
            if predicate():
                print("OK: " + label, flush=True)
                return
            rclpy.spin_once(node, timeout_sec=0.1)
        raise RuntimeError("Timed out: " + label)

    def fresh(topic, age=2.0):
        return topic in received and time.monotonic() - received[topic][0] < age

    def has_transform(target, child):
        try:
            tf_buffer.lookup_transform(target, child, Time())
            return True
        except TransformException:
            return False

    def run_logged(label, arguments, seconds=45):
        print("DANG CHAY: " + label, flush=True)
        with (output / (label + ".log")).open("w", encoding="utf-8") as log:
            result = subprocess.run(arguments, stdout=log, stderr=subprocess.STDOUT,
                                    timeout=seconds, cwd=str(output), env=simulation_env)
        if result.returncode:
            raise RuntimeError("Command failed: %s. See %s.log" % (label, label))

    def activate(name):
        client = node.create_client(ChangeState, name + "/change_state")
        try:
            wait_for(name + " lifecycle service", lambda: client.service_is_ready())
            for transition in (1, 3):
                request = ChangeState.Request()
                request.transition.id = transition
                future = client.call_async(request)
                wait_for(name + " transition " + str(transition), future.done)
                if not future.result().success:
                    raise RuntimeError("Lifecycle transition failed: " + name)
        finally:
            node.destroy_client(client)

    def wheel_route_ok():
        publishers = node.get_publishers_info_by_topic("/carrierbot_controller/cmd_vel_unstamped")
        if len(publishers) != 1 or publishers[0].node_name != "carrierbot_safety_guard":
            return False
        if not fresh("/carrierbot_controller/cmd_vel_unstamped"):
            return False
        command = received["/carrierbot_controller/cmd_vel_unstamped"][1]
        return command.linear.x == 0.0 and command.angular.z == 0.0

    paired_status = {}
    route_status = {}
    route_topic = "/behavior_gate/route_state" if args.behavior_gate else "/avoidance_preview/route_state"
    guidance_name = "fablab_behavior_guidance" if args.behavior_gate else "fablab_avoidance_preview"
    guidance_command_topic = "/safety/input_cmd_vel" if args.behavior_gate else "/avoidance_preview/cmd_vel"
    waypoint_topic = "/behavior_gate/waypoints" if args.behavior_gate else "/avoidance_preview/waypoints"
    arrival_topic = "/behavior_gate/arrival" if args.behavior_gate else "/avoidance_preview/arrival"
    if args.route_preview:
        def receive_route_status(message):
            try:
                value = json.loads(message.data)
                if isinstance(value, dict) and value.get("source") == "fablab_internal_waypoints_v1":
                    route_status.update(wall=time.monotonic(), data=value)
            except (ValueError, TypeError):
                pass
        subscriptions.append(node.create_subscription(String, route_topic, receive_route_status, 1))
    if args.paired_person:
        def receive_paired_status(message):
            try:
                value = json.loads(message.data)
                if isinstance(value, dict):
                    paired_status.update(wall=time.monotonic(), data=value)
            except (ValueError, TypeError):
                pass
        subscriptions.append(node.create_subscription(String, "/test_fixture/person_lidar_status",
                                                       receive_paired_status, 1))

    try:
        launch("gazebo", ["gzserver", str(output / "room.world"),
                          "-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so"])
        launch("rsp", ["ros2", "run", "robot_state_publisher", "robot_state_publisher",
                       "--ros-args", "--params-file", str(output / "rsp.yaml")])
        wait_for("Gazebo clock", lambda: fresh("/clock"), 50)
        run_logged("spawn", ["ros2", "run", "gazebo_ros", "spawn_entity.py",
                             "-entity", "carrierbot", "-file", str(output / "robot.urdf"),
                             "-timeout", "30"])
        manager = node.create_client(ListControllers, "/controller_manager/list_controllers")
        try:
            wait_for("controller_manager service", manager.service_is_ready, 90)
            manager_ready = manager.call_async(ListControllers.Request())
            wait_for("controller_manager response", manager_ready.done, 30)
            manager_ready.result()
        finally:
            node.destroy_client(manager)
        for name in ("joint_state_broadcaster", "carrierbot_controller"):
            run_logged(name, ["ros2", "run", "controller_manager", "spawner.py", name,
                              "--controller-manager", "/controller_manager"])
        wait_for("LaserScan and wheel odometry", lambda: fresh("/scan") and fresh("/odom"))
        launch("gazebo_gui", ["gzclient"])
        launch("ekf", ["ros2", "run", "robot_localization", "ekf_node", "--ros-args",
                       "-r", "__node:=ekf_filter_node", "--params-file", str(output / "ekf_wheel.yaml")])
        wait_for("EKF and odom TF", lambda: fresh("/odometry/filtered") and has_transform("odom", "base_footprint"))
        launch("map", ["ros2", "run", "nav2_map_server", "map_server", "--ros-args",
                       "-p", "use_sim_time:=true", "-p", "yaml_filename:=" + str(output / "crossing_map.yaml")])
        activate("/map_server")
        wait_for("Static room map", lambda: "/map" in received)
        launch("amcl", ["ros2", "run", "nav2_amcl", "amcl", "--ros-args",
                        "--params-file", str(output / "amcl.yaml")])
        activate("/amcl")
        wait_for("AMCL pose and map TF", lambda: fresh("/amcl_pose", 15) and has_transform("map", "base_footprint"))
        if args.paired_person:
            launch("paired_person", ["/usr/bin/python3", str(Path(__file__).with_name("pair_person_lidar.py"))])
            wait_for("Person visual and LiDAR proxy coupling",
                     lambda: paired_status.get("data", {}).get("ready") is True
                     and time.monotonic() - paired_status.get("wall", 0) <= 0.6)
        launch("guard", ["/usr/bin/python3", str(files[5])])
        wait_for("Single guard publisher and zero wheel command", wheel_route_ok)
        launch("fablab_preview", [str(files[6]), "--ros-args",
               "-r", "__node:=" + guidance_name, "-p", "use_sim_time:=true",
               "-p", "avoidance_enabled:=true", "-p", "linear_speed:=" + str(args.speed),
               "-p", "linear_speed_max:=" + str(args.speed), "-p", "min_speed_linear:=0.03",
               "-p", "angular_speed_max:=0.3", "-p", "corner_max_speed:=0.10",
               "-p", "turn_heading_max_speed:=0.08", "-p", "final_max_speed:=0.08",
               "-p", "start_alignment_angular_speed:=0.2",
               "-p", "waypoint_topic:=" + waypoint_topic,
               "-p", "initial_pose_topic:=/avoidance_preview/initialpose",
               "-p", "guidance_log_path:=" + str(output / "guidance_preview.csv"),
               "-r", "/cmd_vel:=" + guidance_command_topic,
               "-r", "/fablab_arrival:=" + arrival_topic] +
               (["-p", "route_preview_enabled:=true", "-p", "route_preview_topic:=" + route_topic]
                if args.route_preview else []) +
               (["-p", "behavior_gate_enabled:=true"] if args.behavior_gate else []) +
               (["-p", "early_rejoin_enabled:=true"] if args.early_rejoin else []) +
               (["-p", "final_approach_ramp_enabled:=true"] if args.final_approach_ramp else []) +
               (["-p", "clear_cruise_enabled:=true"] if args.clear_cruise else []) +
               (["-p", "yield_before_detour:=true"] if args.yield_before_detour else []) +
               (["-p", "compact_standing_detour:=true"] if args.compact_standing_detour else []) +
               (["-p", "standing_center_gap:=" + str(args.standing_center_gap)]
                if args.standing_center_gap is not None else []) +
               (["-p", "standing_post_pass_cruise:=true"] if args.standing_post_pass_cruise else []) +
               (["-p", "avoid_wall_margin:=" + str(detour_wall_margin)]
                if detour_wall_margin is not None else []))
        if not args.behavior_gate:
            wait_for("Guidance command publisher", lambda: node.count_publishers(guidance_command_topic) == 1)
        if args.route_preview:
            wait_for("Internal Fablab route telemetry", lambda: bool(route_status)
                     and time.monotonic()-route_status["wall"] <= 0.6
                     and node.count_publishers(route_topic) == 1)
        if args.behavior_gate:
            if route_status["data"].get("behavior_gate", {}).get("enabled") is not True:
                raise RuntimeError("The running guidance node does not have the behavior gate enabled")
            wait_for("Sole Fablab publisher at " + guidance_command_topic,
                     lambda: guard_input_ready(node.get_publishers_info_by_topic(guidance_command_topic),
                                               guidance_name), seconds=15)
        elif node.count_subscribers("/avoidance_preview/cmd_vel") != 0:
            raise RuntimeError("Unexpected subscriber to the preview velocity topic")
        if not wheel_route_ok():
            raise RuntimeError("Unexpected publisher or command at wheel input")
        seed = node.create_publisher(PoseWithCovarianceStamped, "/avoidance_preview/initialpose", 1)
        wait_for("Preview pose input", lambda: seed.get_subscription_count() == 1)
        # AMCL may only publish once while the robot is stationary. Seed the
        # new guidance node with that measured pose through its own input.
        seed.publish(received["/amcl_pose"][1])
        position = received["/amcl_pose"][1].pose.pose.position
        fixture_config_publisher = None
        if args.sensor_fusion:
            fixture_config_publisher = node.create_publisher(String, "/test_fixture/fusion_config",
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            fixture_config_publisher.publish(String(data=(output/"fusion_fixture.json").read_text(encoding="utf-8")))
            print("SAN SANG THU LIDAR + STEREO. CA THU: " + args.scenario
                  + ". Nguoi se xuat hien khi chay run_huan_fusion_trial.sh.", flush=True)
        print("\n" + ("SAN SANG THU DIEU PHOI. Xe dang WAIT, chua gui waypoint." if args.behavior_gate
                        else "SAN SANG PREVIEW. Chua gui waypoint."), flush=True)
        print("TOC DO CAU HINH: %.2f m/s" % args.speed, flush=True)
        if args.clear_cruise:
            print("GIU TOC DO TREN DOAN THANG THONG THOANG: BAT", flush=True)
        if args.yield_before_detour:
            print("NHUONG TRUOC KHI LAP DUONG VONG: BAT", flush=True)
        if args.compact_standing_detour:
            print("KHOANG HO LAP DUONG VONG QUA NGUOI DUNG YEN: 1.00 m", flush=True)
        if args.standing_center_gap is not None:
            print("KHOANG CACH GIUA HAI TAM KHI LAP DUONG VONG: %.2f m" % args.standing_center_gap, flush=True)
        if args.standing_post_pass_cruise:
            print("THU TOC DO SAU KHI VUOT NGUOI: TOI DA 0.08 m/s", flush=True)
        if detour_wall_margin is not None:
            print("KHOANG HO TUONG TOI THIEU KHI LAP DUONG VONG: %.2f m" % detour_wall_margin, flush=True)
        if args.sensor_fusion:
            print("VUNG NHUONG LIDAR: " + ("legacy 3m corridor" if args.legacy_yield_zone else
                  "%.2f m tu be mat than xe toi diem LiDAR" % args.yield_clearance), flush=True)
        print("PHUC HOI DUONG NE: " + ("BAT" if args.recovery else "BAN GOC"), flush=True)
        if args.behavior_gate:
            print("DIEU PHOI: Fablab -> safety_guard -> banh xe. Mat directive se WAIT.", flush=True)
        elif args.route_preview:
            print("TUYEN NOI BO FABLAB: BAT. Chi xuat du lieu; chua noi lenh ra banh xe.", flush=True)
        print("NHIP MO PHONG MUC TIEU: %.2f" % args.sim_rate, flush=True)
        if args.sim_lidar_rate is not None:
            print("TAN SO LIDAR MO PHONG: %.1f Hz" % args.sim_lidar_rate, flush=True)
        if args.paired_person:
            print("PERSON PAIRED: camera thay nguoi; LiDAR thay proxy di cung nguoi. Xe dang cho.", flush=True)
        print("AMCL x=%.3f y=%.3f. Xe vua spawn tai (0, 0)." % (position.x, position.y), flush=True)
        print("EKF dang dung odometry banh xe. IMU chua bat.", flush=True)
        print("LOG: " + str(output), flush=True)
        print("Giu terminal nay mo. Ctrl+C de dong phien thu.", flush=True)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            for name, process in children:
                if process.poll() is not None:
                    raise RuntimeError("Process stopped: " + name)
    except KeyboardInterrupt:
        print("\nDang dong phien thu...", flush=True)
    except Exception:
        print("\nLOI. Kiem tra log tai: " + str(output), flush=True)
        for label, process in children:
            if process.poll() is not None:
                print("Process stopped: " + label, flush=True)
        raise
    finally:
        # Signal each owned group before waiting, so a slow process cannot
        # keep downstream control alive while the simulator is shutting down.
        for label, process in reversed(children):
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + 6
        while any(p.poll() is None for _, p in children) and time.monotonic() < deadline:
            time.sleep(0.1)
        for label, process in reversed(children):
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        for log in logs:
            log.close()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print("STOP: " + str(error), file=sys.stderr, flush=True)
        sys.exit(1)
