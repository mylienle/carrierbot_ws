#!/usr/bin/env python3
"""Read-only snapshot of guidance, guard commands and logs for a stuck trial."""
from collections import deque
import csv
import json
from pathlib import Path
import time

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import GetParameters


def main():
    rclpy.init()
    node = rclpy.create_node("inspect_lien_test")
    samples = {}
    models = {}
    subscriptions = []
    for topic in ("/avoidance_preview/cmd_vel", "/safety/input_cmd_vel",
                  "/carrierbot_controller/cmd_vel_unstamped"):
        samples[topic] = {"messages": 0, "maximum_abs_v": 0.0, "maximum_abs_w": 0.0}

        def command(message, key=topic):
            row = samples[key]
            row["messages"] += 1
            row["last_v"] = message.linear.x
            row["last_w"] = message.angular.z
            row["maximum_abs_v"] = max(row["maximum_abs_v"], abs(message.linear.x))
            row["maximum_abs_w"] = max(row["maximum_abs_w"], abs(message.angular.z))

        subscriptions.append(node.create_subscription(Twist, topic, command, 1))

    def truth(message):
        for name in ("carrierbot", "lien_crossing_test", "lien_static_test"):
            if name in message.name:
                pose = message.pose[message.name.index(name)]
                models[name] = {"x": pose.position.x, "y": pose.position.y,
                                "orientation_z": pose.orientation.z,
                                "orientation_w": pose.orientation.w}

    subscriptions.append(node.create_subscription(ModelStates, "/gazebo/model_states", truth, 1))
    client = node.create_client(GetParameters, "/fablab_avoidance_preview/get_parameters")
    future = None
    if client.wait_for_service(timeout_sec=2):
        request = GetParameters.Request()
        request.names = ["guidance_log_path"]
        future = client.call_async(request)
    deadline = time.monotonic() + 5
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    root = Path.home() / "carrierbot_test_runs"
    sessions = sorted((p for p in root.glob("lien_*") if p.is_dir()),
                      key=lambda p: p.stat().st_mtime, reverse=True)
    session = sessions[0] if sessions else None
    if future is not None and future.done():
        response = future.result()
        if response is not None and response.values[0].string_value:
            session = Path(response.values[0].string_value).parent
    print("LIVE SNAPSHOT (5 seconds)")
    print(json.dumps({"commands": samples, "gazebo_models": models}, indent=2))
    if session is None:
        print("No session log directory found")
        return
    print("SESSION: " + str(session))
    guidance = session / "guidance_preview.csv"
    if guidance.is_file():
        with guidance.open(encoding="utf-8") as file:
            rows = deque(csv.DictReader(file), maxlen=3)
        print("LAST GUIDANCE ROWS")
        print(json.dumps(list(rows), indent=2))
    for name in ("guard.log", "fablab_preview.log"):
        log = session / name
        if log.is_file():
            print("LAST LOG: " + name)
            with log.open(encoding="utf-8", errors="replace") as file:
                print("".join(deque(file, maxlen=12)))
    reports = sorted(root.glob("lien_turn_back_obstacle_*.json"),
                     key=lambda p: p.stat().st_mtime, reverse=True)
    if reports:
        print("LATEST TURN-BACK REPORT: " + str(reports[0]))
        print(reports[0].read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
