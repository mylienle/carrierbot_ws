#!/usr/bin/env python3
"""Observe Huan's existing CPU Vision output at rest. No motion commands."""
from collections import Counter
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


def number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def vector(value):
    return isinstance(value, list) and len(value) == 3 and all(number(v) for v in value)


def inspect_track(packet, track, now):
    reasons = []
    stamp = packet.get("timestamp")
    fresh = number(stamp) and -0.001 <= now - stamp <= 0.5
    if not fresh:
        reasons.append("STALE_OR_FUTURE_PACKET")
    if not isinstance(track, dict):
        return False, False, ["INVALID_TRACK"]
    ident = track.get("id")
    if not isinstance(ident, int) or isinstance(ident, bool) or ident < 0:
        reasons.append("INVALID_TRACK_ID")
    if track.get("reference_valid") is not True:
        reasons.append("NO_IMAGE_TIME_TF")
    if track.get("odom_frame_id") != "odom" or track.get("odom_source") != "tf2_ekf":
        reasons.append("INVALID_REFERENCE_FRAME")
    if not vector(track.get("xyz_odom_m")):
        reasons.append("NO_ODOM_POSITION")
    if not number(track.get("depth_m")) or not 0.3 <= track["depth_m"] <= 8.0:
        reasons.append("NO_VALID_DEPTH")
    if track.get("pose_status") not in ("OK", "OK_PROFILE"):
        reasons.append("POSE_NOT_READY")
    if not number(track.get("confidence")) or not 0.4 <= track["confidence"] <= 1.0:
        reasons.append("LOW_CONFIDENCE")
    for key in ("timestamp", "reference_query_stamp", "reference_transform_stamp"):
        value = track.get(key)
        if not number(value) or not number(stamp) or abs(value - stamp) > 0.001:
            reasons.append("INVALID_TIMESTAMP_" + key.upper())
    position_ok = not reasons
    velocity_ok = position_ok and track.get("velocity_odom_valid") is True and vector(track.get("velocity_odom_mps"))
    if not velocity_ok:
        reasons.append("VELOCITY_" + str(track.get("velocity_odom_status", "UNAVAILABLE")))
    return position_ok, velocity_ok, reasons


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--roi-pose", action="store_true",
                        help="Use person-ROI pose inference in a separate generated entry point")
    parser.add_argument("--paired-person", action="store_true",
                        help="Compare stereo positions with time-matched LiDAR clusters; robot stays at rest")
    parser.add_argument("--decision-log", action="store_true",
                        help="Record per-person proposals for a hypothetical path; requires ROI and paired-person")
    parser.add_argument("--fablab-route", action="store_true",
                        help="Use the internal Fablab route, send preview waypoints, keep wheels disconnected")
    args = parser.parse_args()
    if args.decision_log and not (args.roi_pose and args.paired_person):
        parser.error("--decision-log requires --roi-pose --paired-person")
    if args.fablab_route and not args.decision_log:
        parser.error("--fablab-route requires --roi-pose --paired-person --decision-log")
    vision = Path.home() / "carrierbot-vm.q09FUG/Robot_Vision_Training"
    python = vision / ".venv-vm/bin/python"
    entry = vision / "main_vm_cpu.py"
    config_path = vision / "config.vm.cpu.yaml"
    for path in (python, entry, config_path, vision / "yolo11n.pt", vision / "yolo11s-pose.pt"):
        if not path.is_file():
            raise RuntimeError("Missing input: " + str(path))
    if 'device="cpu"' not in entry.read_text(encoding="utf-8"):
        raise RuntimeError("Expected the previously tested CPU entry point")
    rclpy.init()
    node = rclpy.create_node("check_stereo_input")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    output = None
    child = None
    log = None
    packets = None
    alignment_file = None
    decision_file = None
    decision = None
    alignment = None
    availability = None
    route_reader = None
    if args.paired_person:
        from sensor_alignment_observer import AlignmentObserver, AlignmentAvailability
        alignment = AlignmentObserver(node, buffer, verbose=not args.decision_log)
        availability = AlignmentAvailability(pause_limit=5.0)
    if args.fablab_route:
        from fablab_route import FablabRouteReader
        route_reader = FablabRouteReader(node, buffer)
    summary = {"stage": "fablab_route_decision_shadow" if args.fablab_route else "stereo_input_contract", "passed": False,
               "pose_input_mode": "person_roi_416" if args.roi_pose else "full_frame_416",
               "packets": 0, "packets_with_tracks": 0, "tracks": 0,
               "valid_position_tracks": 0, "valid_velocity_tracks": 0,
               "prediction_valid_tracks": 0}
    reasons = Counter()

    def receive(message):
        if packets is None:
            return
        try:
            packet = json.loads(message.data)
        except (ValueError, TypeError):
            reasons["INVALID_JSON"] += 1
            return
        if not isinstance(packet, dict) or not isinstance(packet.get("tracks"), list):
            reasons["INVALID_PACKET_SCHEMA"] += 1
            return
        now = node.get_clock().now().nanoseconds / 1e9
        packets.write(json.dumps({"received_sim": now, "packet": packet}) + "\n")
        packets.flush()
        summary["packets"] += 1
        if packet["tracks"]:
            summary["packets_with_tracks"] += 1
        for track in packet["tracks"]:
            summary["tracks"] += 1
            valid_position, valid_velocity, problems = inspect_track(packet, track, now)
            summary["valid_position_tracks"] += int(valid_position)
            summary["valid_velocity_tracks"] += int(valid_velocity)
            if isinstance(track, dict) and track.get("prediction_valid") is True:
                summary["prediction_valid_tracks"] += 1
            reasons.update(problems)
            aligned = alignment.observe(packet, track, valid_position) if alignment is not None else None
            if decision is not None:
                decision.observe(packet, track, aligned, now, valid_position, valid_velocity)
            if not args.decision_log and isinstance(track, dict) and (summary["tracks"] <= 4 or summary["tracks"] % 10 == 0):
                print(json.dumps({"id": track.get("id"), "reference_valid": track.get("reference_valid"),
                      "xyz_odom_m": track.get("xyz_odom_m"), "velocity_odom_mps": track.get("velocity_odom_mps"),
                      "velocity_status": track.get("velocity_odom_status"),
                      "age_sim_s": now - packet["timestamp"] if number(packet.get("timestamp")) else None,
                      "usable_for_risk": valid_velocity}), flush=True)

    subscription = node.create_subscription(String, "/vision/tracks", receive, 1)
    try:
        deadline = time.monotonic() + 12
        ready = False
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                buffer.lookup_transform("odom", "camera_rgb_optical_frame", Time())
                ready = all(node.count_publishers(topic) > 0 for topic in
                            ("/rgb/image_raw", "/depth/image_raw", "/scan"))
            except TransformException:
                pass
            if ready and (alignment is None or alignment.ready()):
                break
            ready = False
        if not ready:
            raise RuntimeError("Start start_lien_test.sh --recovery%s and wait for SAN SANG PREVIEW" %
                               (" --paired-person" if alignment is not None else ""))
        if node.count_publishers("/safety/input_cmd_vel"):
            raise RuntimeError("A motion trial is still connected. Finish that trial before the input check.")
        if node.count_publishers("/vision/tracks"):
            raise RuntimeError("Another Vision node is already publishing. Close it before this input check.")
        runs = Path.home() / "carrierbot_test_runs"
        runs.mkdir(exist_ok=True)
        prefix_name = "fablab_route_shadow_" if args.fablab_route else ("decision_shadow_" if args.decision_log else "stereo_input_")
        output = Path(tempfile.mkdtemp(prefix=time.strftime(prefix_name+"%Y%m%d_%H%M%S_"), dir=str(runs)))
        if route_reader is not None:
            from fablab_route import ensure_preview_mission
            print("DANG GUI/DOI CHIEU TUYEN PREVIEW. LOG: " + str(output), flush=True)
            mission = ensure_preview_mission(node, route_reader, alignment.snapshot)
            summary["preview_mission"] = mission
            print("DA CO TUYEN NOI BO FABLAB: %s. Xe van dung cho; khong noi lenh van toc." %
                  mission["mission_waypoints_map"], flush=True)

        if args.roi_pose:
            tools = Path(__file__).resolve().parent
            if not (tools / "carrierbot_roi_pose.py").is_file():
                raise RuntimeError("Missing carrierbot_roi_pose.py. Update the local tools first.")
            source = entry.read_text(encoding="utf-8")
            original_import = "from src.torso_depth import PoseTorsoDetector, associate_poses, measure_torso_depth"
            original_call = "poses = pose_detector.detect(frame) if pose_detector is not None else []"
            if source.count(original_import) != 1 or source.count(original_call) != 1:
                raise RuntimeError("The CPU source differs from the reviewed entry point")
            source = source.replace(original_import,
                "from src.torso_depth import associate_poses, measure_torso_depth\n"
                "from carrierbot_roi_pose import PoseTorsoDetector")
            source = source.replace(original_call,
                "poses = pose_detector.detect(frame, tracks) if pose_detector is not None else []")
            prefix = "import sys\nsys.path.insert(0, %r)\nsys.path.insert(0, %r)\n" % (str(vision), str(tools))
            compile(prefix + source, "main_roi_cpu.py", "exec")
            entry = output / "main_roi_cpu.py"
            entry.write_text(prefix + source, encoding="utf-8")
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        if args.decision_log and config.get("detection", {}).get("class_id") != 0:
            raise RuntimeError("The reviewed decision logger expects Huan's person-only class_id=0 source")
        config["input"]["reference_frame"] = "odom"
        config["motion_compensation"] = {"enabled": True, "reference_source": "tf2_ekf"}
        config["prediction"]["enabled"] = True
        config["output"].update(show_window=False, save_csv=False, save_video=False)
        (output / "config.yaml").write_text(yaml.safe_dump(config), encoding="utf-8")
        log = (output / "vision.log").open("w", encoding="utf-8")
        packets = (output / "tracks.jsonl").open("w", encoding="utf-8")
        if alignment is not None:
            alignment_file = (output / "alignment.jsonl").open("w", encoding="utf-8")
            alignment.output = alignment_file
        if args.decision_log:
            from decision_shadow_logger import DecisionShadowLogger
            decision_file = (output / "decisions.jsonl").open("w", encoding="utf-8")
            decision = DecisionShadowLogger(buffer, decision_file, route_reader=route_reader)
        child = subprocess.Popen([str(python), "-u", str(entry), "--config", str(output / "config.yaml"),
                                  "--max-frames", "60"], cwd=str(vision), stdout=log,
                                 stderr=subprocess.STDOUT, start_new_session=True)
        print(("DANG GHI DE XUAT QUYET DINH. Xe dung cho; khong gui lenh xe." if args.decision_log else
               "DANG KIEM TRA DAU VAO STEREO. Xe dang dung cho.")+"\nLOG: "+str(output), flush=True)
        print("POSE INPUT: " + summary["pose_input_mode"], flush=True)
        if alignment is not None:
            summary["stage"] = ("fablab_route_decision_shadow" if args.fablab_route else
                                "dynamic_decision_shadow" if args.decision_log else "paired_person_sensor_alignment")
            print(("TUYEN FABLAB THUC: hinh hoc tu node; thoi gian theo toc do danh dinh la uoc tinh. Xe dung cho."
                   if args.fablab_route else
                   "TUYEN GIA DINH: tien thang 0.12 m/s trong 2 giay. Khong phai tuyen Fablab da gui." if args.decision_log else
                   "DOI CHIEU VI TRI STEREO-LIDAR. Van toc va quyet dinh ne chua duoc cham."), flush=True)
        started_wall = time.monotonic()
        started_sim = node.get_clock().now().nanoseconds / 1e9
        deadline = time.monotonic() + 180
        last_status = started_wall
        while time.monotonic() < deadline and summary["packets"] < 40:
            rclpy.spin_once(node, timeout_sec=0.1)
            if alignment is not None:
                snapshot = alignment.snapshot()
                old_event_count = len(availability.events)
                action = availability.update(snapshot, time.monotonic())
                if len(availability.events) > old_event_count:
                    event = availability.events[-1]
                    if event["event"] == "paused":
                        print("TAM DUNG CHAM, CHO DU LIEU MOI (toi da 5 giay): " +
                              ", ".join(snapshot["blockers"]), flush=True)
                    elif event["event"] == "resumed":
                        print("DA CO DU LIEU MOI. Tiep tuc doi chieu; mau trong khoang mat du lieu khong duoc chap nhan.", flush=True)
                if action == "abort":
                    summary["readiness_at_failure"] = snapshot
                    raise RuntimeError("Alignment stopped (%s): %s" %
                                       (availability.events[-1]["event"], ", ".join(snapshot["blockers"])))
            if time.monotonic() - last_status >= 10:
                last_status = time.monotonic()
                print("Dang thu: %d packets, %d vi tri hop le, %d van toc hop le" %
                      (summary["packets"], summary["valid_position_tracks"], summary["valid_velocity_tracks"]), flush=True)
            if child.poll() is not None:
                summary["vision_exit_code"] = child.returncode
                break
        summary["elapsed_wall_s"] = time.monotonic() - started_wall
        summary["elapsed_sim_s"] = node.get_clock().now().nanoseconds / 1e9 - started_sim
        summary["observed_rtf"] = summary["elapsed_sim_s"] / summary["elapsed_wall_s"]
        summary["passed"] = summary["valid_velocity_tracks"] >= 5
        summary["reasons"] = dict(reasons)
        if alignment is not None:
            summary["alignment"] = alignment.finish(summary["tracks"])
            summary["passed"] = summary["alignment"]["passed"]
            if decision is not None:
                summary["decision_shadow"] = decision.finish()
                summary["passed"] = summary["passed"] and summary["decision_shadow"]["recording_complete"]
                print("%s GHI DE XUAT. Xem decisions.jsonl va summary.json; chua cham ne vong kin." %
                      ("DAT" if summary["passed"] else "CHUA DAT"), flush=True)
            else:
                print("%s DOI CHIEU VI TRI. Xem alignment.jsonl va summary.json." %
                      ("DAT" if summary["passed"] else "CHUA DAT"), flush=True)
        elif summary["passed"]:
            print("DAT DAU VAO: co track vi tri va van toc hop le trong odom.", flush=True)
        elif summary["valid_position_tracks"]:
            print("CO VI TRI HOP LE, CHUA DU VAN TOC DE DANH GIA NGUY CO.", flush=True)
        else:
            print("CHUA DAT DAU VAO: xem reasons va vision.log.", flush=True)
    except KeyboardInterrupt:
        summary["error"] = "User stopped the input check"
    except Exception as error:
        summary["error"] = str(error)
        if alignment is not None and "readiness_at_failure" not in summary:
            summary["readiness_at_failure"] = alignment.snapshot()
        print("STOP: " + str(error), flush=True)
    finally:
        if availability is not None:
            summary["readiness_pause_limit_wall_s"] = availability.pause_limit
            summary["readiness_events"] = availability.events
        if child is not None:
            summary["vision_exit_code_before_cleanup"] = child.poll()
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
        if decision is not None and "decision_shadow" not in summary:
            summary["decision_shadow"] = decision.finish()
        for file in (log, packets, alignment_file, decision_file):
            if file is not None:
                file.close()
        if output is not None and (output / "vision.log").is_file():
            text = (output / "vision.log").read_text(encoding="utf-8", errors="replace")
            summary["vision_log_tail"] = "\n".join(text.splitlines()[-70:])[-10000:]
        print(json.dumps(summary, indent=2), flush=True)
        if output is not None:
            (output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
            print("KET QUA: " + str(output / "summary.json"), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return summary["passed"]


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
