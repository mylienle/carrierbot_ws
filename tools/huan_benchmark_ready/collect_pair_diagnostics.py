#!/usr/bin/env python3
"""Collect an existing trial's logs and a separate current, read-only ROS snapshot."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile
import time


def tail(path, lines=70, limit=10000):
    if not path.is_file():
        return None
    return "\n".join(path.read_text(encoding="utf-8", errors="replace").splitlines()[-lines:])[-limit:]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", type=Path, required=True, help="Existing stereo_input run folder")
    parser.add_argument("--open", action="store_true", help="Open the collected results folder")
    args = parser.parse_args()
    run = args.run.expanduser().resolve()
    summary_file = run / "summary.json"
    if not summary_file.is_file():
        raise RuntimeError("Missing summary.json in " + str(run))
    result = {"stage": "paired_run_diagnostics", "source_run": str(run),
              "previous_summary": json.loads(summary_file.read_text(encoding="utf-8")),
              "vision_log_tail": tail(run / "vision.log"),
              "note": "Live snapshots are collected AFTER the failed run. They cannot reconstruct a past transient."}
    root = Path.home() / "carrierbot_test_runs"
    root.mkdir(exist_ok=True)
    # Session metadata is written before launch and is not modified during the trial.
    sessions = [p for p in root.glob("lien_*/session.json")
                if p.stat().st_mtime <= summary_file.stat().st_mtime]
    if sessions:
        session_file = max(sessions, key=lambda p: p.stat().st_mtime)
        result["candidate_startup_folder"] = str(session_file.parent)
        result["candidate_startup_session"] = json.loads(session_file.read_text(encoding="utf-8"))
        result["startup_log_tails"] = {name: tail(session_file.parent / name) for name in
                                       ("paired_person.log", "gazebo.log", "guard.log")}
    import rclpy
    from sensor_alignment_observer import AlignmentObserver
    from tf2_ros import Buffer, TransformListener
    rclpy.init()
    node = rclpy.create_node("collect_pair_diagnostics")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    observer = AlignmentObserver(node, buffer)
    snapshots = []
    try:
        print("DANG GOM LOG CU VA DOC TRANG THAI HIEN TAI. Khong khoi dong Vision hoac gui lenh xe.", flush=True)
        deadline = time.monotonic() + 6
        last = 0.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - last >= 0.5:
                last = time.monotonic()
                snapshots.append(observer.snapshot())
        result["current_snapshots"] = snapshots
        result["current_nodes"] = [namespace.rstrip("/") + "/" + name
                                   for name, namespace in node.get_node_names_and_namespaces()]
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    output = Path(tempfile.mkdtemp(prefix=time.strftime("paired_diagnostics_%Y%m%d_%H%M%S_"), dir=str(root)))
    path = output / "diagnostics.json"
    path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print("KET QUA: " + str(path), flush=True)
    if snapshots:
        print("HIEN TAI: " + json.dumps(snapshots[-1]), flush=True)
    if args.open:
        subprocess.Popen(["xdg-open", str(output)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


if __name__ == "__main__":
    main()
