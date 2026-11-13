"""Read-only estimator comparison on recorded camera measurements; no ROS or motion."""
import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
from huan_cpu_velocity import cpu_sparse_estimator


def load_reviewed(vision, tools=None):
    tools = Path(tools or Path(__file__).resolve().parent)
    expected = json.loads((tools/"huan_vision_core.json").read_text(encoding="utf-8"))["normalized_sha256"]
    modules = []
    for name in ("odom_velocity_estimator", "track_history"):
        relative = "src/"+name+".py"
        path = Path(vision)/relative
        content = path.read_bytes().replace(b"\r\n", b"\n")
        if hashlib.sha256(content).hexdigest() != expected.get(relative):
            raise ValueError("Reviewed source mismatch: "+relative)
        spec = importlib.util.spec_from_file_location("reviewed_"+name, path)
        module = importlib.util.module_from_spec(spec)
        # These two reviewed modules import only NumPy/stdlib, never ROS.
        spec.loader.exec_module(module)
        modules.append(module)
    return modules[0].OdomVelocityEstimator, modules[1].TrackHistory


def compare(packets, original, history_class):
    profiles = {
        "original": original(window_s=.8, min_span_s=.25, min_samples=4, max_gap_s=.6, ema_alpha=.4),
        "cpu-sparse": cpu_sparse_estimator(original)(window_s=.8, min_span_s=.25, min_samples=4,
                                                    max_gap_s=.6, ema_alpha=.4),
    }
    histories = {key: history_class(max_length=30) for key in profiles}
    counts = {key: Counter() for key in profiles}
    valid = Counter()
    comparisons = equal = packets_seen = 0
    for packet in packets:
        packets_seen += 1
        recorded = packet.get("tracks", [])
        for key, estimator in profiles.items():
            tracks = deepcopy(recorded)
            # Retain measured timestamps/XYZ/frame/IDs, including missing XYZ.
            histories[key].update(tracks)
            estimator.update(tracks, histories[key])
            for before, track in zip(recorded, tracks):
                counts[key][track["velocity_odom_status"]] += 1
                valid[key] += int(track.get("velocity_odom_valid") is True)
                if key == "original":
                    comparisons += 1
                    equal += int(before.get("velocity_odom_status") == track["velocity_odom_status"]
                                 and before.get("velocity_odom_valid") == track["velocity_odom_valid"])
    return dict(packets=packets_seen,
                status_counts={key: dict(value) for key, value in counts.items()},
                valid_velocity_tracks={key: valid[key] for key in profiles},
                original_status_flag_agreement=dict(matches=equal, comparisons=comparisons,
                                                   all_match=comparisons > 0 and equal == comparisons),
                candidate_only_for_next_live_input_test=(comparisons > 0 and equal == comparisons
                                                         and valid["cpu-sparse"] >= 2),
                unchanged_limits=dict(min_samples=4, min_span_s=.25, max_gap_s=.6,
                                      max_step_speed_mps=6., max_estimated_speed_mps=4., ema_alpha=.4),
                experimental_window_sim_s=1.4)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--vision", type=Path,
                        default=Path.home()/"carrierbot-vm.q09FUG/Robot_Vision_Training")
    args = parser.parse_args()
    paths = [args.run/"tracks.jsonl", args.run/"summary.json"]
    before = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    original, history = load_reviewed(args.vision)
    def packets():
        with paths[0].open(encoding="utf-8") as stream:
            for line in stream:
                if line.strip():
                    row = json.loads(line)
                    yield row.get("packet", row)
    result = compare(packets(), original, history)
    result.update(read_only=True, run=args.run.name, no_ros_or_actuation=True,
                  original_files_unchanged=all(hashlib.sha256(p.read_bytes()).hexdigest() == before[p.name]
                                               for p in paths),
                  scope="Estimator replay only. Does not evaluate live freshness, fusion, velocity accuracy or navigation; original scores unchanged.")
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
