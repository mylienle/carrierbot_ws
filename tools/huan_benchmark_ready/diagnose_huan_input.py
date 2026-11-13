"""Read recorded packets only: status, cadence and velocity-window feasibility."""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import statistics


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def distribution(values):
    values = sorted(v for v in values if finite(v))
    if not values:
        return None
    return dict(n=len(values), minimum=values[0], median=statistics.median(values),
                p95=values[min(len(values)-1, math.ceil(.95*len(values))-1)], maximum=values[-1])


def diagnose(directory):
    directory = Path(directory)
    summary = json.loads((directory/"summary.json").read_text(encoding="utf-8"))
    packets = []
    with (directory/"tracks.jsonl").open(encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                row = json.loads(line)
                packets.append(row.get("packet", row))
    velocities, poses, references = Counter(), Counter(), Counter()
    timestamps, publications = [], []
    histories = defaultdict(list)
    feasible = {str(w): 0 for w in (.8, 1.2, 1.4, 1.6)}
    contiguous_positions = 0
    for packet in packets:
        t = packet.get("timestamp")
        if finite(t):
            timestamps.append(t)
        perf = packet.get("performance") or {}
        if finite(perf.get("publish_wall_monotonic")):
            publications.append(perf["publish_wall_monotonic"])
        seen = set()
        for track in packet.get("tracks", []):
            ident = track.get("id")
            seen.add(ident)
            velocities[str(track.get("velocity_odom_status", "MISSING"))] += 1
            poses[str(track.get("pose_status", "MISSING"))] += 1
            references[str(track.get("reference_status", "MISSING"))] += 1
            p = track.get("xyz_odom_m")
            good = (finite(t) and isinstance(p, list) and len(p) == 3 and all(finite(x) for x in p)
                    and track.get("reference_valid") is True and track.get("odom_frame_id") == "odom"
                    and track.get("odom_source") == "tf2_ekf")
            h = histories[ident]
            if not good:
                histories[ident] = []
                continue
            if h:
                dt = t-h[-1][0]
                if dt <= 0 or dt > .6 or math.dist(p, h[-1][1]) > 6.*dt:
                    h = []
                    histories[ident] = h
            h.append((t, p))
            histories[ident] = h = [x for x in h if t-x[0] <= 1.6+1e-9]
            contiguous_positions += 1
            for window in (.8, 1.2, 1.4, 1.6):
                samples = [x for x in h if t-x[0] <= window+1e-9]
                if len(samples) >= 4 and samples[-1][0]-samples[0][0] >= .25:
                    feasible[str(window)] += 1
        # Missing detections break a measured chain; do not bridge unseen frames.
        for ident in list(histories):
            if ident not in seen:
                histories[ident] = []
    image_gaps = [b-a for a, b in zip(timestamps, timestamps[1:]) if b > a]
    wall_gaps = [b-a for a, b in zip(publications, publications[1:]) if b > a]
    performance = {}
    for key in ("processing_ms", "image_age_sim_ms", "rgbd_receive_to_publish_ms"):
        performance[key] = distribution((p.get("performance") or {}).get(key) for p in packets)
    runtime = Counter(json.dumps(p.get("runtime_cpu"), sort_keys=True) for p in packets)
    return dict(read_only=True, run=directory.name, goal_sent_count=summary.get("goal_sent_count"),
                packets=len(packets), velocity_status_counts=dict(velocities),
                pose_status_counts=dict(poses), reference_status_counts=dict(references),
                image_gap_sim_s=distribution(image_gaps), publication_gap_wall_s=distribution(wall_gaps),
                valid_position_records_for_cadence_check=contiguous_positions,
                frames_with_four_contiguous_positions_by_window_sim_s=feasible,
                performance=performance, runtime_cpu_packet_counts=dict(runtime),
                note="Window counts check sample availability only. They do not calculate or mark any new velocity valid, and do not change original scores.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    print(json.dumps(diagnose(args.run), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
