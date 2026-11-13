#!/usr/bin/env python3
"""Read an existing trace only. Never change the original score or command ROS."""
import argparse
import json
import math
from pathlib import Path


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def inspect(rows):
    windows, current = [], None
    requested_images, scored_images, simultaneous_images = set(), set(), set()
    effective_samples, simultaneous_samples, examples = 0, 0, []
    for row in rows:
        now = row["sim"]
        decision = row.get("decision", {})
        slow = decision.get("action") == "SLOW"
        if slow:
            if current is None:
                current = {"start_sim": now, "last_sample_sim": now, "samples": 0,
                           "samples_eligible_for_original_0_4s_rule": 0}
            current["last_sample_sim"] = now
            current["samples"] += 1
        elif current is not None:
            current.update(end_sim=now, duration_sim_s=now-current["start_sim"])
            windows.append(current)
            current = None
        route = row.get("route", {})
        gate = route.get("behavior_gate", {})
        upstream = route.get("command_preview", {}).get("linear_x")
        command = row.get("guard_command", [])
        output = command[0] if command else None
        cap = gate.get("speed_cap_mps")
        gate_slow = (gate.get("action") == 1 and finite(cap) and 0 < cap <= .061)
        effective_samples += int(gate_slow)
        images = {item["image_stamp"] for item in decision.get("vision_observations", [])
                  if item.get("within_caution_corridor") is True and finite(item.get("image_stamp"))
                  and 0 <= now-item["image_stamp"] <= .5}
        if slow:
            requested_images.update(images)
            eligible = now-current["start_sim"] >= .4
            current["samples_eligible_for_original_0_4s_rule"] += int(eligible)
            if eligible and finite(output) and output > .01:
                scored_images.update(images)
            measured = (gate_slow and finite(upstream) and .01 < upstream <= .061 and
                        finite(output) and .01 < output <= .061 and finite(route.get("stamp"))
                        and -.02 <= now-route["stamp"] <= .3)
            if measured:
                simultaneous_samples += 1
                simultaneous_images.update(images)
                if len(examples) < 6:
                    examples.append({"sim": now, "vision_image_stamps": sorted(images),
                                     "gate_action": gate["action"], "gate_cap_mps": cap,
                                     "fablab_command_mps": upstream, "guard_command_mps": output,
                                     "directive_sequence": gate.get("directive_sequence")})
    if current is not None:
        current.update(end_sim=current["last_sample_sim"],
                       duration_sim_s=current["last_sample_sim"]-current["start_sim"])
        windows.append(current)
    return {"read_only": True, "rows": len(rows), "original_summary_not_changed": True,
            "slow_windows": windows, "requested_slow_unique_images": len(requested_images),
            "original_rule_unique_images_reconstructed": len(scored_images),
            "effective_gate_slow_samples": effective_samples,
            "simultaneous_slow_and_capped_moving_command_samples": simultaneous_samples,
            "simultaneous_unique_vision_images": len(simultaneous_images), "examples": examples,
            "scope": "Recorded policy/gate/command samples; no measured motor stop latency or automatic score override."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path, help="Existing fusion_yield run directory or trace.jsonl")
    args = parser.parse_args()
    path = args.run/"trace.jsonl" if args.run.is_dir() else args.run
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    print(json.dumps(inspect(rows), indent=2))


if __name__ == "__main__":
    main()
