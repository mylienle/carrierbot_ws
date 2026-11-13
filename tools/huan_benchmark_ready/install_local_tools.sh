#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LOCAL_TOOLS="$HOME/carrierbot-huan-benchmark"

for file in start_huan_fusion.sh run_huan_fusion_trial.sh huan_fusion_bridge.py huan_fusion_policy.py huan_cpu_metadata.py check_huan_install.py analyze_huan_run.py huan_vision_core.json integration_manifest.json huan_fusion_vendor/manifest.json; do
  if [ ! -f "$SCRIPT_DIR/$file" ]; then
    printf 'Missing Huan integration file: %s\n' "$file" >&2
    exit 1
  fi
done

for file in start_lien_test.sh start_lien_test.py run_lien_baseline.py check_stereo_input.sh carrierbot_roi_pose.py person_fixture.py pair_person_lidar.py sensor_pair_geometry.py sensor_alignment_observer.py collect_pair_diagnostics.sh collect_pair_diagnostics.py open_robot_view.sh gazebo_session.py fablab_route.py build_lien_route_preview.sh start_route_shadow.sh; do
  if [ ! -f "$SCRIPT_DIR/$file" ]; then
    printf 'Missing tool file: %s\n' "$SCRIPT_DIR/$file" >&2
    exit 1
  fi
done
for file in build_behavior_gate.sh start_behavior_gate.sh run_behavior_gate_trial.sh behavior_gate_trial.py; do
  if [ ! -f "$SCRIPT_DIR/$file" ]; then
    printf 'Missing motion gate tool: %s\n' "$file" >&2
    exit 1
  fi
done
for file in start_fusion_yield.sh run_fusion_yield_trial.sh fusion_yield_trial.py fusion_yield_policy.py fusion_sensor_adapter.py fusion_fixture.py; do
  if [ ! -f "$SCRIPT_DIR/$file" ]; then
    printf 'Missing sensor trial tool: %s\n' "$file" >&2
    exit 1
  fi
done
for file in fablab_guidance_node.cpp behavior_gate.hpp route_preview.hpp test_behavior_gate.cpp; do
  if [ ! -f "$SCRIPT_DIR/behavior_gate_source/$file" ]; then
    printf 'Missing motion gate source: %s\n' "$file" >&2
    exit 1
  fi
done
for file in fablab_guidance_node.cpp route_preview.hpp test_route_preview.cpp; do
  if [ ! -f "$SCRIPT_DIR/route_preview_source/$file" ]; then
    printf 'Missing internal route source: %s\n' "$file" >&2
    exit 1
  fi
done
for file in dynamic_decision_policy.py decision_shadow_logger.py huan_dynamic_obstacles.py huan_risk_source.json; do
  if [ ! -f "$SCRIPT_DIR/$file" ]; then
    printf 'Missing decision tool: %s\n' "$SCRIPT_DIR/$file" >&2
    exit 1
  fi
done
if [ "$SCRIPT_DIR" = "$LOCAL_TOOLS" ]; then
  printf 'Day la ban local. De cap nhat tu Windows, chay:\n' >&2
  printf '  bash /mnt/carrierbot-share/huan_benchmark_ready/install_local_tools.sh\n' >&2
  exit 1
fi
mkdir -p "$LOCAL_TOOLS"
cp -r "$SCRIPT_DIR"/. "$LOCAL_TOOLS"/

python3 - "$SCRIPT_DIR" "$LOCAL_TOOLS" <<'PY'
import ast
import hashlib
import json
from pathlib import Path
import sys

source, target = map(Path, sys.argv[1:])
for name in ("start_lien_test.sh", "start_lien_test.py", "run_lien_baseline.py", "check_stereo_input.sh", "check_stereo_input.py", "carrierbot_roi_pose.py", "person_fixture.py", "pair_person_lidar.py", "sensor_pair_geometry.py", "sensor_alignment_observer.py", "collect_pair_diagnostics.sh", "collect_pair_diagnostics.py", "open_robot_view.sh", "gazebo_session.py", "fablab_route.py", "build_lien_route_preview.sh", "start_route_shadow.sh", "route_preview_source/fablab_guidance_node.cpp", "route_preview_source/route_preview.hpp", "route_preview_source/test_route_preview.cpp"):
    if hashlib.sha256((source / name).read_bytes()).digest() != hashlib.sha256((target / name).read_bytes()).digest():
        raise SystemExit("File copied incorrectly: " + name)
for name in ("dynamic_decision_policy.py", "decision_shadow_logger.py", "huan_dynamic_obstacles.py", "huan_risk_source.json"):
    if (source / name).read_bytes() != (target / name).read_bytes():
        raise SystemExit("Decision file copied incorrectly: " + name)
for name in ("build_behavior_gate.sh", "start_behavior_gate.sh", "run_behavior_gate_trial.sh", "behavior_gate_trial.py",
             "behavior_gate_source/fablab_guidance_node.cpp", "behavior_gate_source/route_preview.hpp",
             "behavior_gate_source/behavior_gate.hpp", "behavior_gate_source/test_behavior_gate.cpp"):
    if (source / name).read_bytes() != (target / name).read_bytes():
        raise SystemExit("Motion gate file copied incorrectly: " + name)
manifest = json.loads((target / "huan_risk_source.json").read_text(encoding="utf-8"))
for name in ("start_fusion_yield.sh", "run_fusion_yield_trial.sh", "fusion_yield_trial.py",
             "fusion_yield_policy.py", "fusion_sensor_adapter.py", "fusion_fixture.py"):
    if (source / name).read_bytes() != (target / name).read_bytes():
        raise SystemExit("Sensor trial file copied incorrectly: " + name)
if hashlib.sha256((target / "huan_dynamic_obstacles.py").read_bytes()).hexdigest() != manifest["sha256"]:
    raise SystemExit("Huan risk source does not match its provenance manifest")
for name, flag in (("start_lien_test.py", "--sim-rate"), ("check_stereo_input.py", "--roi-pose"), ("start_lien_test.py", "--paired-person"), ("check_stereo_input.py", "--paired-person"), ("check_stereo_input.py", "--decision-log"), ("start_lien_test.py", "--route-preview"), ("check_stereo_input.py", "--fablab-route"), ("start_lien_test.py", "--behavior-gate"), ("start_lien_test.py", "--sensor-fusion")):
    tree = ast.parse((target / name).read_text(encoding="utf-8"))
    flags = {arg.value for node in ast.walk(tree)
             if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
             and node.func.attr == "add_argument"
             for arg in node.args if isinstance(arg, ast.Constant) and isinstance(arg.value, str)}
    if flag not in flags:
        raise SystemExit("The shared source is outdated. Missing " + flag + " in " + name)
    print("OK local: " + name + " supports " + flag)
PY
python3 "$LOCAL_TOOLS/check_huan_install.py"
printf '\nDA CHEP BAN DOI CHUNG VAO: %s\n' "$LOCAL_TOOLS"
printf 'Terminal 1: bash /mnt/carrierbot-share/huan_benchmark_ready/start_huan_fusion.sh --yield-clearance 0.6 --early-rejoin --final-approach-ramp\n'
printf 'Terminal 2, sau SAN SANG: bash ~/carrierbot-huan-benchmark/run_huan_fusion_trial.sh --mode huan --velocity-profile cpu-sparse --vision-threads 1 --reliable-rgbd\n'
