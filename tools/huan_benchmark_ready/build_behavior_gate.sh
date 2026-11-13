#!/usr/bin/env bash
set -e
source /opt/ros/foxy/setup.bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BASE="$HOME/carrierbot-lien-foxy.0bkXPj/repo/src/carrierbot_navigation"
test -f "$BASE/src/fablab_guidance_node.cpp"

python3 - "$BASE" "$SCRIPT_DIR/recovery_source/source_manifest.json" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

base = Path(sys.argv[1])
manifest = json.loads(Path(sys.argv[2]).read_text())
for relative, expected in manifest.items():
    data = (base / relative).read_bytes().replace(b"\r\n", b"\n")
    if hashlib.sha256(data).hexdigest() != expected:
        raise SystemExit("Source differs from the reviewed version: " + relative)
print("OK: original source matches the reviewed branch")
PY

RECOVERY_WS="$(mktemp -d "$HOME/carrierbot-lien-gate.XXXXXX")"
mkdir -p "$RECOVERY_WS/src"
cp -a "$BASE" "$RECOVERY_WS/src/carrierbot_navigation"
cp "$SCRIPT_DIR/behavior_gate_source/fablab_guidance_node.cpp" \
   "$RECOVERY_WS/src/carrierbot_navigation/src/fablab_guidance_node.cpp"
cp "$SCRIPT_DIR/recovery_source/avoidance_recovery.hpp" \
   "$RECOVERY_WS/src/carrierbot_navigation/src/avoidance_recovery.hpp"

cp "$SCRIPT_DIR/behavior_gate_source/route_preview.hpp" \
   "$RECOVERY_WS/src/carrierbot_navigation/src/route_preview.hpp"
cp "$SCRIPT_DIR/behavior_gate_source/behavior_gate.hpp" \
   "$RECOVERY_WS/src/carrierbot_navigation/src/behavior_gate.hpp"
c++ -std=c++14 -O2 -Wall -Wextra \
  -I "$SCRIPT_DIR/behavior_gate_source" \
  "$SCRIPT_DIR/behavior_gate_source/test_behavior_gate.cpp" -o "$RECOVERY_WS/test_behavior_gate"
"$RECOVERY_WS/test_behavior_gate"

c++ -std=c++14 -O2 -Wall -Wextra \
  -I "$SCRIPT_DIR/behavior_gate_source" \
  "$SCRIPT_DIR/route_preview_source/test_route_preview.cpp" -o "$RECOVERY_WS/test_route_preview"
"$RECOVERY_WS/test_route_preview" > "$RECOVERY_WS/route_serialization.jsonl"
python3 - "$RECOVERY_WS/route_serialization.jsonl" <<'PYTEST'
import json, sys
from pathlib import Path
rows = [json.loads(line) for line in Path(sys.argv[1]).read_text().splitlines()]
assert rows[0]["active"] and rows[0]["current_segment"] == 1
assert rows[0]["waypoints_map"] == [[0, 0], [1, 0], [1, 1], [5, 1]]
assert rows[0]["diagnostic_only"] and not rows[1]["active"]
print("OK: route serialization and non-finite rejection")
PYTEST

c++ -std=c++14 -O2 -Wall -Wextra \
  -I "$RECOVERY_WS/src/carrierbot_navigation/src" \
  "$SCRIPT_DIR/recovery_source/test_recovery.cpp" -o "$RECOVERY_WS/test_recovery"
"$RECOVERY_WS/test_recovery"

cd "$RECOVERY_WS"
colcon build --base-paths src --packages-select carrierbot_navigation \
  --parallel-workers 1 --cmake-args -DBUILD_TESTING=OFF
source "$RECOVERY_WS/install/local_setup.bash"
ros2 pkg prefix carrierbot_navigation
ros2 pkg executables carrierbot_navigation | grep fablab_guidance_node

python3 - "$SCRIPT_DIR" "$RECOVERY_WS" <<'PYBUILD'
import hashlib, json, sys
from pathlib import Path
tools, workspace = map(Path, sys.argv[1:])
names = ("behavior_gate_source/fablab_guidance_node.cpp", "behavior_gate_source/route_preview.hpp",
         "behavior_gate_source/behavior_gate.hpp",
         "recovery_source/avoidance_recovery.hpp")
manifest = {name: hashlib.sha256((tools/name).read_bytes()).hexdigest() for name in names}
(workspace/"gate_build_manifest.json").write_text(json.dumps(manifest, indent=2))
PYBUILD

mkdir -p "$HOME/carrierbot_test_runs"
POINTER="$(mktemp "$HOME/carrierbot_test_runs/behavior_huan_workspace.XXXXXX")"
printf '%s\n' "$RECOVERY_WS" > "$POINTER"
mv -f "$POINTER" "$HOME/carrierbot_test_runs/behavior_huan_workspace.txt"
printf '\nBEHAVIOR GATE BUILD OK: %s\n' "$RECOVERY_WS"
