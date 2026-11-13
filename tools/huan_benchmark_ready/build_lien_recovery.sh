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

RECOVERY_WS="$(mktemp -d "$HOME/carrierbot-lien-recovery.XXXXXX")"
mkdir -p "$RECOVERY_WS/src"
cp -a "$BASE" "$RECOVERY_WS/src/carrierbot_navigation"
cp "$SCRIPT_DIR/recovery_source/fablab_guidance_node.cpp" \
   "$RECOVERY_WS/src/carrierbot_navigation/src/fablab_guidance_node.cpp"
cp "$SCRIPT_DIR/recovery_source/avoidance_recovery.hpp" \
   "$RECOVERY_WS/src/carrierbot_navigation/src/avoidance_recovery.hpp"

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

mkdir -p "$HOME/carrierbot_test_runs"
POINTER="$(mktemp "$HOME/carrierbot_test_runs/recovery_workspace.XXXXXX")"
printf '%s\n' "$RECOVERY_WS" > "$POINTER"
mv -f "$POINTER" "$HOME/carrierbot_test_runs/recovery_workspace.txt"
printf '\nRECOVERY BUILD OK: %s\n' "$RECOVERY_WS"
