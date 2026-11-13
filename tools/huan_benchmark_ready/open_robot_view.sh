#!/usr/bin/env bash
# Open an independent client; do not stop or restart the supervised simulation.
set -euo pipefail

MODE="software"
if [ "$#" -eq 1 ] && [ "$1" = "--normal" ]; then
  MODE="normal"
elif [ "$#" -eq 1 ] && [ "$1" = "--gazebo-scene" ]; then
  : # Kept for compatibility; the verified Gazebo scene route is now default.
elif [ "$#" -ne 0 ]; then
  printf 'Usage: bash open_robot_view.sh [--normal|--gazebo-scene]\n' >&2
  exit 2
fi
if ! command -v gzclient >/dev/null 2>&1; then
  printf 'Khong tim thay gzclient. Chay lenh nay trong Ubuntu co Gazebo.\n' >&2
  exit 1
fi

mkdir -p "$HOME/carrierbot_test_runs"
VIEW_LOG="$(mktemp -d "$HOME/carrierbot_test_runs/robot_view_${MODE}_XXXXXX")"
printf 'DANG MO THEM CUA SO GAZEBO: %s\n' "$MODE"
printf 'Giu terminal start_lien_test va cua so Gazebo cu dang mo.\n'
printf 'Chi xem mo phong dang chay tai http://127.0.0.1:11349.\n'
printf 'LOG: %s/gzclient.log\n' "$VIEW_LOG"

# Independent viewers must not discover a different server's /scene_info.
# The supervised startup instead gives its server and client one shared,
# per-run partition, which also keeps their rendering services connected.
VIEW_PARTITION="carrierbot_view_$$_$(date +%s)"
printf 'NGUON CANH: Gazebo master 11349 qua co che fallback san co.\n'
printf 'IGN_PARTITION chi cua GUI moi: %s\n' "$VIEW_PARTITION"
if [ "$MODE" = "software" ]; then
  env GAZEBO_MASTER_URI=http://127.0.0.1:11349 LIBGL_ALWAYS_SOFTWARE=true \
    IGN_PARTITION="$VIEW_PARTITION" \
    gzclient --verbose 2>&1 | tee "$VIEW_LOG/gzclient.log"
else
  env -u LIBGL_ALWAYS_SOFTWARE GAZEBO_MASTER_URI=http://127.0.0.1:11349 \
    IGN_PARTITION="$VIEW_PARTITION" \
    gzclient --verbose 2>&1 | tee "$VIEW_LOG/gzclient.log"
fi
