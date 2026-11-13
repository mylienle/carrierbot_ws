#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LOCAL_TOOLS="$HOME/carrierbot-huan-benchmark"
if [ "$SCRIPT_DIR" != "$LOCAL_TOOLS" ]; then
  bash "$SCRIPT_DIR/install_local_tools.sh"
fi
source /opt/ros/foxy/setup.bash
python3 "$LOCAL_TOOLS/check_huan_install.py"
if ! python3 "$LOCAL_TOOLS/ensure_huan_gate.py"; then
  bash "$LOCAL_TOOLS/build_behavior_gate.sh"
fi
printf '\nTHU DOI CHUNG HUAN: kich ban dong bo; Fablab va safety_guard.\n'
printf 'Khong chay cung phien mo phong cu. Giu terminal nay mo.\n'
exec bash "$LOCAL_TOOLS/start_lien_test.sh" \
  --recovery --route-preview --behavior-gate --sensor-fusion --speed 0.12 --sim-rate 0.35 "$@"
