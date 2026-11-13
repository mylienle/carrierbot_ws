#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [ "$SCRIPT_DIR" != "$HOME/carrierbot-huan-tools" ]; then
  bash "$SCRIPT_DIR/install_local_tools.sh"
fi
if [ ! -f "$HOME/carrierbot_test_runs/behavior_huan_workspace.txt" ]; then
  bash "$HOME/carrierbot-huan-tools/build_behavior_gate.sh"
fi
exec bash "$HOME/carrierbot-huan-tools/start_lien_test.sh" \
  --recovery --route-preview --behavior-gate --sensor-fusion --speed 0.12 --sim-rate 0.35
