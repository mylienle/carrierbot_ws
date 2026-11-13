#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
bash "$SCRIPT_DIR/install_local_tools.sh"
bash "$HOME/carrierbot-huan-tools/build_behavior_gate.sh"
exec bash "$HOME/carrierbot-huan-tools/start_lien_test.sh" \
  --recovery --route-preview --behavior-gate --speed 0.12 --sim-rate 0.35
