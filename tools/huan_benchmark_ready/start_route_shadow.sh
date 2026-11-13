#!/usr/bin/env bash
# First-time setup entry point, run from the Windows share after stopping the old session.
set -e
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
bash "$SCRIPT_DIR/install_local_tools.sh"
bash "$HOME/carrierbot-huan-tools/build_lien_route_preview.sh"
exec bash "$HOME/carrierbot-huan-tools/start_lien_test.sh" \
  --recovery --route-preview --speed 0.12 --sim-rate 0.35 --paired-person
