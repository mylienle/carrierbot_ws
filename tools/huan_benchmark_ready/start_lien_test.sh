#!/usr/bin/env bash
set -e

source /opt/ros/foxy/setup.bash
TRIAL="$HOME/carrierbot-lien-foxy.0bkXPj"
USE_RECOVERY=0
USE_ROUTE=0
USE_GATE=0
for option in "$@"; do
  if [ "$option" = "--recovery" ]; then
    USE_RECOVERY=1
  elif [ "$option" = "--route-preview" ]; then
    USE_ROUTE=1
  elif [ "$option" = "--behavior-gate" ]; then
    USE_GATE=1
  fi
done
if [ "$USE_GATE" -eq 1 ]; then
    IFS= read -r TRIAL < "$HOME/carrierbot_test_runs/behavior_huan_workspace.txt"
    if [[ "$TRIAL" != "$HOME"/carrierbot-lien-gate.* ]]; then
      printf 'Invalid behavior workspace: %s\n' "$TRIAL" >&2
      exit 1
    fi
elif [ "$USE_ROUTE" -eq 1 ]; then
    IFS= read -r TRIAL < "$HOME/carrierbot_test_runs/route_workspace.txt"
    if [[ "$TRIAL" != "$HOME"/carrierbot-lien-route.* ]]; then
      printf 'Invalid route workspace: %s\n' "$TRIAL" >&2
      exit 1
    fi
elif [ "$USE_RECOVERY" -eq 1 ]; then
    IFS= read -r TRIAL < "$HOME/carrierbot_test_runs/recovery_workspace.txt"
    if [[ "$TRIAL" != "$HOME"/carrierbot-lien-recovery.* ]]; then
      printf 'Invalid recovery workspace: %s\n' "$TRIAL" >&2
      exit 1
    fi
fi
source "$TRIAL/install/local_setup.bash"
export ROS_DOMAIN_ID=39 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11349

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$SCRIPT_DIR/start_lien_test.py" "$@"
