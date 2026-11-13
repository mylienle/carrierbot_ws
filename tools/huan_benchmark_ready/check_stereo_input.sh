#!/usr/bin/env bash
set -e
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=39 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11349
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$SCRIPT_DIR/check_stereo_input.py" "$@"
