#!/usr/bin/env bash
set -e
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=39 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11349
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VISION="$HOME/carrierbot-vm.q09FUG/Robot_Vision_Training"
cd "$VISION"
exec "$VISION/.venv-vm/bin/python" -u "$SCRIPT_DIR/diagnose_stereo_pose.py"
