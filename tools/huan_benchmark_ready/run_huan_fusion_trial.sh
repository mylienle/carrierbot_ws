#!/usr/bin/env bash
set -e
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=39 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11349
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python3 "$SCRIPT_DIR/check_huan_install.py"
exec /usr/bin/python3 -u "$SCRIPT_DIR/fusion_yield_trial.py" "$@"
