#!/usr/bin/env bash
set -euo pipefail

# This workspace intentionally uses system ROS packages for non-research modules.
# Source copies under src/navigation2, src/rplidar_ros, and src/slam_toolbox have
# COLCON_IGNORE markers and are retained only as a reversible fallback.

source /opt/ros/foxy/setup.bash

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

required_packages=(
  nav2_amcl
  nav2_behavior_tree
  nav2_bt_navigator
  nav2_controller
  nav2_core
  nav2_costmap_2d
  nav2_lifecycle_manager
  nav2_map_server
  nav2_planner
  nav2_recoveries
  rplidar_ros
  slam_toolbox
  smac_planner
)

missing_packages=()
for package_name in "${required_packages[@]}"; do
  if ros2 pkg prefix "${package_name}" >/dev/null 2>&1; then
    printf 'OK      %s\n' "${package_name}"
  else
    printf 'MISSING %s\n' "${package_name}"
    missing_packages+=("${package_name}")
  fi
done

if ((${#missing_packages[@]} > 0)); then
  cat <<'EOF'

Install the missing vendor packages before building this workspace:

  sudo apt update
  sudo apt install ros-foxy-navigation2 ros-foxy-rplidar-ros ros-foxy-slam-toolbox

The custom research code does not come from apt:
  carrierbot_navigation  (Adaptive LOS+PD)
  carrierbot_firmware    (CAN hardware interface)
  carrierbot_mqtt        (MQTT integration)
EOF
  exit 1
fi

stale_overlay_packages=()
for package_name in "${required_packages[@]}"; do
  if [[ -e "${workspace_root}/install/${package_name}" ]] || \
     [[ -e "${workspace_root}/install/share/ament_index/resource_index/packages/${package_name}" ]]; then
    stale_overlay_packages+=("${package_name}")
  fi
done

if ((${#stale_overlay_packages[@]} > 0)); then
  cat <<EOF

WARNING: this workspace still has source-built vendor packages in install/:
  ${stale_overlay_packages[*]}

They would override /opt/ros/foxy after 'source install/setup.bash'. Move the
old build artifacts aside, then rebuild only the project packages:

  cd "${workspace_root}"
  mv build build.before-system-packages
  mv install install.before-system-packages
  mv log log.before-system-packages
  colcon build --symlink-install

The move is recoverable; it does not remove source code or maps.
EOF
  exit 2
fi

echo 'All required system ROS packages are available.'
