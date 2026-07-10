#!/usr/bin/env bash
set -euo pipefail

TOPICS=(
  /sensing/lidar/concatenated/pointcloud
  /fixposition/fix
  /fixposition/rawimu
  /fixposition/autoware_orientation
  /sensing/gnss/pose_with_covariance
  /localization/pose_with_covariance
  /planning/mission_path
  /planning/trajectory
  /autoracer/control/raw_control_cmd
  /control/command/control_cmd
  /vehicle/status/velocity_status
  /vehicle/status/steering_status
  /vehicle/status/gear_status
  /hooke2/wheel_speed_rpt
  /hooke2/steering_rpt
)

for topic in "${TOPICS[@]}"; do
  if ros2 topic info "$topic" >/dev/null 2>&1; then
    echo "OK   $topic"
  else
    echo "MISS $topic"
  fi
done
