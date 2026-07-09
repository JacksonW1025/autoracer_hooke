#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  rc_stop.sh

Stops RC Autoware, sensor, localization, planning, control, vehicle-interface,
RViz, and rosbag processes started by the formal RC scripts.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ $# -gt 0 ]]; then
  echo "ERROR: unknown argument: $1" >&2
  usage >&2
  exit 1
fi

patterns=(
  "[r]os2 launch autoware_launch autoware.launch.xml"
  "[r]un_official_autoware.sh"
  "[t]opic_tools/relay"
  "[r]viz2"
  "[r]obot_state_publisher"
  "[s]tatic_transform_publisher"
  "[c]omponent_container"
  "[c]omponent_container_mt"
  "[p]ointcloud_container"
  "[l]slidar_driver_node"
  "[p]ointcloud_voxel_filter"
  "[h]ipnuc_imu/lib/hipnuc_imu/talker"
  "[I]MU_publisher"
  "[i]mu_filter_madgwick"
  "[f]ixposition_seed_filter"
  "[m]anual_seed_pose_publisher"
  "[n]dt_initial_pose_predictor"
  "[n]dt_startup_helper"
  "[k]inematic_state_publisher"
  "[a]utoware_ndt_scan_matcher_node"
  "[a]utoware_pointcloud_map_loader"
  "[a]utoware_lanelet2_map_loader"
  "[a]utoware_map_projection_loader_node"
  "[l]anelet_route_planner"
  "[p]ure_pursuit_controller"
  "[c]ommand_gate"
  "[r]c_serial_interface"
  "[r]os2 bag record"
)

for pattern in "${patterns[@]}"; do
  pkill -TERM -f "$pattern" 2>/dev/null || true
done

sleep "${STOP_WAIT_SEC:-1}"

ps -eo pid,comm,args |
  grep -E "component_container|topic_tools/relay|lslidar|pointcloud|hipnuc|IMU_publisher|imu_filter|run_official_autoware|autoware.launch|ndt|map_loader|lanelet|pure_pursuit|command_gate|rc_serial|rviz2|robot_state|rosbag" |
  grep -v grep || true
