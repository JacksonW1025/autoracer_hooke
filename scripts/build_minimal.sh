#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

AUTORACER_SOURCE_LOCAL_SETUP=false
# shellcheck source=scripts/ros_env.sh
source "$ROOT_DIR/scripts/ros_env.sh"

COLCON_WORKER_ARGS=()
if [[ -n "${COLCON_PARALLEL_WORKERS:-}" ]]; then
  COLCON_WORKER_ARGS+=(--parallel-workers "$COLCON_PARALLEL_WORKERS")
fi

PACKAGES=(
  autoracer_description
  autoracer_localization
  autoracer_sensing
  autoracer_planning
  autoracer_control
  autoracer_safety
  autoracer_vehicle_interface
  autoracer_hooke_description
  autoracer_hooke_launch
  autoracer_hooke_sensor_kit_description
  autoracer_hooke_sensor_kit_launch
  hipnuc_imu
  lslidar_msgs
  lslidar_driver
  autoware_launch
  tier4_vehicle_launch
  tier4_sensing_launch
  autoware_adapi_v1_msgs
  autoware_component_interface_specs
  autoware_control_msgs
  autoware_geography_utils
  autoware_internal_debug_msgs
  autoware_internal_localization_msgs
  autoware_lanelet2_extension
  autoware_lanelet2_utils
  autoware_localization_util
  autoware_map_msgs
  autoware_planning_msgs
  autoware_qos_utils
  autoware_sensing_msgs
  autoware_vehicle_msgs
  autoware_agnocast_wrapper
  tier4_api_msgs
  tier4_debug_msgs
  tier4_external_api_msgs
  tier4_vehicle_msgs
  tier4_api_utils
  autoware_vehicle_info_utils
  autoware_map_projection_loader
  autoware_map_loader
  autoware_ndt_scan_matcher
  autoware_gnss_poser
  nebula_msgs
  nebula_hesai
  nebula_hesai_decoders
  autoware_localization_rviz_plugin
  autoware_planning_rviz_plugin
  tier4_control_mode_rviz_plugin
  tier4_state_rviz_plugin
  tier4_vehicle_rviz_plugin
  tier4_planning_factor_rviz_plugin
  fixposition_driver_msgs
  fixposition_driver_lib
  rtcm_msgs
  fpsdk_common
  fpsdk_ros2
  fixposition_driver_ros2
)

OVERRIDE_PACKAGES=(
  autoware_adapi_v1_msgs
  autoware_internal_planning_msgs
  autoware_lanelet2_extension
  autoware_map_msgs
  autoware_perception_msgs
  autoware_planning_msgs
  autoware_utils_geometry
  autoware_utils_math
  autoware_utils_system
  autoware_utils_visualization
  autoware_vehicle_msgs
)

colcon build --symlink-install \
  "${COLCON_WORKER_ARGS[@]}" \
  --packages-up-to "${PACKAGES[@]}" \
  --allow-overriding "${OVERRIDE_PACKAGES[@]}" \
  --cmake-args -DBUILD_TESTING=OFF
