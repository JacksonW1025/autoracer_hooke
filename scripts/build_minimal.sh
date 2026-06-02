#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

AUTORACER_SOURCE_LOCAL_SETUP=false
# shellcheck source=scripts/ros_env.sh
source "$ROOT_DIR/scripts/ros_env.sh"

PACKAGES=(
  autoracer_description
  autoracer_localization
  autoracer_sensing
  autoracer_planning
  autoracer_control
  autoracer_carmaker_sim
  autoracer_safety
  autoracer_bringup
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
  autoware_interpolation
  autoware_kalman_filter
  autoware_motion_utils
  autoware_osqp_interface
  autoware_point_types
  autoware_ekf_localizer
  managed_transform_buffer
  autoware_pcl_extensions
  autoware_pointcloud_preprocessor
  autoware_trajectory_follower_base
  autoware_mpc_lateral_controller
  autoware_pid_longitudinal_controller
  autoware_pure_pursuit
  autoware_trajectory_follower_node
  autoware_map_projection_loader
  autoware_map_loader
  autoware_ndt_scan_matcher
  autoware_gnss_poser
  nebula_msgs
  nebula_hesai
  nebula_hesai_decoders
  wd_byte
  hooke2_msgs
  can_driver
  fixposition_driver_msgs
  fixposition_driver_lib
  rtcm_msgs
  fpsdk_common
  fpsdk_ros2
  fixposition_driver_ros2
  hooke2_description
  hooke2_launch
  hooke2_interface
)

UNDERLAY_OVERRIDES=(
  autoware_adapi_v1_msgs
  autoware_internal_planning_msgs
  autoware_lanelet2_extension
  autoware_map_msgs
  autoware_perception_msgs
  autoware_planning_msgs
  autoware_utils_geometry
)

# Keep vendored Autoware headers ahead of ROS apt underlay headers when both exist.
CMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
CMAKE_CXX_FLAGS="${CMAKE_CXX_FLAGS:-} -I${ROOT_DIR}/install/autoware_lanelet2_extension/include"

colcon build \
  --symlink-install \
  --allow-overriding "${UNDERLAY_OVERRIDES[@]}" \
  --packages-up-to "${PACKAGES[@]}" \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}" \
    -DCMAKE_CXX_FLAGS="${CMAKE_CXX_FLAGS}"
