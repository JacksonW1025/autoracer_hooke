// Copyright 2024 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use node file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__NDT_SCAN_MATCHER__HYPER_PARAMETERS_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__HYPER_PARAMETERS_HPP_

#include "ndt_omp/multigrid_ndt_omp.h"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::ndt_scan_matcher
{

enum class ConvergedParamType {
  TRANSFORM_PROBABILITY = 0,
  NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD = 1
};

enum class CovarianceEstimationType {
  FIXED_VALUE = 0,
  LAPLACE_APPROXIMATION = 1,
  MULTI_NDT = 2,
  MULTI_NDT_SCORE = 3,
};

struct HyperParameters
{
  struct Frame
  {
    std::string base_frame{};
    std::string ndt_base_frame{};
    std::string map_frame{};
  } frame{};

  struct SensorPoints
  {
    double timeout_sec{};
    double required_distance{};
  } sensor_points{};

  pclomp::NdtParams ndt{};
  bool ndt_regularization_enable{};

  struct InitialPoseEstimation
  {
    int64_t particles_num{};
    int64_t n_startup_trials{};
    bool include_initial_pose{};
    bool force_initial_yaw{};
    bool output_initial_yaw{};
    bool use_sensor_points_stamp{};
    double sensor_points_stamp_tolerance_sec{};
    double sensor_points_stamp_wait_timeout_sec{};
    bool deterministic_offsets_enable{};
    std::vector<double> deterministic_offset_along_m{};
    std::vector<double> deterministic_offset_cross_m{};
    std::vector<double> deterministic_offset_yaw_deg{};
  } initial_pose_estimation{};

  struct Validation
  {
    double initial_pose_timeout_sec{};
    double initial_pose_distance_tolerance_m{};
    double initial_to_result_distance_tolerance_m{};
    double critical_upper_bound_exe_time_ms{};
    int64_t skipping_publish_num{};
  } validation{};

  struct ScoreEstimation
  {
    ConvergedParamType converged_param_type{};
    double converged_param_transform_probability{};
    double converged_param_nearest_voxel_transformation_likelihood{};
    struct NoGroundPoints
    {
      bool enable{};
      double z_margin_for_ground_removal{};
    } no_ground_points{};
  } score_estimation{};

  struct Covariance
  {
    std::array<double, 36> output_pose_covariance{};

    struct CovarianceEstimation
    {
      CovarianceEstimationType covariance_estimation_type{};
      std::vector<double> initial_pose_offset_model_x{};
      std::vector<double> initial_pose_offset_model_y{};
      double temperature{};
      double scale_factor{};
    } covariance_estimation{};
  } covariance{};

  struct DynamicMapLoading
  {
    double update_distance{};
    double map_radius{};
    double lidar_radius{};
  } dynamic_map_loading{};

  double output_pose_time_offset_sec{};

  struct RuntimeMultistart
  {
    bool enable{};
    std::string debug_topic{};
    double min_stamp_sec{};
    double trigger_initial_to_result_distance_m{};
    double trigger_yaw_delta_deg{};
    double trigger_score_margin{};
    std::vector<double> offset_along_m{};
    std::vector<double> offset_cross_m{};
    std::vector<double> offset_yaw_deg{};
    double min_transform_probability{};
    double min_nearest_voxel_transformation_likelihood{};
    double max_initial_to_result_distance_m{};
    double max_prior_innovation_m{};
    double max_prior_along_m{};
    double max_prior_cross_m{};
    double max_prior_yaw_deg{};
    double min_total_score{};
    double score_weight{};
    double transform_probability_weight{};
    double innovation_xy_penalty_weight{};
    double innovation_along_penalty_weight{};
    double innovation_cross_penalty_weight{};
    double innovation_yaw_penalty_weight{};
    double initial_to_result_penalty_weight{};
    double covariance_condition_penalty_weight{};
    double base_candidate_raw_score_margin{};
    double raw_score_override_margin{};
    double raw_score_override_max_total_score_drop{};
    double raw_score_override_max_abs_along_m{};
    double raw_score_override_max_abs_cross_m{};
    double raw_score_override_max_abs_yaw_deg{};
    double raw_score_override_max_initial_to_result_distance_m{};
    double tier1_max_abs_along_m{};
    double tier1_max_abs_cross_m{};
    double tier1_max_abs_yaw_deg{};
    double tracking_tier1_period_sec{};
    double tracking_along_health_period_sec{};
    double tracking_far_tier_period_sec{};
    double ambiguity_score_margin{};
    double ambiguity_along_spread_m{};
    int recovery_stable_required_frames{};
    double recovery_stable_max_innovation_m{};
    double recovery_stable_max_yaw_deg{};
    double recovery_far_tier_period_sec{};
    int recovery_far_tier_min_scan_interval{};
  } runtime_multistart{};

public:
  explicit HyperParameters(rclcpp::Node * node)
  {
    frame.base_frame = node->declare_parameter<std::string>("frame.base_frame");
    frame.ndt_base_frame = node->declare_parameter<std::string>("frame.ndt_base_frame");
    frame.map_frame = node->declare_parameter<std::string>("frame.map_frame");

    output_pose_time_offset_sec =
      node->declare_parameter<double>("output_pose_time_offset_sec", 0.0);

    sensor_points.timeout_sec = node->declare_parameter<double>("sensor_points.timeout_sec");
    sensor_points.required_distance =
      node->declare_parameter<double>("sensor_points.required_distance");

    ndt.trans_epsilon = node->declare_parameter<double>("ndt.trans_epsilon");
    ndt.step_size = node->declare_parameter<double>("ndt.step_size");
    ndt.resolution = node->declare_parameter<float>("ndt.resolution");
    ndt.max_iterations = static_cast<int>(node->declare_parameter<int64_t>("ndt.max_iterations"));
    ndt.num_threads = static_cast<int>(node->declare_parameter<int64_t>("ndt.num_threads"));
    ndt.num_threads = std::max(ndt.num_threads, 1);
    ndt_regularization_enable = node->declare_parameter<bool>("ndt.regularization.enable");
    ndt.regularization_scale_factor =
      static_cast<float>(node->declare_parameter<float>("ndt.regularization.scale_factor"));

    initial_pose_estimation.particles_num =
      node->declare_parameter<int64_t>("initial_pose_estimation.particles_num");
    initial_pose_estimation.n_startup_trials =
      node->declare_parameter<int64_t>("initial_pose_estimation.n_startup_trials");
    initial_pose_estimation.include_initial_pose =
      node->declare_parameter<bool>("initial_pose_estimation.include_initial_pose");
    initial_pose_estimation.force_initial_yaw =
      node->declare_parameter<bool>("initial_pose_estimation.force_initial_yaw");
    initial_pose_estimation.output_initial_yaw =
      node->declare_parameter<bool>("initial_pose_estimation.output_initial_yaw");
    initial_pose_estimation.use_sensor_points_stamp =
      node->declare_parameter<bool>("initial_pose_estimation.use_sensor_points_stamp", false);
    initial_pose_estimation.sensor_points_stamp_tolerance_sec = node->declare_parameter<double>(
      "initial_pose_estimation.sensor_points_stamp_tolerance_sec", 0.15);
    initial_pose_estimation.sensor_points_stamp_wait_timeout_sec = node->declare_parameter<double>(
      "initial_pose_estimation.sensor_points_stamp_wait_timeout_sec", 0.8);
    initial_pose_estimation.deterministic_offsets_enable = node->declare_parameter<bool>(
      "initial_pose_estimation.deterministic_offsets.enable", false);
    initial_pose_estimation.deterministic_offset_along_m =
      node->declare_parameter<std::vector<double>>(
        "initial_pose_estimation.deterministic_offsets.along_m", std::vector<double>{});
    initial_pose_estimation.deterministic_offset_cross_m =
      node->declare_parameter<std::vector<double>>(
        "initial_pose_estimation.deterministic_offsets.cross_m", std::vector<double>{});
    initial_pose_estimation.deterministic_offset_yaw_deg =
      node->declare_parameter<std::vector<double>>(
        "initial_pose_estimation.deterministic_offsets.yaw_deg", std::vector<double>{});

    validation.initial_pose_timeout_sec =
      node->declare_parameter<double>("validation.initial_pose_timeout_sec");
    validation.initial_pose_distance_tolerance_m =
      node->declare_parameter<double>("validation.initial_pose_distance_tolerance_m");
    validation.initial_to_result_distance_tolerance_m =
      node->declare_parameter<double>("validation.initial_to_result_distance_tolerance_m");
    validation.critical_upper_bound_exe_time_ms =
      node->declare_parameter<double>("validation.critical_upper_bound_exe_time_ms");
    validation.skipping_publish_num =
      node->declare_parameter<int64_t>("validation.skipping_publish_num");

    const int64_t converged_param_type_tmp =
      node->declare_parameter<int64_t>("score_estimation.converged_param_type");
    score_estimation.converged_param_type =
      static_cast<ConvergedParamType>(converged_param_type_tmp);
    score_estimation.converged_param_transform_probability =
      node->declare_parameter<double>("score_estimation.converged_param_transform_probability");
    score_estimation.converged_param_nearest_voxel_transformation_likelihood =
      node->declare_parameter<double>(
        "score_estimation.converged_param_nearest_voxel_transformation_likelihood");
    score_estimation.no_ground_points.enable =
      node->declare_parameter<bool>("score_estimation.no_ground_points.enable");
    score_estimation.no_ground_points.z_margin_for_ground_removal = node->declare_parameter<double>(
      "score_estimation.no_ground_points.z_margin_for_ground_removal");

    std::vector<double> output_pose_covariance =
      node->declare_parameter<std::vector<double>>("covariance.output_pose_covariance");
    for (std::size_t i = 0; i < output_pose_covariance.size(); ++i) {
      covariance.output_pose_covariance[i] = output_pose_covariance[i];
    }
    const int64_t covariance_estimation_type_tmp = node->declare_parameter<int64_t>(
      "covariance.covariance_estimation.covariance_estimation_type");
    covariance.covariance_estimation.covariance_estimation_type =
      static_cast<CovarianceEstimationType>(covariance_estimation_type_tmp);
    covariance.covariance_estimation.initial_pose_offset_model_x =
      node->declare_parameter<std::vector<double>>(
        "covariance.covariance_estimation.initial_pose_offset_model_x");
    covariance.covariance_estimation.initial_pose_offset_model_y =
      node->declare_parameter<std::vector<double>>(
        "covariance.covariance_estimation.initial_pose_offset_model_y");
    if (
      covariance.covariance_estimation.initial_pose_offset_model_x.size() !=
      covariance.covariance_estimation.initial_pose_offset_model_y.size()) {
      std::stringstream message;
      message << "Invalid initial pose offset model parameters."
              << "Please make sure that the number of elements in "
              << "initial_pose_offset_model_x and initial_pose_offset_model_y are the same.";
      throw std::runtime_error(message.str());
    }
    covariance.covariance_estimation.temperature =
      node->declare_parameter<double>("covariance.covariance_estimation.temperature");
    covariance.covariance_estimation.scale_factor =
      node->declare_parameter<double>("covariance.covariance_estimation.scale_factor");

    dynamic_map_loading.update_distance =
      node->declare_parameter<double>("dynamic_map_loading.update_distance");
    dynamic_map_loading.map_radius =
      node->declare_parameter<double>("dynamic_map_loading.map_radius");
    dynamic_map_loading.lidar_radius =
      node->declare_parameter<double>("dynamic_map_loading.lidar_radius");

    runtime_multistart.enable = node->declare_parameter<bool>("runtime_multistart.enable");
    runtime_multistart.debug_topic =
      node->declare_parameter<std::string>("runtime_multistart.debug_topic");
    runtime_multistart.min_stamp_sec =
      node->declare_parameter<double>("runtime_multistart.min_stamp_sec");
    runtime_multistart.trigger_initial_to_result_distance_m =
      node->declare_parameter<double>("runtime_multistart.trigger_initial_to_result_distance_m");
    runtime_multistart.trigger_yaw_delta_deg =
      node->declare_parameter<double>("runtime_multistart.trigger_yaw_delta_deg");
    runtime_multistart.trigger_score_margin =
      node->declare_parameter<double>("runtime_multistart.trigger_score_margin");
    runtime_multistart.offset_along_m =
      node->declare_parameter<std::vector<double>>("runtime_multistart.offset_along_m");
    runtime_multistart.offset_cross_m =
      node->declare_parameter<std::vector<double>>("runtime_multistart.offset_cross_m");
    runtime_multistart.offset_yaw_deg =
      node->declare_parameter<std::vector<double>>("runtime_multistart.offset_yaw_deg");
    if (
      runtime_multistart.offset_along_m.size() != runtime_multistart.offset_cross_m.size() ||
      runtime_multistart.offset_along_m.size() != runtime_multistart.offset_yaw_deg.size()) {
      throw std::runtime_error(
        "runtime_multistart offset_along_m, offset_cross_m, and offset_yaw_deg sizes must match");
    }
    runtime_multistart.min_transform_probability =
      node->declare_parameter<double>("runtime_multistart.min_transform_probability");
    runtime_multistart.min_nearest_voxel_transformation_likelihood = node->declare_parameter<double>(
      "runtime_multistart.min_nearest_voxel_transformation_likelihood");
    runtime_multistart.max_initial_to_result_distance_m =
      node->declare_parameter<double>("runtime_multistart.max_initial_to_result_distance_m");
    runtime_multistart.max_prior_innovation_m =
      node->declare_parameter<double>("runtime_multistart.max_prior_innovation_m");
    runtime_multistart.max_prior_along_m =
      node->declare_parameter<double>("runtime_multistart.max_prior_along_m");
    runtime_multistart.max_prior_cross_m =
      node->declare_parameter<double>("runtime_multistart.max_prior_cross_m");
    runtime_multistart.max_prior_yaw_deg =
      node->declare_parameter<double>("runtime_multistart.max_prior_yaw_deg");
    runtime_multistart.min_total_score =
      node->declare_parameter<double>("runtime_multistart.min_total_score");
    runtime_multistart.score_weight =
      node->declare_parameter<double>("runtime_multistart.score_weight");
    runtime_multistart.transform_probability_weight =
      node->declare_parameter<double>("runtime_multistart.transform_probability_weight");
    runtime_multistart.innovation_xy_penalty_weight =
      node->declare_parameter<double>("runtime_multistart.innovation_xy_penalty_weight");
    runtime_multistart.innovation_along_penalty_weight =
      node->declare_parameter<double>("runtime_multistart.innovation_along_penalty_weight", 0.05);
    runtime_multistart.innovation_cross_penalty_weight =
      node->declare_parameter<double>("runtime_multistart.innovation_cross_penalty_weight", 0.55);
    runtime_multistart.innovation_yaw_penalty_weight =
      node->declare_parameter<double>("runtime_multistart.innovation_yaw_penalty_weight");
    runtime_multistart.initial_to_result_penalty_weight =
      node->declare_parameter<double>("runtime_multistart.initial_to_result_penalty_weight");
    runtime_multistart.covariance_condition_penalty_weight =
      node->declare_parameter<double>("runtime_multistart.covariance_condition_penalty_weight");
    runtime_multistart.base_candidate_raw_score_margin =
      node->declare_parameter<double>("runtime_multistart.base_candidate_raw_score_margin");
    runtime_multistart.raw_score_override_margin =
      node->declare_parameter<double>("runtime_multistart.raw_score_override_margin", 0.0);
    runtime_multistart.raw_score_override_max_total_score_drop = node->declare_parameter<double>(
      "runtime_multistart.raw_score_override_max_total_score_drop", 0.0);
    runtime_multistart.raw_score_override_max_abs_along_m =
      node->declare_parameter<double>("runtime_multistart.raw_score_override_max_abs_along_m", 0.0);
    runtime_multistart.raw_score_override_max_abs_cross_m =
      node->declare_parameter<double>("runtime_multistart.raw_score_override_max_abs_cross_m", 0.0);
    runtime_multistart.raw_score_override_max_abs_yaw_deg =
      node->declare_parameter<double>("runtime_multistart.raw_score_override_max_abs_yaw_deg", 0.0);
    runtime_multistart.raw_score_override_max_initial_to_result_distance_m =
      node->declare_parameter<double>(
        "runtime_multistart.raw_score_override_max_initial_to_result_distance_m", 0.0);
    runtime_multistart.tier1_max_abs_along_m =
      node->declare_parameter<double>("runtime_multistart.tier1_max_abs_along_m", 1.0);
    runtime_multistart.tier1_max_abs_cross_m =
      node->declare_parameter<double>("runtime_multistart.tier1_max_abs_cross_m", 0.75);
    runtime_multistart.tier1_max_abs_yaw_deg =
      node->declare_parameter<double>("runtime_multistart.tier1_max_abs_yaw_deg", 2.0);
    runtime_multistart.tracking_tier1_period_sec =
      node->declare_parameter<double>("runtime_multistart.tracking_tier1_period_sec", 1.0);
    runtime_multistart.tracking_along_health_period_sec =
      node->declare_parameter<double>("runtime_multistart.tracking_along_health_period_sec", 0.0);
    runtime_multistart.tracking_far_tier_period_sec =
      node->declare_parameter<double>("runtime_multistart.tracking_far_tier_period_sec", 0.0);
    runtime_multistart.ambiguity_score_margin =
      node->declare_parameter<double>("runtime_multistart.ambiguity_score_margin", 0.15);
    runtime_multistart.ambiguity_along_spread_m =
      node->declare_parameter<double>("runtime_multistart.ambiguity_along_spread_m", 1.5);
    runtime_multistart.recovery_stable_required_frames = static_cast<int>(
      node->declare_parameter<int64_t>("runtime_multistart.recovery_stable_required_frames", 3));
    runtime_multistart.recovery_stable_max_innovation_m =
      node->declare_parameter<double>("runtime_multistart.recovery_stable_max_innovation_m", 1.0);
    runtime_multistart.recovery_stable_max_yaw_deg =
      node->declare_parameter<double>("runtime_multistart.recovery_stable_max_yaw_deg", 5.0);
    runtime_multistart.recovery_far_tier_period_sec =
      node->declare_parameter<double>("runtime_multistart.recovery_far_tier_period_sec", 1.0);
    runtime_multistart.recovery_far_tier_min_scan_interval = static_cast<int>(
      node->declare_parameter<int64_t>("runtime_multistart.recovery_far_tier_min_scan_interval", 10));
  }
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__HYPER_PARAMETERS_HPP_
