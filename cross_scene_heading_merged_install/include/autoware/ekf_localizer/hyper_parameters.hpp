// Copyright 2022 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__EKF_LOCALIZER__HYPER_PARAMETERS_HPP_
#define AUTOWARE__EKF_LOCALIZER__HYPER_PARAMETERS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cstdint>
#include <string>

namespace autoware::ekf_localizer {

class HyperParameters {
public:
  explicit HyperParameters(rclcpp::Node *node)
      : show_debug_info(node->declare_parameter<bool>("node.show_debug_info")),
        ekf_rate(node->declare_parameter<double>("node.predict_frequency")),
        ekf_dt(1.0 / std::max(ekf_rate, 0.1)),
        tf_rate_(node->declare_parameter<double>("node.tf_rate")),
        enable_yaw_bias_estimation(
            node->declare_parameter<bool>("node.enable_yaw_bias_estimation")),
        extend_state_step(
            node->declare_parameter<int>("node.extend_state_step")),
        pose_frame_id(
            node->declare_parameter<std::string>("misc.pose_frame_id")),
        pose_additional_delay(node->declare_parameter<double>(
            "pose_measurement.pose_additional_delay")),
        pose_gate_dist(
            node->declare_parameter<double>("pose_measurement.pose_gate_dist")),
        pose_smoothing_steps(node->declare_parameter<int>(
            "pose_measurement.pose_smoothing_steps")),
        max_pose_queue_size(node->declare_parameter<int>(
            "pose_measurement.max_pose_queue_size")),
        pose_subscription_qos_depth(static_cast<size_t>(std::max<std::int64_t>(
            node->declare_parameter<int>(
                "pose_measurement.pose_subscription_qos_depth", 1),
            1))),
        enable_position_only_state_mask(node->declare_parameter<bool>(
            "pose_measurement.enable_position_only_state_mask", false)),
        position_only_yaw_variance_threshold(node->declare_parameter<double>(
            "pose_measurement.position_only_yaw_variance_threshold", 0.5)),
        position_only_state_variance_floor_m2(node->declare_parameter<double>(
            "pose_measurement.position_only_state_variance_floor_m2", 0.0)),
        position_only_decorrelate_covariance(node->declare_parameter<bool>(
            "pose_measurement.position_only_decorrelate_covariance", false)),
        position_only_update_correlated_states(node->declare_parameter<bool>(
            "pose_measurement.position_only_update_correlated_states", false)),
        position_only_update_velocity_state(node->declare_parameter<bool>(
            "pose_measurement.position_only_update_velocity_state", false)),
        enable_gnss_bias_estimation(node->declare_parameter<bool>(
            "pose_measurement.enable_gnss_bias_estimation", false)),
        enable_position_only_dual_projection(node->declare_parameter<bool>(
            "pose_measurement.enable_position_only_dual_projection", false)),
        position_only_information_variance_multiplier(node->declare_parameter<
                                                      double>(
            "pose_measurement.position_only_information_variance_multiplier",
            1.0)),
        position_only_nis_gate(node->declare_parameter<double>(
            "pose_measurement.position_only_nis_gate", 0.0)),
        position_only_sequential_integrity_enabled(
            node->declare_parameter<bool>(
                "pose_measurement.position_only_sequential_integrity_enabled",
                false)),
        position_only_sequential_warmup_samples(
            static_cast<size_t>(std::max<std::int64_t>(
                node->declare_parameter<int>(
                    "pose_measurement.position_only_sequential_warmup_samples",
                    50),
                0))),
        position_only_sequential_reference_sigma(
            node->declare_parameter<double>(
                "pose_measurement.position_only_sequential_reference_sigma",
                0.5)),
        position_only_sequential_decision_threshold(
            node->declare_parameter<double>(
                "pose_measurement.position_only_sequential_decision_threshold",
                15.0)),
        position_only_sequential_recovery_samples(static_cast<
                                                  size_t>(std::max<
                                                          std::int64_t>(
            node->declare_parameter<int>(
                "pose_measurement.position_only_sequential_recovery_samples",
                20),
            1))),
        position_only_sequential_recovery_nis_threshold(node->declare_parameter<
                                                        double>(
            "pose_measurement.position_only_sequential_recovery_nis_threshold",
            5.991464547107979)),
        position_only_sequential_reset_gap_sec(node->declare_parameter<double>(
            "pose_measurement.position_only_sequential_reset_gap_sec", 1.0)),
        position_only_gnss_minimum_xy_std_m(node->declare_parameter<double>(
            "pose_measurement.position_only_gnss_minimum_xy_std_m", 0.0)),
        position_only_ndt_fixed_xy_std_m(node->declare_parameter<double>(
            "pose_measurement.position_only_ndt_fixed_xy_std_m", 0.0)),
        position_only_correction_full_age_sec(node->declare_parameter<double>(
            "pose_measurement.position_only_correction_full_age_sec", 0.25)),
        position_only_correction_zero_age_sec(node->declare_parameter<double>(
            "pose_measurement.position_only_correction_zero_age_sec", 1.0)),
        position_only_correction_rise_rate_per_sec(
            node->declare_parameter<double>(
                "pose_measurement.position_only_correction_rise_rate_per_sec",
                1.0)),
        position_only_correction_initial_settle_sec(
            node->declare_parameter<double>(
                "pose_measurement.position_only_correction_initial_settle_sec",
                0.0)),
        enable_height_only_measurement(node->declare_parameter<bool>(
            "pose_measurement.enable_height_only_measurement", false)),
        height_only_xy_variance_threshold_m2(node->declare_parameter<double>(
            "pose_measurement.height_only_xy_variance_threshold_m2", 100000.0)),
        height_only_z_variance_threshold_m2(node->declare_parameter<double>(
            "pose_measurement.height_only_z_variance_threshold_m2", 3.24)),
        height_only_nis_gate(node->declare_parameter<double>(
            "pose_measurement.height_only_nis_gate", 10.827566170662733)),
        twist_additional_delay(node->declare_parameter<double>(
            "twist_measurement.twist_additional_delay")),
        twist_gate_dist(node->declare_parameter<double>(
            "twist_measurement.twist_gate_dist")),
        twist_smoothing_steps(node->declare_parameter<int>(
            "twist_measurement.twist_smoothing_steps")),
        max_twist_queue_size(node->declare_parameter<int>(
            "twist_measurement.max_twist_queue_size")),
        proc_stddev_vx_c(
            node->declare_parameter<double>("process_noise.proc_stddev_vx_c")),
        proc_stddev_wz_c(
            node->declare_parameter<double>("process_noise.proc_stddev_wz_c")),
        proc_stddev_yaw_c(
            node->declare_parameter<double>("process_noise.proc_stddev_yaw_c")),
        gnss_bias_stationary_stddev_m(node->declare_parameter<double>(
            "process_noise.gnss_bias_stationary_stddev_m",
            0.18407490243154828)),
        gnss_bias_correlation_time_sec(node->declare_parameter<double>(
            "process_noise.gnss_bias_correlation_time_sec", 5.0)),
        z_filter_proc_dev(node->declare_parameter<double>(
            "simple_1d_filter_parameters.z_filter_proc_dev")),
        roll_filter_proc_dev(node->declare_parameter<double>(
            "simple_1d_filter_parameters.roll_filter_proc_dev")),
        pitch_filter_proc_dev(node->declare_parameter<double>(
            "simple_1d_filter_parameters.pitch_filter_proc_dev")),
        enable_slope_kinematic_prediction(node->declare_parameter<bool>(
            "simple_1d_filter_parameters.enable_slope_kinematic_prediction",
            false)),
        pose_no_update_count_threshold_warn(node->declare_parameter<int>(
            "diagnostics.pose_no_update_count_threshold_warn")),
        pose_no_update_count_threshold_error(node->declare_parameter<int>(
            "diagnostics.pose_no_update_count_threshold_error")),
        twist_no_update_count_threshold_warn(node->declare_parameter<int>(
            "diagnostics.twist_no_update_count_threshold_warn")),
        twist_no_update_count_threshold_error(node->declare_parameter<int>(
            "diagnostics.twist_no_update_count_threshold_error")),
        ellipse_scale(
            node->declare_parameter<double>("diagnostics.ellipse_scale")),
        error_ellipse_size(
            node->declare_parameter<double>("diagnostics.error_ellipse_size")),
        warn_ellipse_size(
            node->declare_parameter<double>("diagnostics.warn_ellipse_size")),
        error_ellipse_size_lateral_direction(node->declare_parameter<double>(
            "diagnostics.error_ellipse_size_lateral_direction")),
        warn_ellipse_size_lateral_direction(node->declare_parameter<double>(
            "diagnostics.warn_ellipse_size_lateral_direction")),
        threshold_observable_velocity_mps(node->declare_parameter<double>(
            "misc.threshold_observable_velocity_mps")) {}

  const bool show_debug_info;
  const double ekf_rate;
  const double ekf_dt;
  const double tf_rate_;
  const bool enable_yaw_bias_estimation;
  const size_t extend_state_step;
  const std::string pose_frame_id;
  const double pose_additional_delay;
  const double pose_gate_dist;
  const size_t pose_smoothing_steps;
  const size_t max_pose_queue_size;
  const size_t pose_subscription_qos_depth;
  const bool enable_position_only_state_mask;
  const double position_only_yaw_variance_threshold;
  const double position_only_state_variance_floor_m2;
  const bool position_only_decorrelate_covariance;
  const bool position_only_update_correlated_states;
  const bool position_only_update_velocity_state;
  const bool enable_gnss_bias_estimation;
  const bool enable_position_only_dual_projection;
  const double position_only_information_variance_multiplier;
  const double position_only_nis_gate;
  const bool position_only_sequential_integrity_enabled;
  const size_t position_only_sequential_warmup_samples;
  const double position_only_sequential_reference_sigma;
  const double position_only_sequential_decision_threshold;
  const size_t position_only_sequential_recovery_samples;
  const double position_only_sequential_recovery_nis_threshold;
  const double position_only_sequential_reset_gap_sec;
  const double position_only_gnss_minimum_xy_std_m;
  const double position_only_ndt_fixed_xy_std_m;
  const double position_only_correction_full_age_sec;
  const double position_only_correction_zero_age_sec;
  const double position_only_correction_rise_rate_per_sec;
  const double position_only_correction_initial_settle_sec;
  const bool enable_height_only_measurement;
  const double height_only_xy_variance_threshold_m2;
  const double height_only_z_variance_threshold_m2;
  const double height_only_nis_gate;
  const double twist_additional_delay;
  const double twist_gate_dist;
  const size_t twist_smoothing_steps;
  const size_t max_twist_queue_size;
  const double proc_stddev_vx_c;  //!< @brief  vx process noise
  const double proc_stddev_wz_c;  //!< @brief  wz process noise
  const double proc_stddev_yaw_c; //!< @brief  yaw process noise
  const double gnss_bias_stationary_stddev_m;
  const double gnss_bias_correlation_time_sec;
  const double z_filter_proc_dev;
  const double roll_filter_proc_dev;
  const double pitch_filter_proc_dev;
  const bool enable_slope_kinematic_prediction;
  const size_t pose_no_update_count_threshold_warn;
  const size_t pose_no_update_count_threshold_error;
  const size_t twist_no_update_count_threshold_warn;
  const size_t twist_no_update_count_threshold_error;
  double ellipse_scale;
  double error_ellipse_size;
  double warn_ellipse_size;
  double error_ellipse_size_lateral_direction;
  double warn_ellipse_size_lateral_direction;

  const double threshold_observable_velocity_mps;
};

} // namespace autoware::ekf_localizer

#endif // AUTOWARE__EKF_LOCALIZER__HYPER_PARAMETERS_HPP_
