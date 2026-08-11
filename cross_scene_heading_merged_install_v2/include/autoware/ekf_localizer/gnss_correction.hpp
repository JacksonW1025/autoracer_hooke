// Copyright 2026 Autoware Foundation
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

#ifndef AUTOWARE__EKF_LOCALIZER__GNSS_CORRECTION_HPP_
#define AUTOWARE__EKF_LOCALIZER__GNSS_CORRECTION_HPP_

#include "autoware/ekf_localizer/state_index.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace autoware::ekf_localizer {

inline double desired_gnss_correction_authority(
    const double source_age_sec, const bool initial_settle_complete,
    const double full_authority_age_sec, const double zero_authority_age_sec) {
  if (!initial_settle_complete || !std::isfinite(source_age_sec) ||
      source_age_sec < 0.0 || !std::isfinite(full_authority_age_sec) ||
      !std::isfinite(zero_authority_age_sec) || full_authority_age_sec < 0.0 ||
      zero_authority_age_sec <= full_authority_age_sec) {
    return 0.0;
  }
  if (source_age_sec <= full_authority_age_sec) {
    return 1.0;
  }
  if (source_age_sec >= zero_authority_age_sec) {
    return 0.0;
  }
  return (zero_authority_age_sec - source_age_sec) /
         (zero_authority_age_sec - full_authority_age_sec);
}

inline double slew_gnss_correction_authority(const double previous_authority,
                                             const double desired_authority,
                                             const double delta_sec,
                                             const double rise_rate_per_sec) {
  if (!std::isfinite(previous_authority) || !std::isfinite(desired_authority) ||
      !std::isfinite(delta_sec) || !std::isfinite(rise_rate_per_sec) ||
      delta_sec < 0.0 || rise_rate_per_sec < 0.0) {
    return 0.0;
  }
  const double previous = std::clamp(previous_authority, 0.0, 1.0);
  const double desired = std::clamp(desired_authority, 0.0, 1.0);
  const double maximum = previous + rise_rate_per_sec * delta_sec;
  // Loss of trust is immediate.  Only reacquisition is rate limited.
  return std::min(desired, maximum);
}

inline Eigen::MatrixXd
position_only_bias_measurement_matrix(const Eigen::Index state_dimension) {
  if (state_dimension <= IDX::GNSS_BY) {
    throw std::invalid_argument(
        "GNSS bias measurement requires the eight-state EKF");
  }
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(2, state_dimension);
  matrix(0, IDX::GNSS_BX) = 1.0;
  matrix(1, IDX::GNSS_BY) = 1.0;
  return matrix;
}

inline Eigen::Matrix2d regularized_position_measurement_covariance(
    const Eigen::Matrix2d &receiver_covariance,
    const double minimum_receiver_std_m, const double ndt_fixed_std_m) {
  if (!receiver_covariance.array().isFinite().all() ||
      !std::isfinite(minimum_receiver_std_m) ||
      !std::isfinite(ndt_fixed_std_m) || minimum_receiver_std_m < 0.0 ||
      ndt_fixed_std_m < 0.0) {
    throw std::invalid_argument("position covariance inputs must be finite");
  }
  Eigen::Matrix2d covariance =
      0.5 * (receiver_covariance + receiver_covariance.transpose());
  const double minimum_variance =
      minimum_receiver_std_m * minimum_receiver_std_m;
  covariance(0, 0) = std::max(covariance(0, 0), minimum_variance);
  covariance(1, 1) = std::max(covariance(1, 1), minimum_variance);
  covariance.diagonal().array() += ndt_fixed_std_m * ndt_fixed_std_m;
  return covariance;
}

inline Eigen::MatrixXd
public_pose_projection_matrix(const Eigen::Index state_dimension,
                              const double correction_authority) {
  if (state_dimension < 6) {
    throw std::invalid_argument("pose projection requires at least six states");
  }
  Eigen::MatrixXd projection = Eigen::MatrixXd::Zero(6, state_dimension);
  projection.leftCols<6>().setIdentity();
  if (state_dimension > IDX::GNSS_BY) {
    const double authority = std::clamp(correction_authority, 0.0, 1.0);
    projection(IDX::X, IDX::GNSS_BX) = authority;
    projection(IDX::Y, IDX::GNSS_BY) = authority;
  }
  return projection;
}

} // namespace autoware::ekf_localizer

#endif // AUTOWARE__EKF_LOCALIZER__GNSS_CORRECTION_HPP_
