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

#ifndef AUTOWARE__EKF_LOCALIZER__MEASUREMENT_HPP_
#define AUTOWARE__EKF_LOCALIZER__MEASUREMENT_HPP_

#include <Eigen/Core>

#include <array>

namespace autoware::ekf_localizer
{

bool is_position_only_pose_measurement(
  const std::array<double, 36ul> & covariance, bool feature_enabled,
  double yaw_variance_threshold);
bool is_height_only_pose_measurement(
  const std::array<double, 36ul> & covariance, bool feature_enabled,
  double xy_variance_threshold, double z_variance_threshold,
  double yaw_variance_threshold);
Eigen::Matrix<double, 3, 6> pose_measurement_matrix();
Eigen::Matrix<double, 2, 6> position_only_pose_measurement_matrix();
Eigen::Matrix<double, 2, 6> twist_measurement_matrix();
Eigen::MatrixXd pose_measurement_matrix(Eigen::Index state_dimension);
Eigen::MatrixXd position_only_pose_measurement_matrix(
  Eigen::Index state_dimension, bool include_gnss_bias);
Eigen::MatrixXd twist_measurement_matrix(Eigen::Index state_dimension);
Eigen::Matrix3d pose_measurement_covariance(
  const std::array<double, 36ul> & covariance, const size_t smoothing_step);
Eigen::Matrix2d position_only_pose_measurement_covariance(
  const std::array<double, 36ul> & covariance, const size_t smoothing_step);
Eigen::Matrix2d position_only_pose_innovation_covariance(
  const Eigen::MatrixXd & state_covariance, bool include_gnss_bias,
  const std::array<double, 36ul> & measurement_covariance);
Eigen::Matrix2d twist_measurement_covariance(
  const std::array<double, 36ul> & covariance, const size_t smoothing_step);

}  // namespace autoware::ekf_localizer

#endif  // AUTOWARE__EKF_LOCALIZER__MEASUREMENT_HPP_
