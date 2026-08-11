// Copyright 2018-2019 Autoware Foundation
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

#ifndef AUTOWARE__KALMAN_FILTER__TIME_DELAY_KALMAN_FILTER_HPP_
#define AUTOWARE__KALMAN_FILTER__TIME_DELAY_KALMAN_FILTER_HPP_

#include "autoware/kalman_filter/kalman_filter.hpp"

#include <Eigen/Core>
#include <Eigen/LU>

#include <iostream>

namespace autoware::kalman_filter
{
/**
 * @file time_delay_kalman_filter.h
 * @brief kalman filter with delayed measurement class
 * @author Takamasa Horibe
 * @date 2019.05.01
 */

class TimeDelayKalmanFilter : public KalmanFilter
{
public:
  /**
   * @brief No initialization constructor.
   */
  TimeDelayKalmanFilter() = default;

  ~TimeDelayKalmanFilter() = default;

  TimeDelayKalmanFilter(const TimeDelayKalmanFilter &) = delete;
  TimeDelayKalmanFilter & operator=(const TimeDelayKalmanFilter &) = delete;
  TimeDelayKalmanFilter(TimeDelayKalmanFilter &&) noexcept = default;
  TimeDelayKalmanFilter & operator=(TimeDelayKalmanFilter &&) noexcept = default;

  /**
   * @brief initialization of kalman filter
   * @param x initial state
   * @param P0 initial covariance of estimated state
   * @param max_delay_step Maximum number of delay steps, which determines the dimension of the
   * extended kalman filter
   * @param non_delayed_tail_size Number of trailing state elements kept only at the current time.
   * This is intended for slowly varying nuisance states that are observed together with delayed
   * vehicle states but do not require a full delay history.
   */
  void init(
    const Eigen::MatrixXd & x, const Eigen::MatrixXd & P, int max_delay_step,
    int non_delayed_tail_size = 0);

  /**
   * @brief get latest time estimated state
   */
  Eigen::MatrixXd getLatestX() const;

  /**
   * @brief get latest time estimation covariance
   */
  Eigen::MatrixXd getLatestP() const;

  /**
   * @brief Get a logical state element at a delayed vehicle-state step.
   *
   * A configured non-delayed tail is shared by all delay steps and therefore
   * always resolves to its current value.
   */
  double getDelayedXelement(int delay_step, int state_index) const;

  [[nodiscard]] int getExtendedStateDimension() const { return dim_x_ex_; }

  /**
   * @brief calculate kalman filter covariance by precision model with time delay. This is mainly
   * for EKF of nonlinear process model.
   * @param x_next predicted state by prediction model
   * @param A coefficient matrix of x for process model
   * @param Q covariance matrix for process model
   * @return bool to check matrix operations are being performed properly
   */
  bool predictWithDelay(
    const Eigen::MatrixXd & x_next, const Eigen::MatrixXd & A, const Eigen::MatrixXd & Q);

  /**
   * @brief calculate kalman filter covariance by measurement model with time delay. This is mainly
   * for EKF of nonlinear process model.
   * @param y measured values
   * @param C coefficient matrix of x for measurement model
   * @param R covariance matrix for measurement model
   * @param delay_step measurement delay
   * @return bool to check matrix operations are being performed properly
   */
  bool updateWithDelay(
    const Eigen::MatrixXd & y, const Eigen::MatrixXd & C, const Eigen::MatrixXd & R,
    const int delay_step);

  /**
   * @brief delayed measurement update constrained to selected per-step states.
   *
   * The per-step mask is repeated over the extended delay state so a delayed
   * position correction can propagate to current and historical positions
   * without changing unrelated state variables.
   */
  bool updateWithDelayStateMask(
    const Eigen::MatrixXd & y, const Eigen::MatrixXd & C, const Eigen::MatrixXd & R,
    int delay_step, const Eigen::VectorXi & state_update_mask,
    double selected_state_variance_floor = 0.0,
    bool decorrelate_selected_states = false);

private:
  [[nodiscard]] int delayedStateOffset(int delay_step) const;
  [[nodiscard]] Eigen::MatrixXd createExtendedMeasurementMatrix(
    const Eigen::MatrixXd & C, int delay_step) const;

  int max_delay_step_{0};  //!< @brief maximum number of delay steps
  int dim_x_{0};           //!< @brief dimension of latest state
  int dim_x_ex_{0};        //!< @brief dimension of extended state with time delay
  int non_delayed_tail_size_{0};
  int delayed_state_size_{0};
};
}  // namespace autoware::kalman_filter
#endif  // AUTOWARE__KALMAN_FILTER__TIME_DELAY_KALMAN_FILTER_HPP_
