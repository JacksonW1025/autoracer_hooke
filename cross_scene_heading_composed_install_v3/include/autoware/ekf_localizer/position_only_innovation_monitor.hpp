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

#ifndef AUTOWARE__EKF_LOCALIZER__POSITION_ONLY_INNOVATION_MONITOR_HPP_
#define AUTOWARE__EKF_LOCALIZER__POSITION_ONLY_INNOVATION_MONITOR_HPP_

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace autoware::ekf_localizer {

struct PositionOnlyInnovationMonitorConfig {
  bool enabled{false};
  std::size_t warmup_samples{50};
  double reference_sigma{0.5};
  double decision_threshold{15.0};
  std::size_t recovery_samples{20};
  double recovery_nis_threshold{5.991464547107979};
  double reset_gap_sec{1.0};
};

struct PositionOnlyInnovationMonitorResult {
  bool reject{false};
  bool latched{false};
  double statistic{0.0};
};

/// Two-sided Page-CUSUM monitor for persistent, signed position innovations.
///
/// The ordinary per-sample NIS gate remains the first line of defence.  This
/// monitor is evaluated only for samples that already passed that gate.  It
/// detects a different failure mode: many individually plausible innovations
/// that keep pushing in one direction.  When disabled it is a strict no-op.
class PositionOnlyInnovationMonitor {
public:
  explicit PositionOnlyInnovationMonitor(
      const PositionOnlyInnovationMonitorConfig &config = {})
      : config_(config) {
    validate_config();
  }

  void reset() {
    positive_cusum_.fill(0.0);
    negative_cusum_.fill(0.0);
    warmup_count_ = 0;
    recovery_count_ = 0;
    observed_count_ = 0;
    rejected_count_ = 0;
    last_stamp_ns_ = 0;
    latched_ = false;
    statistic_ = 0.0;
  }

  PositionOnlyInnovationMonitorResult
  observe(const Eigen::Vector2d &marginal_standardized_innovation,
          const double nis, const std::int64_t stamp_ns) {
    if (!config_.enabled) {
      return {};
    }
    if (!marginal_standardized_innovation.allFinite() || !std::isfinite(nis) ||
        nis < 0.0) {
      latched_ = true;
      ++rejected_count_;
      return result(true);
    }

    if (last_stamp_ns_ != 0) {
      // The EKF's pose smoothing intentionally applies one source
      // measurement several times.  Those applications are not independent
      // innovation samples.  Count and evaluate each source timestamp once;
      // otherwise smoothing both over-weights the evidence and repeatedly
      // resets the sequence on a zero timestamp gap.
      if (stamp_ns == last_stamp_ns_) {
        return result(latched_);
      }
      const double gap_sec =
          static_cast<double>(stamp_ns - last_stamp_ns_) / 1.0e9;
      if (gap_sec < 0.0 || gap_sec > config_.reset_gap_sec) {
        reset_sequence_preserving_latch();
      }
    }
    last_stamp_ns_ = stamp_ns;
    ++observed_count_;

    if (latched_) {
      if (nis <= config_.recovery_nis_threshold) {
        ++recovery_count_;
      } else {
        recovery_count_ = 0;
      }
      if (recovery_count_ >= config_.recovery_samples) {
        latched_ = false;
        reset_sequence_preserving_latch();
        return result(false);
      }
      ++rejected_count_;
      return result(true);
    }

    if (warmup_count_ < config_.warmup_samples) {
      ++warmup_count_;
      return result(false);
    }

    for (std::size_t axis = 0; axis < 2; ++axis) {
      const double value =
          marginal_standardized_innovation(static_cast<Eigen::Index>(axis));
      positive_cusum_[axis] = std::max(0.0, positive_cusum_[axis] + value -
                                                config_.reference_sigma);
      negative_cusum_[axis] = std::max(0.0, negative_cusum_[axis] - value -
                                                config_.reference_sigma);
    }
    statistic_ = std::max({positive_cusum_[0], positive_cusum_[1],
                           negative_cusum_[0], negative_cusum_[1]});
    if (statistic_ >= config_.decision_threshold) {
      latched_ = true;
      recovery_count_ = 0;
      ++rejected_count_;
      return result(true);
    }
    return result(false);
  }

  [[nodiscard]] bool latched() const { return latched_; }
  [[nodiscard]] double statistic() const { return statistic_; }
  [[nodiscard]] std::size_t observed_count() const { return observed_count_; }
  [[nodiscard]] std::size_t rejected_count() const { return rejected_count_; }

private:
  void validate_config() const {
    if (!std::isfinite(config_.reference_sigma) ||
        config_.reference_sigma < 0.0 ||
        !std::isfinite(config_.decision_threshold) ||
        config_.decision_threshold <= 0.0 || config_.recovery_samples == 0 ||
        !std::isfinite(config_.recovery_nis_threshold) ||
        config_.recovery_nis_threshold <= 0.0 ||
        !std::isfinite(config_.reset_gap_sec) || config_.reset_gap_sec <= 0.0) {
      throw std::invalid_argument(
          "invalid position-only innovation monitor configuration");
    }
  }

  void reset_sequence_preserving_latch() {
    positive_cusum_.fill(0.0);
    negative_cusum_.fill(0.0);
    warmup_count_ = 0;
    recovery_count_ = 0;
    statistic_ = 0.0;
  }

  [[nodiscard]] PositionOnlyInnovationMonitorResult
  result(const bool reject) const {
    return {reject, latched_, statistic_};
  }

  PositionOnlyInnovationMonitorConfig config_;
  std::array<double, 2> positive_cusum_{{0.0, 0.0}};
  std::array<double, 2> negative_cusum_{{0.0, 0.0}};
  std::size_t warmup_count_{0};
  std::size_t recovery_count_{0};
  std::size_t observed_count_{0};
  std::size_t rejected_count_{0};
  std::int64_t last_stamp_ns_{0};
  bool latched_{false};
  double statistic_{0.0};
};

} // namespace autoware::ekf_localizer

#endif // AUTOWARE__EKF_LOCALIZER__POSITION_ONLY_INNOVATION_MONITOR_HPP_
