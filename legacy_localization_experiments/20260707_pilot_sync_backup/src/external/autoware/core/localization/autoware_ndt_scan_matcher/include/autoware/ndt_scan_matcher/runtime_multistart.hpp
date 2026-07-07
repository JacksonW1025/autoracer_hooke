#ifndef AUTOWARE__NDT_SCAN_MATCHER__RUNTIME_MULTISTART_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__RUNTIME_MULTISTART_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace autoware::ndt_scan_matcher
{

struct RuntimeCandidate
{
  std::size_t index{};
  bool converged{};
  int iteration_num{};
  int max_iterations{};
  double transform_probability{};
  double nearest_voxel_transformation_likelihood{};
  double initial_to_result_distance_m{};
  double innovation_along_m{};
  double innovation_cross_m{};
  double innovation_yaw_rad{};
  double offset_along_m{};
  double offset_cross_m{};
  double offset_yaw_deg{};
  double covariance_condition_number{1.0};
  double localizability_along_variance_m2{};
  double localizability_cross_variance_m2{};
  double execution_time_ms{};
  bool has_gnss_weak_prior{};
  double gnss_weak_prior_distance_m{};
  double gnss_weak_prior_penalty{};
};

struct RuntimeCandidateScoringOptions
{
  double min_transform_probability{-std::numeric_limits<double>::infinity()};
  double min_nearest_voxel_transformation_likelihood{-std::numeric_limits<double>::infinity()};
  double max_initial_to_result_distance_m{5.0};
  double max_prior_innovation_m{0.0};
  double max_prior_along_m{0.0};
  double max_prior_cross_m{0.0};
  double max_prior_yaw_deg{0.0};
  double min_total_score{-std::numeric_limits<double>::infinity()};
  int max_iteration_num{0};
  double score_weight{1.0};
  double transform_probability_weight{0.05};
  double innovation_xy_penalty_weight{0.55};
  double innovation_along_penalty_weight{0.05};
  double innovation_cross_penalty_weight{0.55};
  double innovation_yaw_penalty_weight{0.05};
  double initial_to_result_penalty_weight{0.25};
  double covariance_condition_penalty_weight{0.01};
  double base_candidate_raw_score_margin{0.0};
  double raw_score_override_margin{0.0};
  double raw_score_override_max_total_score_drop{0.0};
  double raw_score_override_max_abs_along_m{0.0};
  double raw_score_override_max_abs_cross_m{0.0};
  double raw_score_override_max_abs_yaw_deg{0.0};
  double raw_score_override_max_initial_to_result_distance_m{0.0};
  bool enable_gnss_weak_prior{};
  double gnss_weak_prior_weight{0.0};
  double gnss_weak_prior_sigma_m{5.0};
  double gnss_weak_prior_max_penalty{8.0};
  double gnss_weak_prior_max_distance_m{0.0};
  bool gnss_weak_prior_innovation_gate_enable{};
  double gnss_weak_prior_innovation_gate_m{0.0};
};

struct RuntimeCandidateTierOptions
{
  double tier1_max_abs_along_m{1.0};
  double tier1_max_abs_cross_m{0.75};
  double tier1_max_abs_yaw_deg{2.0};
  double tracking_along_health_period_sec{0.0};
  double tracking_far_tier_period_sec{0.0};
  double ambiguity_score_margin{0.15};
  double ambiguity_along_spread_m{1.5};
  double recovery_stable_max_innovation_m{1.0};
  double recovery_stable_max_yaw_deg{5.0};
  double recovery_far_tier_period_sec{1.0};
  int recovery_far_tier_min_scan_interval{10};
};

struct RuntimeCandidateScore
{
  std::size_t candidate_index{};
  double total_score{-std::numeric_limits<double>::infinity()};
  std::string reject_reason{};
};

struct RuntimeCandidateSelection
{
  bool has_selected_candidate{};
  std::size_t selected_candidate_index{};
  std::vector<RuntimeCandidateScore> candidate_scores{};
};

struct RuntimeOutputSelection
{
  bool has_output_candidate{};
  std::size_t output_candidate_index{};
};

struct RuntimeCandidateSpreadCovariance
{
  bool ambiguous{};
  int contender_count{};
  double along_variance_m2{};
  double cross_variance_m2{};
};

struct RuntimeGnssWeakPriorGateOptions
{
  bool enable_gnss_weak_prior{};
  bool condition_enable{};
  bool has_fresh_gnss_prior{};
  bool base_candidate_converged{};
  bool recovery_active{};
  bool small_tier_ambiguous{};
  bool spread_covariance_ambiguous{};
  int rejected_scan_streak{};
  double base_localizability_along_variance_m2{};
  double min_along_variance_m2{};
  bool healthy_base_passthrough_enable{};
  int weak_prior_hold_remaining_scans{};
};

struct RuntimeGnssWeakPriorGateDecision
{
  bool enable_weak_prior{};
  bool healthy_base_passthrough{};
  std::string reason{"disabled"};
};

struct RuntimeOutputCovariance
{
  double along_variance_m2{};
  double cross_variance_m2{};
};

inline double project_planar_pose_covariance(
  const std::array<double, 36> & covariance, const double axis_x, const double axis_y)
{
  const double xx = covariance[0 + 6 * 0];
  const double xy = covariance[0 + 6 * 1];
  const double yx = covariance[1 + 6 * 0];
  const double yy = covariance[1 + 6 * 1];
  return axis_x * axis_x * xx + axis_x * axis_y * (xy + yx) + axis_y * axis_y * yy;
}

inline RuntimeOutputCovariance project_runtime_output_covariance(
  const std::array<double, 36> & covariance, const double forward_x, const double forward_y,
  const double lateral_x, const double lateral_y)
{
  RuntimeOutputCovariance projected;
  projected.along_variance_m2 = project_planar_pose_covariance(covariance, forward_x, forward_y);
  projected.cross_variance_m2 = project_planar_pose_covariance(covariance, lateral_x, lateral_y);
  return projected;
}

inline RuntimeGnssWeakPriorGateDecision decide_runtime_gnss_weak_prior_gate(
  const RuntimeGnssWeakPriorGateOptions & options)
{
  RuntimeGnssWeakPriorGateDecision decision;
  if (!options.enable_gnss_weak_prior) {
    decision.reason = "global_disabled";
    return decision;
  }
  if (!options.has_fresh_gnss_prior) {
    decision.reason = "no_fresh_gnss";
    return decision;
  }
  if (!options.condition_enable) {
    decision.enable_weak_prior = true;
    decision.reason = "unconditional";
    return decision;
  }
  if (options.weak_prior_hold_remaining_scans > 0) {
    if (options.healthy_base_passthrough_enable && options.base_candidate_converged) {
      decision.healthy_base_passthrough = true;
      decision.reason = "condition_hold_healthy_base_passthrough";
      return decision;
    }
    decision.enable_weak_prior = true;
    decision.reason = "condition_hold";
    return decision;
  }
  if (!options.base_candidate_converged) {
    decision.enable_weak_prior = true;
    decision.reason = "base_not_converged";
    return decision;
  }
  if (options.rejected_scan_streak > 0) {
    decision.enable_weak_prior = true;
    decision.reason = "rejected_scan_streak";
    return decision;
  }
  if (options.recovery_active) {
    decision.enable_weak_prior = true;
    decision.reason = "recovery_active";
    return decision;
  }
  if (options.small_tier_ambiguous) {
    decision.enable_weak_prior = true;
    decision.reason = "small_tier_ambiguous";
    return decision;
  }
  if (options.spread_covariance_ambiguous) {
    decision.enable_weak_prior = true;
    decision.reason = "spread_covariance_ambiguous";
    return decision;
  }
  if (
    options.min_along_variance_m2 > 0.0 &&
    options.base_localizability_along_variance_m2 >= options.min_along_variance_m2) {
    decision.enable_weak_prior = true;
    decision.reason = "along_variance_high";
    return decision;
  }

  decision.reason = "healthy_tracking";
  decision.healthy_base_passthrough =
    options.healthy_base_passthrough_enable && options.base_candidate_converged;
  return decision;
}

inline void disable_runtime_selection_gates_for_single_start(
  RuntimeCandidateScoringOptions & options)
{
  options.min_transform_probability = -std::numeric_limits<double>::infinity();
  options.min_nearest_voxel_transformation_likelihood =
    -std::numeric_limits<double>::infinity();
  options.max_initial_to_result_distance_m = 0.0;
  options.max_prior_innovation_m = 0.0;
  options.max_prior_along_m = 0.0;
  options.max_prior_cross_m = 0.0;
  options.max_prior_yaw_deg = 0.0;
  options.min_total_score = -std::numeric_limits<double>::infinity();
  options.max_iteration_num = 0;
}

inline RuntimeOutputSelection choose_runtime_output_candidate(
  const bool runtime_controls_output, const std::vector<RuntimeCandidate> & candidates,
  const RuntimeCandidateSelection & selection)
{
  RuntimeOutputSelection output;
  if (!runtime_controls_output) {
    if (!candidates.empty()) {
      output.has_output_candidate = true;
      output.output_candidate_index = candidates.front().index;
    }
    return output;
  }
  output.has_output_candidate = selection.has_selected_candidate;
  output.output_candidate_index = selection.selected_candidate_index;
  return output;
}

struct RuntimeRecoveryState
{
  int rejected_scan_streak{};
  int recovery_stable_frames{};
  bool recovery_active{};
  bool recovery_verified{};
};

inline RuntimeRecoveryState update_runtime_recovery_state(
  const bool has_selected_candidate, const bool selected_candidate_stable,
  const bool recovery_attempt, const bool previous_recovery_active,
  const int previous_recovery_stable_frames, const int previous_rejected_scan_streak,
  const int recovery_stable_required_frames)
{
  (void)previous_recovery_active;
  RuntimeRecoveryState state;
  if (!has_selected_candidate) {
    state.rejected_scan_streak = previous_rejected_scan_streak + 1;
    state.recovery_active = true;
    return state;
  }

  state.rejected_scan_streak = 0;
  if (recovery_attempt && selected_candidate_stable) {
    state.recovery_stable_frames = previous_recovery_stable_frames + 1;
    state.recovery_verified =
      state.recovery_stable_frames >= std::max(1, recovery_stable_required_frames);
    state.recovery_active = !state.recovery_verified;
  }
  return state;
}

inline bool runtime_candidate_fails_gnss_innovation_gate(
  const RuntimeCandidate & candidate, const RuntimeCandidateScoringOptions & options)
{
  return options.gnss_weak_prior_innovation_gate_enable &&
         options.gnss_weak_prior_innovation_gate_m > 0.0 && candidate.has_gnss_weak_prior &&
         candidate.gnss_weak_prior_distance_m > options.gnss_weak_prior_innovation_gate_m;
}

inline RuntimeCandidateScore score_runtime_candidate(
  const RuntimeCandidate & candidate, const RuntimeCandidateScoringOptions & options)
{
  RuntimeCandidateScore score;
  score.candidate_index = candidate.index;

  if (!candidate.converged) {
    score.reject_reason = "not_converged";
    return score;
  }
  if (
    options.max_iteration_num > 0 && candidate.iteration_num >= options.max_iteration_num &&
    !candidate.converged) {
    score.reject_reason = "max_iteration";
    return score;
  }
  if (candidate.transform_probability < options.min_transform_probability) {
    score.reject_reason = "score_below_threshold";
    return score;
  }
  if (
    candidate.nearest_voxel_transformation_likelihood <
    options.min_nearest_voxel_transformation_likelihood) {
    score.reject_reason = "score_below_threshold";
    return score;
  }
  if (
    options.max_initial_to_result_distance_m > 0.0 &&
    candidate.initial_to_result_distance_m > options.max_initial_to_result_distance_m) {
    score.reject_reason = "initial_to_result_too_large";
    return score;
  }

  const double innovation_xy =
    std::hypot(candidate.innovation_along_m, candidate.innovation_cross_m);
  const double innovation_yaw_deg = std::abs(candidate.innovation_yaw_rad) * 180.0 / M_PI;
  if (
    options.max_prior_along_m > 0.0 &&
    std::abs(candidate.innovation_along_m) > options.max_prior_along_m) {
    score.reject_reason = "prior_along_too_large";
    return score;
  }
  if (
    options.max_prior_cross_m > 0.0 &&
    std::abs(candidate.innovation_cross_m) > options.max_prior_cross_m) {
    score.reject_reason = "prior_cross_too_large";
    return score;
  }
  if (options.max_prior_innovation_m > 0.0 && innovation_xy > options.max_prior_innovation_m) {
    score.reject_reason = "prior_innovation_too_large";
    return score;
  }
  if (options.max_prior_yaw_deg > 0.0 && innovation_yaw_deg > options.max_prior_yaw_deg) {
    score.reject_reason = "prior_yaw_too_large";
    return score;
  }
  const double condition_penalty = std::log1p(std::max(0.0, candidate.covariance_condition_number));
  score.total_score =
    options.score_weight * candidate.nearest_voxel_transformation_likelihood +
    options.transform_probability_weight * candidate.transform_probability -
    options.innovation_along_penalty_weight * std::abs(candidate.innovation_along_m) -
    options.innovation_cross_penalty_weight * std::abs(candidate.innovation_cross_m) -
    options.innovation_yaw_penalty_weight * innovation_yaw_deg -
    options.initial_to_result_penalty_weight * candidate.initial_to_result_distance_m -
    options.covariance_condition_penalty_weight * condition_penalty;
  if (options.enable_gnss_weak_prior && candidate.has_gnss_weak_prior) {
    if (
      options.gnss_weak_prior_max_distance_m > 0.0 &&
      candidate.gnss_weak_prior_distance_m > options.gnss_weak_prior_max_distance_m) {
      score.reject_reason = "gnss_weak_prior_distance_too_large";
      return score;
    }
    score.total_score -=
      std::max(0.0, options.gnss_weak_prior_weight) *
      std::min(
        std::max(0.0, candidate.gnss_weak_prior_penalty),
        std::max(0.0, options.gnss_weak_prior_max_penalty));
  }
  if (runtime_candidate_fails_gnss_innovation_gate(candidate, options)) {
    score.reject_reason = "gnss_weak_prior_innovation_too_large";
    return score;
  }
  if (score.total_score < options.min_total_score) {
    score.reject_reason = "total_score_below_threshold";
  }
  return score;
}

inline bool is_far_runtime_candidate(
  const RuntimeCandidate & candidate, const RuntimeCandidateTierOptions & options)
{
  return std::abs(candidate.offset_along_m) > options.tier1_max_abs_along_m ||
         std::abs(candidate.offset_cross_m) > options.tier1_max_abs_cross_m ||
         std::abs(candidate.offset_yaw_deg) > options.tier1_max_abs_yaw_deg;
}

inline bool is_along_health_runtime_candidate(
  const RuntimeCandidate & candidate, const RuntimeCandidateTierOptions & options)
{
  constexpr double epsilon = 1e-6;
  return std::abs(candidate.offset_along_m) > epsilon &&
         std::abs(candidate.offset_along_m) <= options.tier1_max_abs_along_m &&
         std::abs(candidate.offset_cross_m) <= epsilon &&
         std::abs(candidate.offset_yaw_deg) <= epsilon;
}

inline const RuntimeCandidate * find_runtime_candidate_by_index(
  const std::vector<RuntimeCandidate> & candidates, const std::size_t index)
{
  const auto iter = std::find_if(
    candidates.begin(), candidates.end(),
    [index](const RuntimeCandidate & candidate) { return candidate.index == index; });
  return iter == candidates.end() ? nullptr : &(*iter);
}

inline const RuntimeCandidateScore * find_runtime_candidate_score_by_index(
  const std::vector<RuntimeCandidateScore> & scores, const std::size_t index)
{
  const auto iter = std::find_if(
    scores.begin(), scores.end(),
    [index](const RuntimeCandidateScore & score) { return score.candidate_index == index; });
  return iter == scores.end() ? nullptr : &(*iter);
}

inline double runtime_candidate_raw_score(
  const RuntimeCandidate & candidate, const RuntimeCandidateScoringOptions & options)
{
  const bool use_transform_probability =
    std::isfinite(options.min_transform_probability) &&
    !std::isfinite(options.min_nearest_voxel_transformation_likelihood);
  return use_transform_probability ? candidate.transform_probability
                                   : candidate.nearest_voxel_transformation_likelihood;
}

inline bool runtime_candidate_is_stable_for_recovery(
  const RuntimeCandidate & candidate, const RuntimeCandidateTierOptions & options)
{
  const bool far_candidate = is_far_runtime_candidate(candidate, options);
  const double innovation_xy =
    std::hypot(candidate.innovation_along_m, candidate.innovation_cross_m);
  const double result_from_initial_yaw_rad = std::atan2(
    std::sin(candidate.innovation_yaw_rad - candidate.offset_yaw_deg * M_PI / 180.0),
    std::cos(candidate.innovation_yaw_rad - candidate.offset_yaw_deg * M_PI / 180.0));
  const double stable_translation_m =
    far_candidate ? candidate.initial_to_result_distance_m : innovation_xy;
  const double stable_yaw_deg =
    (far_candidate ? std::abs(result_from_initial_yaw_rad) : std::abs(candidate.innovation_yaw_rad)) *
    180.0 / M_PI;
  return candidate.converged && stable_translation_m <= options.recovery_stable_max_innovation_m &&
         stable_yaw_deg <= options.recovery_stable_max_yaw_deg;
}

inline bool should_evaluate_far_runtime_tier(
  const std::vector<RuntimeCandidate> & evaluated_candidates,
  const RuntimeCandidateSelection & selection, const RuntimeCandidateTierOptions & options,
  const int rejected_scan_streak, const bool recovery_active, const double seconds_since_last_far_tier,
  const bool small_tier_ambiguous = false,
  const int scans_since_last_far_tier = std::numeric_limits<int>::max())
{
  (void)small_tier_ambiguous;
  if (
    options.recovery_far_tier_min_scan_interval > 0 &&
    scans_since_last_far_tier < options.recovery_far_tier_min_scan_interval) {
    return false;
  }
  if (!selection.has_selected_candidate) {
    return true;
  }
  if (
    options.tracking_far_tier_period_sec > 0.0 &&
    seconds_since_last_far_tier >= options.tracking_far_tier_period_sec) {
    return true;
  }
  const RuntimeCandidate * selected =
    find_runtime_candidate_by_index(evaluated_candidates, selection.selected_candidate_index);
  if (selected == nullptr) {
    return true;
  }
  if (rejected_scan_streak > 0 || recovery_active) {
    return options.recovery_far_tier_period_sec <= 0.0 ||
           seconds_since_last_far_tier >= options.recovery_far_tier_period_sec;
  }
  return false;
}

inline bool should_refresh_tracking_tier1(
  const double stamp_sec, const double last_tier1_stamp_sec, const double period_sec)
{
  return period_sec > 0.0 &&
         (last_tier1_stamp_sec < 0.0 || stamp_sec - last_tier1_stamp_sec >= period_sec);
}

inline RuntimeCandidateSelection select_runtime_candidate(
  const std::vector<RuntimeCandidate> & candidates,
  const RuntimeCandidateScoringOptions & options = RuntimeCandidateScoringOptions{})
{
  RuntimeCandidateSelection selection;
  selection.candidate_scores.reserve(candidates.size());

  double best_score = -std::numeric_limits<double>::infinity();
  for (const auto & candidate : candidates) {
    RuntimeCandidateScore score = score_runtime_candidate(candidate, options);
    if (score.reject_reason.empty() && score.total_score > best_score) {
      best_score = score.total_score;
      selection.has_selected_candidate = true;
      selection.selected_candidate_index = candidate.index;
    }
    selection.candidate_scores.push_back(score);
  }

  if (
    selection.has_selected_candidate && candidates.size() > 1 &&
    options.base_candidate_raw_score_margin > 0.0) {
    const RuntimeCandidate & base_candidate = candidates.front();
    const auto selected_it = std::find_if(
      candidates.begin(), candidates.end(), [&selection](const RuntimeCandidate & candidate) {
        return candidate.index == selection.selected_candidate_index;
      });
    const double base_raw_score = runtime_candidate_raw_score(base_candidate, options);
    const double selected_raw_score =
      selected_it == candidates.end()
        ? -std::numeric_limits<double>::infinity()
        : runtime_candidate_raw_score(*selected_it, options);
    const bool base_above_threshold =
      base_candidate.converged &&
      base_candidate.transform_probability >= options.min_transform_probability &&
      base_candidate.nearest_voxel_transformation_likelihood >=
        options.min_nearest_voxel_transformation_likelihood;
    if (
      base_above_threshold && base_candidate.index != selection.selected_candidate_index &&
      selected_raw_score - base_raw_score <= options.base_candidate_raw_score_margin) {
      selection.selected_candidate_index = base_candidate.index;
      for (auto & score : selection.candidate_scores) {
        if (score.candidate_index != base_candidate.index) {
          continue;
        }
        if (
          score.reject_reason != "not_converged" && score.reject_reason != "max_iteration" &&
          score.reject_reason != "score_below_threshold") {
          score.reject_reason.clear();
          if (!std::isfinite(score.total_score)) {
            score.total_score = base_raw_score;
          }
        }
        break;
      }
    }
  }

  if (
    selection.has_selected_candidate && candidates.size() > 1 &&
    options.raw_score_override_margin > 0.0) {
    const RuntimeCandidate * selected =
      find_runtime_candidate_by_index(candidates, selection.selected_candidate_index);
    const RuntimeCandidateScore * selected_score =
      find_runtime_candidate_score_by_index(selection.candidate_scores, selection.selected_candidate_index);
    const double selected_raw_score =
      selected == nullptr ? -std::numeric_limits<double>::infinity()
                          : runtime_candidate_raw_score(*selected, options);
    const double selected_total_score =
      selected_score == nullptr ? -std::numeric_limits<double>::infinity()
                                : selected_score->total_score;

    const RuntimeCandidate * best_raw_candidate = nullptr;
    double best_raw_score = selected_raw_score;
    for (const auto & score : selection.candidate_scores) {
      if (!score.reject_reason.empty() || !std::isfinite(score.total_score)) {
        continue;
      }
      const RuntimeCandidate * candidate =
        find_runtime_candidate_by_index(candidates, score.candidate_index);
      if (candidate == nullptr) {
        continue;
      }
      const double innovation_yaw_deg = std::abs(candidate->innovation_yaw_rad) * 180.0 / M_PI;
      if (
        options.raw_score_override_max_total_score_drop > 0.0 &&
        std::isfinite(selected_total_score) &&
        selected_total_score - score.total_score >
          options.raw_score_override_max_total_score_drop) {
        continue;
      }
      if (
        options.raw_score_override_max_abs_along_m > 0.0 &&
        std::abs(candidate->innovation_along_m) >
          options.raw_score_override_max_abs_along_m) {
        continue;
      }
      if (
        options.raw_score_override_max_abs_cross_m > 0.0 &&
        std::abs(candidate->innovation_cross_m) >
          options.raw_score_override_max_abs_cross_m) {
        continue;
      }
      if (
        options.raw_score_override_max_abs_yaw_deg > 0.0 &&
        innovation_yaw_deg > options.raw_score_override_max_abs_yaw_deg) {
        continue;
      }
      if (
        options.raw_score_override_max_initial_to_result_distance_m > 0.0 &&
        candidate->initial_to_result_distance_m >
          options.raw_score_override_max_initial_to_result_distance_m) {
        continue;
      }
      const double raw_score = runtime_candidate_raw_score(*candidate, options);
      if (
        std::isfinite(raw_score) &&
        raw_score >= selected_raw_score + options.raw_score_override_margin &&
        raw_score > best_raw_score) {
        best_raw_score = raw_score;
        best_raw_candidate = candidate;
      }
    }

    if (best_raw_candidate != nullptr) {
      selection.selected_candidate_index = best_raw_candidate->index;
    }
  }

  return selection;
}

inline RuntimeCandidateSpreadCovariance estimate_runtime_candidate_spread_covariance(
  const std::vector<RuntimeCandidate> & candidates, const RuntimeCandidateSelection & selection,
  const RuntimeCandidateScoringOptions & options, const double raw_score_margin)
{
  RuntimeCandidateSpreadCovariance spread;
  if (raw_score_margin < 0.0) {
    return spread;
  }

  double reference_raw_score = -std::numeric_limits<double>::infinity();
  if (selection.has_selected_candidate) {
    const RuntimeCandidate * selected =
      find_runtime_candidate_by_index(candidates, selection.selected_candidate_index);
    if (selected == nullptr) {
      return spread;
    }
    reference_raw_score = runtime_candidate_raw_score(*selected, options);
  } else {
    for (const auto & score : selection.candidate_scores) {
      const RuntimeCandidate * candidate =
        find_runtime_candidate_by_index(candidates, score.candidate_index);
      if (candidate == nullptr) {
        continue;
      }
      const double raw_score = runtime_candidate_raw_score(*candidate, options);
      if (std::isfinite(raw_score)) {
        reference_raw_score = std::max(reference_raw_score, raw_score);
      }
    }
  }
  if (!std::isfinite(reference_raw_score)) {
    return spread;
  }

  double min_along = std::numeric_limits<double>::infinity();
  double max_along = -std::numeric_limits<double>::infinity();
  double min_cross = std::numeric_limits<double>::infinity();
  double max_cross = -std::numeric_limits<double>::infinity();
  for (const auto & score : selection.candidate_scores) {
    if (selection.has_selected_candidate && !score.reject_reason.empty()) {
      continue;
    }
    const RuntimeCandidate * candidate =
      find_runtime_candidate_by_index(candidates, score.candidate_index);
    if (candidate == nullptr || (selection.has_selected_candidate && !candidate->converged)) {
      continue;
    }
    const double raw_score = runtime_candidate_raw_score(*candidate, options);
    if (!std::isfinite(raw_score) || reference_raw_score - raw_score > raw_score_margin) {
      continue;
    }

    const double innovation_yaw_deg = std::abs(candidate->innovation_yaw_rad) * 180.0 / M_PI;
    if (
      options.raw_score_override_max_abs_along_m > 0.0 &&
      std::abs(candidate->innovation_along_m) > options.raw_score_override_max_abs_along_m) {
      continue;
    }
    if (
      options.raw_score_override_max_abs_cross_m > 0.0 &&
      std::abs(candidate->innovation_cross_m) > options.raw_score_override_max_abs_cross_m) {
      continue;
    }
    if (
      options.raw_score_override_max_abs_yaw_deg > 0.0 &&
      innovation_yaw_deg > options.raw_score_override_max_abs_yaw_deg) {
      continue;
    }
    if (
      options.raw_score_override_max_initial_to_result_distance_m > 0.0 &&
      candidate->initial_to_result_distance_m >
        options.raw_score_override_max_initial_to_result_distance_m) {
      continue;
    }

    min_along = std::min(min_along, candidate->innovation_along_m);
    max_along = std::max(max_along, candidate->innovation_along_m);
    min_cross = std::min(min_cross, candidate->innovation_cross_m);
    max_cross = std::max(max_cross, candidate->innovation_cross_m);
    ++spread.contender_count;
  }

  if (spread.contender_count < 2) {
    return spread;
  }
  const double along_span_m = std::max(0.0, max_along - min_along);
  const double cross_span_m = std::max(0.0, max_cross - min_cross);
  spread.along_variance_m2 = 0.25 * along_span_m * along_span_m;
  spread.cross_variance_m2 = 0.25 * cross_span_m * cross_span_m;
  spread.ambiguous = spread.along_variance_m2 > 0.0 || spread.cross_variance_m2 > 0.0;
  return spread;
}

inline bool runtime_small_tier_is_ambiguous(
  const std::vector<RuntimeCandidate> & candidates, const RuntimeCandidateSelection & selection,
  const RuntimeCandidateTierOptions & options)
{
  if (!selection.has_selected_candidate || options.ambiguity_score_margin < 0.0) {
    return false;
  }

  double best_score = -std::numeric_limits<double>::infinity();
  double second_score = -std::numeric_limits<double>::infinity();
  for (const auto & score : selection.candidate_scores) {
    if (!score.reject_reason.empty() || !std::isfinite(score.total_score)) {
      continue;
    }
    if (score.total_score > best_score) {
      second_score = best_score;
      best_score = score.total_score;
    } else if (score.total_score > second_score) {
      second_score = score.total_score;
    }
  }
  if (!std::isfinite(best_score) || !std::isfinite(second_score)) {
    return false;
  }
  if (best_score - second_score > options.ambiguity_score_margin) {
    return false;
  }

  double min_along = std::numeric_limits<double>::infinity();
  double max_along = -std::numeric_limits<double>::infinity();
  int contender_count = 0;
  for (const auto & score : selection.candidate_scores) {
    if (
      !score.reject_reason.empty() || !std::isfinite(score.total_score) ||
      best_score - score.total_score > options.ambiguity_score_margin) {
      continue;
    }
    const RuntimeCandidate * candidate = find_runtime_candidate_by_index(candidates, score.candidate_index);
    if (candidate == nullptr || is_far_runtime_candidate(*candidate, options)) {
      continue;
    }
    min_along = std::min(min_along, candidate->innovation_along_m);
    max_along = std::max(max_along, candidate->innovation_along_m);
    ++contender_count;
  }
  return contender_count >= 2 && max_along - min_along >= options.ambiguity_along_spread_m;
}

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__RUNTIME_MULTISTART_HPP_
