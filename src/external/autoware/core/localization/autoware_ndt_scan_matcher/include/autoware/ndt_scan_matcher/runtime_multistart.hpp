#ifndef AUTOWARE__NDT_SCAN_MATCHER__RUNTIME_MULTISTART_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__RUNTIME_MULTISTART_HPP_

#include <algorithm>
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
  double execution_time_ms{};
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

struct RuntimeCandidateSpreadCovariance
{
  bool ambiguous{};
  int contender_count{};
  double along_variance_m2{};
  double cross_variance_m2{};
};

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
  if (!selection.has_selected_candidate || raw_score_margin < 0.0) {
    return spread;
  }

  const RuntimeCandidate * selected =
    find_runtime_candidate_by_index(candidates, selection.selected_candidate_index);
  if (selected == nullptr) {
    return spread;
  }
  const double selected_raw_score = runtime_candidate_raw_score(*selected, options);
  if (!std::isfinite(selected_raw_score)) {
    return spread;
  }

  double min_along = std::numeric_limits<double>::infinity();
  double max_along = -std::numeric_limits<double>::infinity();
  double min_cross = std::numeric_limits<double>::infinity();
  double max_cross = -std::numeric_limits<double>::infinity();
  for (const auto & score : selection.candidate_scores) {
    if (!score.reject_reason.empty()) {
      continue;
    }
    const RuntimeCandidate * candidate =
      find_runtime_candidate_by_index(candidates, score.candidate_index);
    if (candidate == nullptr || !candidate->converged) {
      continue;
    }
    const double raw_score = runtime_candidate_raw_score(*candidate, options);
    if (!std::isfinite(raw_score) || selected_raw_score - raw_score > raw_score_margin) {
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
