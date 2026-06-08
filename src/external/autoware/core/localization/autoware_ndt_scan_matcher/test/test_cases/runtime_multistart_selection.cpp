#include <autoware/ndt_scan_matcher/runtime_multistart.hpp>
#include <autoware/ndt_scan_matcher/time_offset.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace autoware::ndt_scan_matcher
{
namespace
{

RuntimeCandidate make_candidate(
  const std::size_t index, const double nvtl, const double innovation_xy_m,
  const double innovation_yaw_deg, const double initial_to_result_m)
{
  RuntimeCandidate candidate;
  candidate.index = index;
  candidate.converged = true;
  candidate.iteration_num = 12;
  candidate.max_iterations = 60;
  candidate.transform_probability = 3.0;
  candidate.nearest_voxel_transformation_likelihood = nvtl;
  candidate.initial_to_result_distance_m = initial_to_result_m;
  candidate.innovation_along_m = innovation_xy_m;
  candidate.innovation_cross_m = 0.0;
  candidate.innovation_yaw_rad = innovation_yaw_deg * M_PI / 180.0;
  candidate.covariance_condition_number = 4.0;
  return candidate;
}

RuntimeCandidate make_offset_candidate(
  const std::size_t index, const double offset_along_m, const double offset_cross_m,
  const double offset_yaw_deg, const double innovation_xy_m = 0.2,
  const double innovation_yaw_deg = 0.5)
{
  RuntimeCandidate candidate = make_candidate(index, 3.35, innovation_xy_m, innovation_yaw_deg, 0.4);
  candidate.offset_along_m = offset_along_m;
  candidate.offset_cross_m = offset_cross_m;
  candidate.offset_yaw_deg = offset_yaw_deg;
  return candidate;
}

TEST(RuntimeMultistartSelection, prefersMotionConsistentBasinOverHigherRawScore)
{
  const RuntimeCandidate near_basin = make_candidate(0, 3.35, 0.18, 0.4, 0.22);
  const RuntimeCandidate wrong_basin = make_candidate(1, 3.85, 2.4, 7.0, 2.8);

  RuntimeCandidateScoringOptions options;
  options.min_nearest_voxel_transformation_likelihood = 2.3;
  options.max_initial_to_result_distance_m = 5.0;
  options.max_iteration_num = 60;

  const RuntimeCandidateSelection selection =
    select_runtime_candidate({near_basin, wrong_basin}, options);

  ASSERT_TRUE(selection.has_selected_candidate);
  ASSERT_EQ(selection.selected_candidate_index, 0U);
  ASSERT_EQ(selection.candidate_scores.size(), 2U);
  EXPECT_GT(selection.candidate_scores[0].total_score, selection.candidate_scores[1].total_score);
  EXPECT_EQ(selection.candidate_scores[0].reject_reason, "");
  EXPECT_EQ(selection.candidate_scores[1].reject_reason, "");
}

TEST(RuntimeMultistartSelection, rejectsUnconvergedAndBelowThresholdCandidates)
{
  RuntimeCandidate unconverged = make_candidate(0, 4.0, 0.1, 0.1, 0.1);
  unconverged.converged = false;
  RuntimeCandidate low_score = make_candidate(1, 2.0, 0.1, 0.1, 0.1);

  RuntimeCandidateScoringOptions options;
  options.min_nearest_voxel_transformation_likelihood = 2.3;
  options.max_initial_to_result_distance_m = 5.0;
  options.max_iteration_num = 60;

  const RuntimeCandidateSelection selection =
    select_runtime_candidate({unconverged, low_score}, options);

  EXPECT_FALSE(selection.has_selected_candidate);
  ASSERT_EQ(selection.candidate_scores.size(), 2U);
  EXPECT_EQ(selection.candidate_scores[0].reject_reason, "not_converged");
  EXPECT_EQ(selection.candidate_scores[1].reject_reason, "score_below_threshold");
}

TEST(RuntimeMultistartSelection, rejectsCandidatesThatJumpTooFarFromPrior)
{
  RuntimeCandidate near_basin = make_candidate(0, 3.1, 0.3, 1.0, 0.2);
  RuntimeCandidate far_recovery_offset = make_candidate(1, 3.5, 10.0, 1.0, 0.2);

  RuntimeCandidateScoringOptions options;
  options.min_nearest_voxel_transformation_likelihood = 2.3;
  options.max_initial_to_result_distance_m = 5.0;
  options.max_prior_innovation_m = 3.0;
  options.max_prior_yaw_deg = 10.0;
  options.max_iteration_num = 60;

  const RuntimeCandidateSelection selection =
    select_runtime_candidate({near_basin, far_recovery_offset}, options);

  ASSERT_TRUE(selection.has_selected_candidate);
  EXPECT_EQ(selection.selected_candidate_index, 0U);
  ASSERT_EQ(selection.candidate_scores.size(), 2U);
  EXPECT_EQ(selection.candidate_scores[1].reject_reason, "prior_innovation_too_large");
}

TEST(RuntimeMultistartSelection, AllowsAlongRecoveryWhileRejectingCrossJump)
{
  RuntimeCandidate along_recovery = make_candidate(0, 3.4, 1.4, 0.2, 1.0);
  RuntimeCandidate cross_jump = make_candidate(1, 3.5, 0.1, 0.2, 1.0);
  cross_jump.innovation_cross_m = 1.4;

  RuntimeCandidateScoringOptions options;
  options.min_nearest_voxel_transformation_likelihood = 2.3;
  options.max_initial_to_result_distance_m = 5.0;
  options.max_prior_along_m = 1.8;
  options.max_prior_cross_m = 0.85;
  options.max_iteration_num = 60;

  const RuntimeCandidateSelection selection =
    select_runtime_candidate({along_recovery, cross_jump}, options);

  ASSERT_TRUE(selection.has_selected_candidate);
  EXPECT_EQ(selection.selected_candidate_index, 0U);
  ASSERT_EQ(selection.candidate_scores.size(), 2U);
  EXPECT_EQ(selection.candidate_scores[0].reject_reason, "");
  EXPECT_EQ(selection.candidate_scores[1].reject_reason, "prior_cross_too_large");
}

TEST(RuntimeMultistartSelection, AllowsAlongCorrectionWhenCrossAndYawRemainConsistent)
{
  RuntimeCandidate yaw_offset = make_candidate(14, 3.107798, -0.232533, -2.187787, 0.358382);
  yaw_offset.transform_probability = 7.545674;
  yaw_offset.iteration_num = 11;
  yaw_offset.innovation_cross_m = 0.117958;

  RuntimeCandidate along_correction = make_candidate(3, 3.167573, 0.937389, -0.026735, 0.29955);
  along_correction.transform_probability = 7.511163;
  along_correction.iteration_num = 4;
  along_correction.innovation_cross_m = 0.058626;

  RuntimeCandidateScoringOptions options;
  options.min_nearest_voxel_transformation_likelihood = 2.3;
  options.max_initial_to_result_distance_m = 5.0;
  options.max_iteration_num = 100;

  const RuntimeCandidateSelection selection =
    select_runtime_candidate({yaw_offset, along_correction}, options);

  ASSERT_TRUE(selection.has_selected_candidate);
  EXPECT_EQ(selection.selected_candidate_index, 3U);
}

TEST(RuntimeMultistartSelection, MaxIterationGateIsDisabledWhenOptionIsZero)
{
  RuntimeCandidate oscillation_accepted = make_candidate(0, 3.2, 0.2, 0.2, 0.2);
  oscillation_accepted.iteration_num = oscillation_accepted.max_iterations;
  oscillation_accepted.converged = true;

  RuntimeCandidateScoringOptions options;
  options.min_nearest_voxel_transformation_likelihood = 2.3;
  options.max_initial_to_result_distance_m = 5.0;
  options.max_iteration_num = 0;

  const RuntimeCandidateSelection selection =
    select_runtime_candidate({oscillation_accepted}, options);

  ASSERT_TRUE(selection.has_selected_candidate);
  EXPECT_EQ(selection.selected_candidate_index, 0U);
  ASSERT_EQ(selection.candidate_scores.size(), 1U);
  EXPECT_EQ(selection.candidate_scores[0].reject_reason, "");
}

TEST(RuntimeMultistartSelection, AcceptsCandidateAlreadyMarkedConvergedAtMaxIteration)
{
  RuntimeCandidate oscillation_accepted = make_candidate(0, 3.2, 0.2, 0.2, 0.2);
  oscillation_accepted.iteration_num = oscillation_accepted.max_iterations;
  oscillation_accepted.converged = true;

  RuntimeCandidateScoringOptions options;
  options.min_nearest_voxel_transformation_likelihood = 2.3;
  options.max_initial_to_result_distance_m = 5.0;
  options.max_iteration_num = oscillation_accepted.max_iterations;

  const RuntimeCandidateSelection selection =
    select_runtime_candidate({oscillation_accepted}, options);

  ASSERT_TRUE(selection.has_selected_candidate);
  EXPECT_EQ(selection.selected_candidate_index, 0U);
  ASSERT_EQ(selection.candidate_scores.size(), 1U);
  EXPECT_EQ(selection.candidate_scores[0].reject_reason, "");
}

TEST(RuntimeMultistartSelection, SingleStartOriginalSemanticsIgnoresRuntimeSelectionThresholds)
{
  RuntimeCandidate candidate = make_candidate(0, 1.55, 0.03, 0.01, 0.09);
  candidate.transform_probability = 3.26;
  candidate.iteration_num = 3;
  candidate.converged = true;

  RuntimeCandidateScoringOptions options;
  options.min_transform_probability = 9.0;
  options.min_nearest_voxel_transformation_likelihood = 2.3;
  options.max_initial_to_result_distance_m = 0.01;
  options.max_prior_innovation_m = 0.01;
  options.max_prior_along_m = 0.01;
  options.max_prior_cross_m = 0.01;
  options.max_prior_yaw_deg = 0.01;
  options.min_total_score = 1000.0;
  options.max_iteration_num = 3;

  disable_runtime_selection_gates_for_single_start(options);
  const RuntimeCandidateSelection selection = select_runtime_candidate({candidate}, options);

  ASSERT_TRUE(selection.has_selected_candidate);
  EXPECT_EQ(selection.selected_candidate_index, 0U);
  ASSERT_EQ(selection.candidate_scores.size(), 1U);
  EXPECT_EQ(selection.candidate_scores[0].reject_reason, "");
}

TEST(RuntimeMultistartSelection, KeepsBaseCandidateWhenRawScoreGainIsSmall)
{
  RuntimeCandidate base = make_candidate(0, 3.35, 1.2, 2.0, 1.1);
  RuntimeCandidate smooth_wrong_basin = make_candidate(1, 3.40, 0.2, 0.2, 0.2);

  RuntimeCandidateScoringOptions options;
  options.min_nearest_voxel_transformation_likelihood = 2.3;
  options.max_initial_to_result_distance_m = 5.0;
  options.max_prior_innovation_m = 1.0;
  options.max_prior_yaw_deg = 6.0;
  options.max_iteration_num = 60;
  options.base_candidate_raw_score_margin = 0.10;

  const RuntimeCandidateSelection selection =
    select_runtime_candidate({base, smooth_wrong_basin}, options);

  ASSERT_TRUE(selection.has_selected_candidate);
  EXPECT_EQ(selection.selected_candidate_index, 0U);
  ASSERT_EQ(selection.candidate_scores.size(), 2U);
  EXPECT_EQ(selection.candidate_scores[0].reject_reason, "");
}

TEST(RuntimeMultistartSelection, AllowsNonBaseCandidateWhenRawScoreGainIsLarge)
{
  RuntimeCandidate base = make_candidate(0, 3.35, 1.2, 2.0, 1.1);
  RuntimeCandidate clearly_better_basin = make_candidate(1, 3.80, 0.2, 0.2, 0.2);

  RuntimeCandidateScoringOptions options;
  options.min_nearest_voxel_transformation_likelihood = 2.3;
  options.max_initial_to_result_distance_m = 5.0;
  options.max_prior_innovation_m = 1.0;
  options.max_prior_yaw_deg = 6.0;
  options.max_iteration_num = 60;
  options.base_candidate_raw_score_margin = 0.10;

  const RuntimeCandidateSelection selection =
    select_runtime_candidate({base, clearly_better_basin}, options);

  ASSERT_TRUE(selection.has_selected_candidate);
  EXPECT_EQ(selection.selected_candidate_index, 1U);
  ASSERT_EQ(selection.candidate_scores.size(), 2U);
  EXPECT_EQ(selection.candidate_scores[0].reject_reason, "prior_innovation_too_large");
}

TEST(NDTOutputTimeOffset, AppliesSmallSignedOffsetToPublishedPoseStamp)
{
  const rclcpp::Time sensor_time(123, 450000000, RCL_ROS_TIME);

  const rclcpp::Time earlier = apply_output_pose_time_offset(sensor_time, -0.0075);
  const rclcpp::Time later = apply_output_pose_time_offset(sensor_time, 0.010);

  EXPECT_NEAR((earlier - sensor_time).seconds(), -0.0075, 1e-9);
  EXPECT_NEAR((later - sensor_time).seconds(), 0.010, 1e-9);
}

TEST(RuntimeMultistartSelection, RawScoreOverrideEscapesContaminatedAlongPrior)
{
  RuntimeCandidate prior_consistent_wrong = make_offset_candidate(4, -1.0, 0.0, 0.0);
  prior_consistent_wrong.nearest_voxel_transformation_likelihood = 3.077;
  prior_consistent_wrong.transform_probability = 6.648;
  prior_consistent_wrong.initial_to_result_distance_m = 0.404;
  prior_consistent_wrong.innovation_along_m = -0.955;
  prior_consistent_wrong.innovation_cross_m = -0.343;
  prior_consistent_wrong.innovation_yaw_rad = -0.280 * M_PI / 180.0;

  RuntimeCandidate map_consistent_candidate = make_offset_candidate(13, 0.0, 0.0, 2.0);
  map_consistent_candidate.nearest_voxel_transformation_likelihood = 3.172;
  map_consistent_candidate.transform_probability = 7.056;
  map_consistent_candidate.initial_to_result_distance_m = 2.466;
  map_consistent_candidate.innovation_along_m = 2.379;
  map_consistent_candidate.innovation_cross_m = -0.415;
  map_consistent_candidate.innovation_yaw_rad = 1.499 * M_PI / 180.0;

  RuntimeCandidateScoringOptions options;
  options.min_nearest_voxel_transformation_likelihood = 2.3;
  options.max_initial_to_result_distance_m = 5.0;
  options.max_prior_innovation_m = 12.0;
  options.max_prior_yaw_deg = 6.0;
  options.max_iteration_num = 100;
  options.raw_score_override_margin = 0.03;
  options.raw_score_override_max_total_score_drop = 0.75;
  options.raw_score_override_max_abs_along_m = 3.0;
  options.raw_score_override_max_abs_cross_m = 0.85;
  options.raw_score_override_max_abs_yaw_deg = 3.0;
  options.raw_score_override_max_initial_to_result_distance_m = 3.0;

  const RuntimeCandidateSelection selection =
    select_runtime_candidate({prior_consistent_wrong, map_consistent_candidate}, options);

  ASSERT_TRUE(selection.has_selected_candidate);
  EXPECT_EQ(selection.selected_candidate_index, 13U);
}

TEST(RuntimeMultistartSelection, RawScoreOverrideCanPromoteSmallAlongRecoveryCandidate)
{
  RuntimeCandidate prior_consistent_wrong = make_offset_candidate(0, 0.0, 0.0, 0.0);
  prior_consistent_wrong.nearest_voxel_transformation_likelihood = 1.645764;
  prior_consistent_wrong.transform_probability = 3.2;
  prior_consistent_wrong.initial_to_result_distance_m = 0.24;
  prior_consistent_wrong.innovation_along_m = 0.24;
  prior_consistent_wrong.innovation_cross_m = 0.01;
  prior_consistent_wrong.innovation_yaw_rad = 0.05 * M_PI / 180.0;

  RuntimeCandidate map_consistent_recovery = make_offset_candidate(3, 1.0, 0.0, 0.0);
  map_consistent_recovery.nearest_voxel_transformation_likelihood = 1.676081;
  map_consistent_recovery.transform_probability = 3.2;
  map_consistent_recovery.initial_to_result_distance_m = 1.04;
  map_consistent_recovery.innovation_along_m = 1.04;
  map_consistent_recovery.innovation_cross_m = 0.02;
  map_consistent_recovery.innovation_yaw_rad = 0.05 * M_PI / 180.0;

  RuntimeCandidateScoringOptions options;
  options.min_nearest_voxel_transformation_likelihood = 1.0;
  options.max_initial_to_result_distance_m = 5.0;
  options.max_prior_innovation_m = 12.0;
  options.max_prior_yaw_deg = 6.0;
  options.max_iteration_num = 100;
  options.raw_score_override_margin = 0.02;
  options.raw_score_override_max_total_score_drop = 0.75;
  options.raw_score_override_max_abs_along_m = 3.0;
  options.raw_score_override_max_abs_cross_m = 1.5;
  options.raw_score_override_max_abs_yaw_deg = 3.0;
  options.raw_score_override_max_initial_to_result_distance_m = 3.0;

  const RuntimeCandidateSelection selection =
    select_runtime_candidate({prior_consistent_wrong, map_consistent_recovery}, options);

  ASSERT_TRUE(selection.has_selected_candidate);
  EXPECT_EQ(selection.selected_candidate_index, 3U);
}

TEST(RuntimeMultistartSelection, RawScoreOverrideRejectsUnboundedAlongOutlier)
{
  RuntimeCandidate prior_consistent = make_offset_candidate(0, 0.0, 0.0, 0.0);
  prior_consistent.nearest_voxel_transformation_likelihood = 3.10;
  prior_consistent.transform_probability = 7.4;
  prior_consistent.initial_to_result_distance_m = 0.4;
  prior_consistent.innovation_along_m = 0.2;
  prior_consistent.innovation_cross_m = 0.1;
  prior_consistent.innovation_yaw_rad = 0.2 * M_PI / 180.0;

  RuntimeCandidate raw_high_far_outlier = make_offset_candidate(7, 5.0, 0.0, 0.0);
  raw_high_far_outlier.nearest_voxel_transformation_likelihood = 3.30;
  raw_high_far_outlier.transform_probability = 7.5;
  raw_high_far_outlier.initial_to_result_distance_m = 1.0;
  raw_high_far_outlier.innovation_along_m = 5.4;
  raw_high_far_outlier.innovation_cross_m = 0.1;
  raw_high_far_outlier.innovation_yaw_rad = 1.0 * M_PI / 180.0;

  RuntimeCandidateScoringOptions options;
  options.min_nearest_voxel_transformation_likelihood = 2.3;
  options.max_initial_to_result_distance_m = 5.0;
  options.max_prior_innovation_m = 12.0;
  options.max_prior_yaw_deg = 6.0;
  options.max_iteration_num = 100;
  options.raw_score_override_margin = 0.03;
  options.raw_score_override_max_total_score_drop = 0.75;
  options.raw_score_override_max_abs_along_m = 3.0;
  options.raw_score_override_max_abs_cross_m = 0.85;
  options.raw_score_override_max_abs_yaw_deg = 3.0;
  options.raw_score_override_max_initial_to_result_distance_m = 3.0;

  const RuntimeCandidateSelection selection =
    select_runtime_candidate({prior_consistent, raw_high_far_outlier}, options);

  ASSERT_TRUE(selection.has_selected_candidate);
  EXPECT_EQ(selection.selected_candidate_index, 0U);
}

TEST(RuntimeMultistartSelection, CandidateSpreadInflatesAlongCovarianceWhenRawScoresAreAmbiguous)
{
  RuntimeCandidate selected = make_offset_candidate(0, 0.0, 0.0, 0.0);
  selected.nearest_voxel_transformation_likelihood = 1.68;
  selected.innovation_along_m = 0.0;
  selected.innovation_cross_m = 0.0;

  RuntimeCandidate competing_forward = make_offset_candidate(3, 1.0, 0.0, 0.0);
  competing_forward.nearest_voxel_transformation_likelihood = 1.66;
  competing_forward.innovation_along_m = 1.2;
  competing_forward.innovation_cross_m = 0.1;

  RuntimeCandidate competing_backward = make_offset_candidate(4, -1.0, 0.0, 0.0);
  competing_backward.nearest_voxel_transformation_likelihood = 1.65;
  competing_backward.innovation_along_m = -1.0;
  competing_backward.innovation_cross_m = -0.1;

  RuntimeCandidateScoringOptions options;
  options.min_nearest_voxel_transformation_likelihood = 1.0;
  options.max_initial_to_result_distance_m = 5.0;
  options.max_prior_innovation_m = 12.0;
  options.max_prior_yaw_deg = 6.0;
  options.raw_score_override_margin = 0.02;
  options.raw_score_override_max_total_score_drop = 0.75;
  options.raw_score_override_max_abs_along_m = 2.5;
  options.raw_score_override_max_abs_cross_m = 0.85;
  options.raw_score_override_max_abs_yaw_deg = 3.0;
  options.raw_score_override_max_initial_to_result_distance_m = 2.5;

  const std::vector<RuntimeCandidate> candidates{
    selected, competing_forward, competing_backward};
  const RuntimeCandidateSelection selection = select_runtime_candidate(candidates, options);
  const RuntimeCandidateSpreadCovariance spread =
    estimate_runtime_candidate_spread_covariance(candidates, selection, options, 0.05);

  ASSERT_TRUE(selection.has_selected_candidate);
  EXPECT_TRUE(spread.ambiguous);
  EXPECT_GE(spread.along_variance_m2, 1.0);
  EXPECT_LT(spread.cross_variance_m2, 0.1);
}

TEST(RuntimeMultistartTiering, SplitsTrackingSmallGridFromFarRecoveryGrid)
{
  RuntimeCandidateTierOptions options;
  options.tier1_max_abs_along_m = 1.0;
  options.tier1_max_abs_cross_m = 0.75;
  options.tier1_max_abs_yaw_deg = 2.0;

  EXPECT_FALSE(is_far_runtime_candidate(make_offset_candidate(0, 0.0, 0.0, 0.0), options));
  EXPECT_FALSE(is_far_runtime_candidate(make_offset_candidate(1, 1.0, 0.0, 0.0), options));
  EXPECT_FALSE(is_far_runtime_candidate(make_offset_candidate(2, 0.0, 0.75, 0.0), options));
  EXPECT_FALSE(is_far_runtime_candidate(make_offset_candidate(4, 0.0, 0.0, 2.0), options));

  EXPECT_TRUE(is_far_runtime_candidate(make_offset_candidate(5, 2.0, 0.0, 0.0), options));
  EXPECT_TRUE(is_far_runtime_candidate(make_offset_candidate(6, 0.0, 1.5, 0.0), options));
  EXPECT_TRUE(is_far_runtime_candidate(make_offset_candidate(7, 0.0, 0.0, 5.0), options));
}

TEST(RuntimeMultistartTiering, TreatsFiveDegreeYawProbeAsRecoveryTierByDefault)
{
  RuntimeCandidateTierOptions options;

  EXPECT_FALSE(is_far_runtime_candidate(make_offset_candidate(13, 0.0, 0.0, 2.0), options));
  EXPECT_TRUE(is_far_runtime_candidate(make_offset_candidate(15, 0.0, 0.0, 5.0), options));
}

TEST(RuntimeMultistartTiering, ClassifiesAlongHealthProbeSubset)
{
  RuntimeCandidateTierOptions options;
  options.tier1_max_abs_along_m = 1.0;
  options.tier1_max_abs_cross_m = 0.75;
  options.tier1_max_abs_yaw_deg = 5.0;

  EXPECT_TRUE(is_along_health_runtime_candidate(make_offset_candidate(1, 0.5, 0.0, 0.0), options));
  EXPECT_TRUE(is_along_health_runtime_candidate(make_offset_candidate(2, -1.0, 0.0, 0.0), options));
  EXPECT_FALSE(is_along_health_runtime_candidate(make_offset_candidate(0, 0.0, 0.0, 0.0), options));
  EXPECT_FALSE(is_along_health_runtime_candidate(make_offset_candidate(11, 0.0, 0.75, 0.0), options));
  EXPECT_FALSE(is_along_health_runtime_candidate(make_offset_candidate(13, 0.0, 0.0, 2.0), options));
  EXPECT_FALSE(is_along_health_runtime_candidate(make_offset_candidate(5, 2.0, 0.0, 0.0), options));
}

TEST(RuntimeMultistartTiering, RunsFarTierOnlyWhenSmallTierCannotVerifyTracking)
{
  RuntimeCandidateTierOptions tier_options;
  tier_options.recovery_stable_max_innovation_m = 1.0;
  tier_options.recovery_stable_max_yaw_deg = 5.0;
  tier_options.ambiguity_score_margin = 0.15;
  tier_options.ambiguity_along_spread_m = 1.5;
  tier_options.recovery_far_tier_min_scan_interval = 10;

  RuntimeCandidateScoringOptions scoring_options;
  scoring_options.min_nearest_voxel_transformation_likelihood = 2.3;
  scoring_options.max_initial_to_result_distance_m = 5.0;
  scoring_options.max_iteration_num = 60;

  const RuntimeCandidate stable_small = make_offset_candidate(0, 0.5, 0.0, 0.0, 0.4, 1.0);
  const RuntimeCandidateSelection stable_selection =
    select_runtime_candidate({stable_small}, scoring_options);
  EXPECT_FALSE(
    should_evaluate_far_runtime_tier({stable_small}, stable_selection, tier_options, 0, false, 2.0));

  RuntimeCandidate unstable_small = make_offset_candidate(0, 0.5, 0.0, 0.0, 2.2, 1.0);
  const RuntimeCandidateSelection unstable_selection =
    select_runtime_candidate({unstable_small}, scoring_options);
  EXPECT_FALSE(
    should_evaluate_far_runtime_tier(
      {unstable_small}, unstable_selection, tier_options, 0, false, 2.0));

  RuntimeCandidate rejected_small = make_offset_candidate(0, 0.5, 0.0, 0.0, 0.4, 1.0);
  rejected_small.converged = false;
  const RuntimeCandidateSelection rejected_selection =
    select_runtime_candidate({rejected_small}, scoring_options);
  EXPECT_TRUE(
    should_evaluate_far_runtime_tier(
      {rejected_small}, rejected_selection, tier_options, 0, false, 2.0));

  EXPECT_TRUE(
    should_evaluate_far_runtime_tier(
      {stable_small}, stable_selection, tier_options, 2, false, 2.0, false, 10));
  EXPECT_TRUE(
    should_evaluate_far_runtime_tier(
      {stable_small}, stable_selection, tier_options, 0, true, 2.0, false, 10));
  EXPECT_FALSE(
    should_evaluate_far_runtime_tier(
      {stable_small}, stable_selection, tier_options, 0, true, 0.2, false, 10));
  EXPECT_FALSE(
    should_evaluate_far_runtime_tier(
      {stable_small}, stable_selection, tier_options, 0, true, 2.0, false, 3));
}

TEST(RuntimeMultistartTiering, PeriodicallyProbesFarTierWhileTrackingWhenConfigured)
{
  RuntimeCandidateTierOptions tier_options;
  tier_options.tracking_far_tier_period_sec = 5.0;
  tier_options.recovery_far_tier_min_scan_interval = 10;

  RuntimeCandidate stable_small = make_offset_candidate(0, 0.0, 0.0, 0.0, 0.2, 0.1);
  stable_small.nearest_voxel_transformation_likelihood = 3.5;

  RuntimeCandidateScoringOptions scoring_options;
  scoring_options.min_nearest_voxel_transformation_likelihood = 2.3;
  scoring_options.max_initial_to_result_distance_m = 5.0;
  scoring_options.max_iteration_num = 100;

  const RuntimeCandidateSelection stable_selection =
    select_runtime_candidate({stable_small}, scoring_options);

  ASSERT_TRUE(stable_selection.has_selected_candidate);
  EXPECT_FALSE(
    should_evaluate_far_runtime_tier(
      {stable_small}, stable_selection, tier_options, 0, false, 4.9, false, 50));
  EXPECT_FALSE(
    should_evaluate_far_runtime_tier(
      {stable_small}, stable_selection, tier_options, 0, false, 5.0, false, 9));
  EXPECT_TRUE(
    should_evaluate_far_runtime_tier(
      {stable_small}, stable_selection, tier_options, 0, false, 5.0, false, 10));

  RuntimeCandidateTierOptions default_options;
  default_options.recovery_far_tier_min_scan_interval = 10;
  EXPECT_FALSE(
    should_evaluate_far_runtime_tier(
      {stable_small}, stable_selection, default_options, 0, false, 60.0, false, 100));
}

TEST(RuntimeMultistartTiering, KeepsFarTierRecoveryOnlyWhenSmallTierIsAmbiguous)
{
  RuntimeCandidateTierOptions tier_options;
  tier_options.ambiguity_score_margin = 0.15;
  tier_options.ambiguity_along_spread_m = 1.5;

  RuntimeCandidate plus_basin = make_offset_candidate(3, 1.0, 0.0, 0.0, 1.02, 0.1);
  plus_basin.nearest_voxel_transformation_likelihood = 3.206;
  plus_basin.transform_probability = 7.94;
  plus_basin.initial_to_result_distance_m = 0.39;
  plus_basin.innovation_along_m = 1.02;

  RuntimeCandidate minus_basin = make_offset_candidate(4, -1.0, 0.0, 0.0, 0.85, 0.2);
  minus_basin.nearest_voxel_transformation_likelihood = 3.185;
  minus_basin.transform_probability = 7.84;
  minus_basin.initial_to_result_distance_m = 0.41;
  minus_basin.innovation_along_m = -0.81;

  RuntimeCandidateScoringOptions scoring_options;
  scoring_options.min_nearest_voxel_transformation_likelihood = 2.3;
  scoring_options.max_initial_to_result_distance_m = 5.0;
  scoring_options.max_iteration_num = 100;

  const std::vector<RuntimeCandidate> candidates{plus_basin, minus_basin};
  const RuntimeCandidateSelection selection = select_runtime_candidate(candidates, scoring_options);

  ASSERT_TRUE(selection.has_selected_candidate);
  EXPECT_TRUE(runtime_small_tier_is_ambiguous(candidates, selection, tier_options));
  EXPECT_FALSE(
    should_evaluate_far_runtime_tier(candidates, selection, tier_options, 0, false, 2.0, true, 10));
  EXPECT_TRUE(
    should_evaluate_far_runtime_tier(candidates, selection, tier_options, 0, true, 2.0, true, 10));
}

TEST(RuntimeMultistartTiering, DoesNotMarkClearSmallTierAsAmbiguous)
{
  RuntimeCandidateTierOptions tier_options;
  tier_options.ambiguity_score_margin = 0.15;
  tier_options.ambiguity_along_spread_m = 1.5;

  RuntimeCandidate clear_best = make_offset_candidate(1, 0.5, 0.0, 0.0, 0.3, 0.2);
  clear_best.nearest_voxel_transformation_likelihood = 3.8;
  RuntimeCandidate weaker = make_offset_candidate(2, -0.5, 0.0, 0.0, 0.4, 0.2);
  weaker.nearest_voxel_transformation_likelihood = 3.1;

  RuntimeCandidateScoringOptions scoring_options;
  scoring_options.min_nearest_voxel_transformation_likelihood = 2.3;
  scoring_options.max_initial_to_result_distance_m = 5.0;
  scoring_options.max_iteration_num = 100;

  const std::vector<RuntimeCandidate> candidates{clear_best, weaker};
  const RuntimeCandidateSelection selection = select_runtime_candidate(candidates, scoring_options);

  ASSERT_TRUE(selection.has_selected_candidate);
  EXPECT_FALSE(runtime_small_tier_is_ambiguous(candidates, selection, tier_options));
  EXPECT_FALSE(
    should_evaluate_far_runtime_tier(candidates, selection, tier_options, 0, false, 2.0, false, 10));
}

TEST(RuntimeMultistartRecoveryState, DoesNotKeepFarTierActiveAfterUnstableSelectedCandidate)
{
  const RuntimeRecoveryState state = update_runtime_recovery_state(
    true, false, true, true, 0, 0, 3);

  EXPECT_FALSE(state.recovery_active);
  EXPECT_FALSE(state.recovery_verified);
  EXPECT_EQ(state.recovery_stable_frames, 0);
  EXPECT_EQ(state.rejected_scan_streak, 0);
}

TEST(RuntimeMultistartRecoveryState, KeepsRecoveryActiveWithoutSelectedCandidate)
{
  const RuntimeRecoveryState state = update_runtime_recovery_state(
    false, false, true, false, 0, 2, 3);

  EXPECT_TRUE(state.recovery_active);
  EXPECT_FALSE(state.recovery_verified);
  EXPECT_EQ(state.recovery_stable_frames, 0);
  EXPECT_EQ(state.rejected_scan_streak, 3);
}

TEST(RuntimeMultistartRecoveryState, VerifiesRecoveryAfterStableSelectedFrames)
{
  const RuntimeRecoveryState state = update_runtime_recovery_state(
    true, true, true, true, 2, 0, 3);

  EXPECT_FALSE(state.recovery_active);
  EXPECT_TRUE(state.recovery_verified);
  EXPECT_EQ(state.recovery_stable_frames, 3);
  EXPECT_EQ(state.rejected_scan_streak, 0);
}

TEST(RuntimeMultistartRecoveryState, TreatsFarCandidateStableWhenNdtStaysNearItsOwnInitialPose)
{
  RuntimeCandidate far_recovery = make_offset_candidate(12, 12.0, -15.0, 0.0);
  far_recovery.initial_to_result_distance_m = 0.35;
  far_recovery.innovation_along_m = 12.2;
  far_recovery.innovation_cross_m = -14.8;
  far_recovery.innovation_yaw_rad = 0.4 * M_PI / 180.0;

  RuntimeCandidateTierOptions options;
  options.recovery_stable_max_innovation_m = 1.0;
  options.recovery_stable_max_yaw_deg = 5.0;

  EXPECT_TRUE(runtime_candidate_is_stable_for_recovery(far_recovery, options));
}

TEST(RuntimeMultistartTiering, PeriodicallyRefreshesSmallTrackingTier)
{
  EXPECT_TRUE(should_refresh_tracking_tier1(20.0, -1.0, 1.0));
  EXPECT_TRUE(should_refresh_tracking_tier1(20.0, 18.9, 1.0));
  EXPECT_FALSE(should_refresh_tracking_tier1(20.0, 19.5, 1.0));
  EXPECT_FALSE(should_refresh_tracking_tier1(20.0, 18.0, 0.0));
}

}  // namespace
}  // namespace autoware::ndt_scan_matcher
