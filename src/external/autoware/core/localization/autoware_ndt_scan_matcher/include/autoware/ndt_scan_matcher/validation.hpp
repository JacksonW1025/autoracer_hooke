#ifndef AUTOWARE__NDT_SCAN_MATCHER__VALIDATION_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__VALIDATION_HPP_

namespace autoware::ndt_scan_matcher
{

inline bool is_initial_to_result_distance_valid(
  const double distance_initial_to_result_m, const double tolerance_m)
{
  return tolerance_m <= 0.0 || distance_initial_to_result_m <= tolerance_m;
}

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__VALIDATION_HPP_
