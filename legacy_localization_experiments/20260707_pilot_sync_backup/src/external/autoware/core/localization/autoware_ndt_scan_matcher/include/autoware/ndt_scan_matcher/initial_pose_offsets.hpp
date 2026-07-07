#ifndef AUTOWARE__NDT_SCAN_MATCHER__INITIAL_POSE_OFFSETS_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__INITIAL_POSE_OFFSETS_HPP_

#include <autoware/localization_util/util_func.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::ndt_scan_matcher
{

inline double normalize_initial_pose_offset_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline std::size_t count_complete_initial_pose_offsets(
  const std::vector<double> & along_m, const std::vector<double> & cross_m,
  const std::vector<double> & yaw_deg)
{
  return std::min({along_m.size(), cross_m.size(), yaw_deg.size()});
}

inline geometry_msgs::msg::Pose apply_initial_pose_offset(
  const geometry_msgs::msg::Pose & seed_pose, const double along_m, const double cross_m,
  const double yaw_offset_deg)
{
  geometry_msgs::msg::Pose ret = seed_pose;
  const auto rpy = autoware::localization_util::get_rpy(seed_pose);
  const double yaw = rpy.z;
  ret.position.x += std::cos(yaw) * along_m - std::sin(yaw) * cross_m;
  ret.position.y += std::sin(yaw) * along_m + std::cos(yaw) * cross_m;

  tf2::Quaternion quaternion;
  quaternion.setRPY(
    rpy.x, rpy.y, normalize_initial_pose_offset_angle(yaw + yaw_offset_deg * M_PI / 180.0));
  ret.orientation = tf2::toMsg(quaternion);
  return ret;
}

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__INITIAL_POSE_OFFSETS_HPP_
