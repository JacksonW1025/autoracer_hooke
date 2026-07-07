#ifndef DIRECT_POSE_VALIDATION_HPP_
#define DIRECT_POSE_VALIDATION_HPP_

#include <rclcpp/rclcpp.hpp>

#include <cmath>

namespace autoware::pose_initializer
{

inline bool direct_pose_stamp_is_fresh(
  const rclcpp::Time & pose_stamp, const rclcpp::Time & reference_stamp,
  const double max_age_sec)
{
  return std::abs((reference_stamp - pose_stamp).seconds()) <= max_age_sec;
}

}  // namespace autoware::pose_initializer

#endif  // DIRECT_POSE_VALIDATION_HPP_
