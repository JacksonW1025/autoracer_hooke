#ifndef AUTOWARE__NDT_SCAN_MATCHER__TIME_OFFSET_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__TIME_OFFSET_HPP_

#include <rclcpp/rclcpp.hpp>

namespace autoware::ndt_scan_matcher
{

inline rclcpp::Time apply_output_pose_time_offset(
  const rclcpp::Time & sensor_ros_time, const double offset_sec)
{
  return sensor_ros_time + rclcpp::Duration::from_seconds(offset_sec);
}

inline rclcpp::Time select_alignment_output_stamp(
  const rclcpp::Time & request_stamp, const rclcpp::Time & sensor_points_stamp,
  const bool use_sensor_points_stamp)
{
  return use_sensor_points_stamp ? sensor_points_stamp : request_stamp;
}

inline bool is_alignment_sensor_stamp_fresh(
  const rclcpp::Time & request_stamp, const rclcpp::Time & sensor_points_stamp,
  const double tolerance_sec)
{
  return sensor_points_stamp >= request_stamp - rclcpp::Duration::from_seconds(tolerance_sec);
}

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__TIME_OFFSET_HPP_
