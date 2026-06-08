#include "../src/direct_pose_validation.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

namespace autoware::pose_initializer
{
namespace
{

TEST(DirectPoseValidation, RejectsStaleDirectSeed)
{
  const rclcpp::Time request_time(21, 199000000, RCL_ROS_TIME);
  const rclcpp::Time stale_pose_time(19, 462000000, RCL_ROS_TIME);
  const rclcpp::Time fresh_pose_time(21, 100000000, RCL_ROS_TIME);

  EXPECT_FALSE(direct_pose_stamp_is_fresh(stale_pose_time, request_time, 0.5));
  EXPECT_TRUE(direct_pose_stamp_is_fresh(fresh_pose_time, request_time, 0.5));
}

}  // namespace
}  // namespace autoware::pose_initializer
