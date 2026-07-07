#include <autoware/ndt_scan_matcher/validation.hpp>
#include <autoware/ndt_scan_matcher/initial_pose_offsets.hpp>
#include <autoware/ndt_scan_matcher/time_offset.hpp>

#include <gtest/gtest.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace autoware::ndt_scan_matcher
{

TEST(NDTValidation, RejectsInitialToResultDistanceAbovePositiveTolerance)
{
  EXPECT_TRUE(is_initial_to_result_distance_valid(1.5, 1.5));
  EXPECT_FALSE(is_initial_to_result_distance_valid(1.5001, 1.5));
}

TEST(NDTValidation, NonPositiveToleranceKeepsValidationDisabled)
{
  EXPECT_TRUE(is_initial_to_result_distance_valid(100.0, 0.0));
  EXPECT_TRUE(is_initial_to_result_distance_valid(100.0, -1.0));
}

TEST(NDTValidation, AlignmentOutputStampCanFollowSensorPointsStamp)
{
  const rclcpp::Time request_stamp(1, 0, RCL_ROS_TIME);
  const rclcpp::Time sensor_stamp(3, 500000000, RCL_ROS_TIME);

  EXPECT_EQ(select_alignment_output_stamp(request_stamp, sensor_stamp, false), request_stamp);
  EXPECT_EQ(select_alignment_output_stamp(request_stamp, sensor_stamp, true), sensor_stamp);
}

TEST(NDTValidation, ChecksAlignmentSensorStampFreshnessAgainstRequestStamp)
{
  const rclcpp::Time request_stamp(10, 0, RCL_ROS_TIME);

  EXPECT_TRUE(is_alignment_sensor_stamp_fresh(request_stamp, rclcpp::Time(9, 900000000, RCL_ROS_TIME), 0.15));
  EXPECT_TRUE(is_alignment_sensor_stamp_fresh(request_stamp, rclcpp::Time(10, 0, RCL_ROS_TIME), 0.15));
  EXPECT_FALSE(is_alignment_sensor_stamp_fresh(request_stamp, rclcpp::Time(9, 800000000, RCL_ROS_TIME), 0.15));
}

TEST(NDTValidation, AppliesInitialPoseOffsetInSeedBodyFrame)
{
  geometry_msgs::msg::Pose seed;
  seed.position.x = 10.0;
  seed.position.y = 20.0;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, M_PI_2);
  seed.orientation = tf2::toMsg(quaternion);

  const geometry_msgs::msg::Pose shifted =
    apply_initial_pose_offset(seed, 2.0, 1.0, -90.0);

  EXPECT_NEAR(shifted.position.x, 9.0, 1e-9);
  EXPECT_NEAR(shifted.position.y, 22.0, 1e-9);
  EXPECT_NEAR(autoware::localization_util::get_rpy(shifted).z, 0.0, 1e-9);
}

TEST(NDTValidation, CountsOnlyCompleteDeterministicOffsetTriples)
{
  const std::vector<double> along{0.0, -4.0, -3.0};
  const std::vector<double> cross{0.0, 0.0};
  const std::vector<double> yaw{0.0, 0.0, 0.0};

  EXPECT_EQ(count_complete_initial_pose_offsets(along, cross, yaw), 2U);
}

}  // namespace autoware::ndt_scan_matcher
