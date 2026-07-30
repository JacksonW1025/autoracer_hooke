#include "autoracer_rc_adapter/rc_vehicle_kinematics.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace autoracer_rc_adapter
{
namespace
{

TEST(
  RcVehicleKinematics,
  PassesDirectSpeedAndSteeringIncludingStationarySteering) {
  const VehicleCommandLimits limits;
  const auto moving = control_to_chassis_motion(1.25, -0.2, limits);
  EXPECT_DOUBLE_EQ(moving.speed_mps, 1.25);
  EXPECT_DOUBLE_EQ(moving.steering_tire_angle_rad, -0.2);
  EXPECT_FALSE(moving.speed_saturated);
  EXPECT_FALSE(moving.steering_saturated);
  EXPECT_FALSE(moving.speed_below_minimum);

  const auto stopped = control_to_chassis_motion(0.0, 0.2, limits);
  EXPECT_DOUBLE_EQ(stopped.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(stopped.steering_tire_angle_rad, 0.2);
  EXPECT_FALSE(stopped.speed_below_minimum);
}

TEST(RcVehicleKinematics, PreservesForwardReverseAndLeftRightSigns) {
  const VehicleCommandLimits limits;
  const auto forward_left = control_to_chassis_motion(0.5, 0.15, limits);
  EXPECT_DOUBLE_EQ(forward_left.speed_mps, 0.5);
  EXPECT_DOUBLE_EQ(forward_left.steering_tire_angle_rad, 0.15);

  const auto reverse_right = control_to_chassis_motion(-0.5, -0.15, limits);
  EXPECT_DOUBLE_EQ(reverse_right.speed_mps, -0.5);
  EXPECT_DOUBLE_EQ(reverse_right.steering_tire_angle_rad, -0.15);
}

TEST(RcVehicleKinematics, ClampsConfirmedProductBoundaries) {
  const VehicleCommandLimits limits;
  const auto positive = control_to_chassis_motion(3.5, 0.8, limits);
  EXPECT_DOUBLE_EQ(positive.speed_mps, 3.0);
  EXPECT_DOUBLE_EQ(positive.steering_tire_angle_rad, 0.349);
  EXPECT_TRUE(positive.speed_saturated);
  EXPECT_TRUE(positive.steering_saturated);

  const auto negative = control_to_chassis_motion(-3.5, -0.8, limits);
  EXPECT_DOUBLE_EQ(negative.speed_mps, -3.0);
  EXPECT_DOUBLE_EQ(negative.steering_tire_angle_rad, -0.349);
  EXPECT_TRUE(negative.speed_saturated);
  EXPECT_TRUE(negative.steering_saturated);

  const auto boundary = control_to_chassis_motion(3.0, -0.349, limits);
  EXPECT_FALSE(boundary.speed_saturated);
  EXPECT_FALSE(boundary.steering_saturated);
}

TEST(RcVehicleKinematics, MapsSubMinimumSpeedToZeroWithoutDiscardingSteering) {
  const VehicleCommandLimits limits;
  const auto positive = control_to_chassis_motion(0.299, 0.2, limits);
  EXPECT_DOUBLE_EQ(positive.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(positive.steering_tire_angle_rad, 0.2);
  EXPECT_TRUE(positive.speed_below_minimum);

  const auto negative = control_to_chassis_motion(-0.299, -0.2, limits);
  EXPECT_DOUBLE_EQ(negative.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(negative.steering_tire_angle_rad, -0.2);
  EXPECT_TRUE(negative.speed_below_minimum);

  EXPECT_DOUBLE_EQ(control_to_chassis_motion(0.3, 0.0, limits).speed_mps, 0.3);
  EXPECT_DOUBLE_EQ(
    control_to_chassis_motion(-0.3, 0.0, limits).speed_mps,
    -0.3);
}

TEST(RcVehicleKinematics, RejectsNonfiniteInputsAndInvalidLimits) {
  const VehicleCommandLimits defaults;
  EXPECT_THROW(
    (void)control_to_chassis_motion(
      std::numeric_limits<double>::quiet_NaN(), 0.0, defaults),
    std::invalid_argument);
  EXPECT_THROW(
    (void)control_to_chassis_motion(
      0.0, std::numeric_limits<double>::infinity(), defaults),
    std::invalid_argument);

  auto invalid = defaults;
  invalid.maximum_command_speed_mps = 0.0;
  EXPECT_THROW(
    (void)control_to_chassis_motion(0.0, 0.0, invalid),
    std::invalid_argument);
  invalid = defaults;
  invalid.minimum_command_speed_mps = -0.1;
  EXPECT_THROW(
    (void)control_to_chassis_motion(0.0, 0.0, invalid),
    std::invalid_argument);
  invalid = defaults;
  invalid.minimum_command_speed_mps = 3.1;
  EXPECT_THROW(
    (void)control_to_chassis_motion(0.0, 0.0, invalid),
    std::invalid_argument);
  invalid = defaults;
  invalid.max_steering_tire_angle_rad = -0.1;
  EXPECT_THROW(
    (void)control_to_chassis_motion(0.0, 0.0, invalid),
    std::invalid_argument);
}

} // namespace
} // namespace autoracer_rc_adapter
