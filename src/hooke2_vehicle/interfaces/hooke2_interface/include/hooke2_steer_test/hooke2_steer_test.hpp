// Copyright 2022 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HOOKE2_STEER_TEST__HOOKE2_STEER_TEST_HPP_
#define HOOKE2_STEER_TEST__HOOKE2_STEER_TEST_HPP_

#include <rclcpp/rclcpp.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/engage.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <hooke2_msgs/msg/global_rpt.hpp>
#include <hooke2_msgs/msg/steering_cmd.hpp>
#include <hooke2_msgs/msg/system_cmd_float.hpp>
#include <hooke2_msgs/msg/system_cmd_int.hpp>
#include <hooke2_msgs/msg/system_rpt_float.hpp>
#include <hooke2_msgs/msg/system_rpt_int.hpp>
#include <hooke2_msgs/msg/wheel_speed_rpt.hpp>
#include <std_msgs/msg/bool.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>

class Hooke2SteerTest : public rclcpp::Node
{
public:
  Hooke2SteerTest();

private:
  using Hooke2FeedbacksSyncPolicy = message_filters::sync_policies::ApproximateTime<
    hooke2_msgs::msg::SystemRptFloat, hooke2_msgs::msg::WheelSpeedRpt,
    hooke2_msgs::msg::SystemRptFloat, hooke2_msgs::msg::SystemRptFloat,
    hooke2_msgs::msg::SystemRptInt, hooke2_msgs::msg::SystemRptInt, hooke2_msgs::msg::GlobalRpt>;

  /* subscribers */
  // From Autoware
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::Engage>::SharedPtr engage_cmd_sub_;

  // From Hooke2
  std::unique_ptr<message_filters::Subscriber<hooke2_msgs::msg::SystemRptFloat>>
    steer_wheel_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<hooke2_msgs::msg::WheelSpeedRpt>>
    wheel_speed_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<hooke2_msgs::msg::SystemRptFloat>> accel_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<hooke2_msgs::msg::SystemRptFloat>> brake_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<hooke2_msgs::msg::SystemRptInt>> shift_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<hooke2_msgs::msg::SystemRptInt>> turn_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<hooke2_msgs::msg::GlobalRpt>> global_rpt_sub_;
  std::unique_ptr<message_filters::Synchronizer<Hooke2FeedbacksSyncPolicy>> hooke2_feedbacks_sync_;

  /* publishers */
  // To Hooke2
  rclcpp::Publisher<hooke2_msgs::msg::SystemCmdFloat>::SharedPtr accel_cmd_pub_;
  rclcpp::Publisher<hooke2_msgs::msg::SystemCmdFloat>::SharedPtr brake_cmd_pub_;
  rclcpp::Publisher<hooke2_msgs::msg::SteeringCmd>::SharedPtr steer_cmd_pub_;
  rclcpp::Publisher<hooke2_msgs::msg::SystemCmdInt>::SharedPtr shift_cmd_pub_;
  rclcpp::Publisher<hooke2_msgs::msg::SystemCmdInt>::SharedPtr turn_cmd_pub_;

  // output vehicle info
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr
    control_mode_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr vehicle_twist_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr
    steering_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr shift_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr
    turn_signal_status_pub_;

  vehicle_info_util::VehicleInfo vehicle_info_;
  rclcpp::TimerBase::SharedPtr timer_;

  /* ros param */
  std::string base_frame_id_;
  int command_timeout_ms_;  // vehicle_cmd timeout [ms]
  bool is_hooke2_rpt_received_ = false;
  bool is_hooke2_enabled_ = false;
  bool is_clear_override_needed_ = false;
  bool prev_override_ = false;
  double loop_rate_;    // [Hz]
  double tire_radius_;  // [m]
  double wheel_base_;   // [m]
  double vgr_coef_a_;   // variable gear ratio coeffs
  double vgr_coef_b_;   // variable gear ratio coeffs
  double vgr_coef_c_;   // variable gear ratio coeffs

  bool enable_steering_rate_control_;  // use steering angle speed for command [rad/s]

  hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr steer_wheel_rpt_ptr_;  // [rad]
  hooke2_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt_ptr_;   // [m/s]
  hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt_ptr_;
  hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt_ptr_;
  hooke2_msgs::msg::SystemRptInt::ConstSharedPtr shift_rpt_ptr_;
  hooke2_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt_ptr_;
  hooke2_msgs::msg::GlobalRpt::ConstSharedPtr global_rpt_ptr_;
  bool engage_cmd_ = false;

  /* callbacks */
  void callbackEngage(const autoware_auto_vehicle_msgs::msg::Engage::ConstSharedPtr msg);
  void callbackHooke2Rpt(
    const hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr steer_wheel_rpt,
    const hooke2_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt,
    const hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt,
    const hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt,
    const hooke2_msgs::msg::SystemRptInt::ConstSharedPtr shift_rpt,
    const hooke2_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt,
    const hooke2_msgs::msg::GlobalRpt::ConstSharedPtr global_rpt);

  /*  functions */
  void publishCommands();
  double calculateVehicleVelocity(
    const hooke2_msgs::msg::WheelSpeedRpt & wheel_speed_rpt,
    const hooke2_msgs::msg::SystemRptInt & shift_rpt);
  double calculateVariableGearRatio(const double vel, const double steer_wheel);
  double testSteerCommand();
  double sineWave();
  double increasedSineWave();
  double stepWithTwoValue();
  double stepWithThreeValue();
  double increasedStepWithTwoValue();
  bool checkDriveShift();
  double getAccel();
  double getBrake();

  double amplitude_;
  double delta_amplitude_;
  double hz_;
  double offset_;
  double steer_rate_;
  bool stopping_;
  double accel_value_;
  double brake_value_;

  const double brake_for_shift_trans = 0.7;

  enum TestMode {
    SineWave = 0,
    IncreasedSineWave,
    StepWithTwoValue,
    StepWithThreeValue,
    IncreasedStepWithTwoValue
  } test_mode_;
};

#endif  // HOOKE2_STEER_TEST__HOOKE2_STEER_TEST_HPP_
