// Copyright 2021 Tier IV, Inc.
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

#ifndef HOOKE2_INTERFACE__HOOKE2_DIAG_PUBLISHER_HPP_
#define HOOKE2_INTERFACE__HOOKE2_DIAG_PUBLISHER_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <hooke2_msgs/msg/global_rpt.hpp>
#include <hooke2_msgs/msg/steering_cmd.hpp>
#include <hooke2_msgs/msg/system_cmd_float.hpp>
#include <hooke2_msgs/msg/system_cmd_int.hpp>
#include <hooke2_msgs/msg/system_rpt_float.hpp>
#include <hooke2_msgs/msg/system_rpt_int.hpp>
#include <hooke2_msgs/msg/wheel_speed_rpt.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using autoware_control_msgs::msg::Control;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;

class Hooke2DiagPublisher : public rclcpp::Node
{
  public:
    Hooke2DiagPublisher();

  private:
    using Hooke2FeedbacksSyncPolicy = message_filters::sync_policies::ApproximateTime<
      hooke2_msgs::msg::SystemRptFloat, hooke2_msgs::msg::WheelSpeedRpt,
      hooke2_msgs::msg::SystemRptFloat, hooke2_msgs::msg::SystemRptFloat,
      hooke2_msgs::msg::SystemRptInt, hooke2_msgs::msg::SystemRptInt, hooke2_msgs::msg::GlobalRpt>;

    /* subscribers */

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

    // From CAN
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

    // Acceleration-related Topics
    rclcpp::Subscription<AccelWithCovarianceStamped>::SharedPtr current_acc_sub_;
    rclcpp::Subscription<Control>::SharedPtr control_cmd_sub_;
    rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;

    /* ros parameters */
    double can_timeout_sec_;
    double wd_hooke2_msgs_timeout_sec_;

    /* variables */
    rclcpp::Time last_can_received_time_;
    rclcpp::Time last_wd_hooke2_msgs_received_time_;
    bool is_hooke2_rpt_received_ = false;
    hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr steer_wheel_rpt_ptr_;  // [rad]
    hooke2_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt_ptr_;   // [m/s]
    hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt_ptr_;
    hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt_ptr_;
    hooke2_msgs::msg::SystemRptInt::ConstSharedPtr shift_rpt_ptr_;
    hooke2_msgs::msg::GlobalRpt::ConstSharedPtr global_rpt_ptr_;
    hooke2_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt_ptr_;
    Odometry::ConstSharedPtr odom_ptr_;

    // Diagnostic Updater
    std::shared_ptr<diagnostic_updater::Updater> updater_ptr_;

    // Acceleration
    std::vector<std::pair<builtin_interfaces::msg::Time, double>> acc_que_;
    std::vector<std::pair<builtin_interfaces::msg::Time, double>> acc_cmd_que_;
    double accel_store_time_;
    double accel_diff_thresh_;
    double min_decel_;
    double max_accel_;
    double accel_brake_fault_check_min_velocity_;

    /* callbacks */
    void callbackHooke2Rpt(
      const hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr steer_wheel_rpt,
      const hooke2_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt,
      const hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt,
      const hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt,
      const hooke2_msgs::msg::SystemRptInt::ConstSharedPtr shift_rpt,
      const hooke2_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt,
      const hooke2_msgs::msg::GlobalRpt::ConstSharedPtr global_rpt);

    void callbackCan(const can_msgs::msg::Frame::ConstSharedPtr can);

    void callbackAccel(const AccelWithCovarianceStamped::ConstSharedPtr accel);
    void callbackOdometry(const Odometry::SharedPtr odom);
    void callbackControlCmd(const Control::ConstSharedPtr control_cmd);

    /* functions */
    void checkHooke2Msgs(diagnostic_updater::DiagnosticStatusWrapper & stat);
    void checkHooke2AccelBrake(diagnostic_updater::DiagnosticStatusWrapper & stat);
    std::string addMsg(const std::string & original_msg, const std::string & additional_msg);

    void addValueToQue(
      std::vector<std::pair<builtin_interfaces::msg::Time, double>> & que, const double value,
      const builtin_interfaces::msg::Time timestamp, const double store_time);
    bool checkEnoughDataStored(
      const std::vector<std::pair<builtin_interfaces::msg::Time, double>> que,
      const double store_time);
    double getMinValue(std::vector<std::pair<builtin_interfaces::msg::Time, double>> que);
    double getMaxValue(std::vector<std::pair<builtin_interfaces::msg::Time, double>> que);
    bool checkAccelFault();
    bool checkBrakeFault();

    bool isTimeoutCanMsgs();
    bool isTimeoutHooke2Msgs();
    bool receivedHooke2Msgs();
    bool isBrakeActuatorAccident();
    bool isBrakeWireAccident();
    bool isAccelAccident();
    bool isOtherAccident();
};

#endif  // HOOKE2_INTERFACE__HOOKE2_DIAG_PUBLISHER_HPP_
