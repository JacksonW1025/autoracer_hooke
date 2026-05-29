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

#include <hooke2_interface/hooke2_diag_publisher.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>

Hooke2DiagPublisher::Hooke2DiagPublisher()
: Node("hooke2_diag_publisher"),
  last_can_received_time_(this->now()),
  last_wd_hooke2_msgs_received_time_(this->now())
{
  /* ros parameters */
  can_timeout_sec_ = declare_parameter("can_timeout_sec", 10.0);
  wd_hooke2_msgs_timeout_sec_ = declare_parameter("hooke2_msg_timeout_sec", 10.0);
  const double update_rate = declare_parameter("update_rate", 10.0);

  accel_store_time_ = declare_parameter("accel_store_time", 1.0);
  accel_diff_thresh_ = declare_parameter("accel_diff_thresh", 1.0);
  min_decel_ = declare_parameter("min_decel", -3.0);
  max_accel_ = declare_parameter("max_accel", 3.0);
  accel_brake_fault_check_min_velocity_ =
    declare_parameter("accel_brake_fault_check_min_velocity", 1.39);

  /* Diagnostic Updater */
  updater_ptr_ = std::make_shared<diagnostic_updater::Updater>(this, 1.0 / update_rate);
  updater_ptr_->setHardwareID("hooke2_checker");
  updater_ptr_->add("hooke2_checker", this, &Hooke2DiagPublisher::checkHooke2Msgs);
  updater_ptr_->add("hooke2_accel_brake_fault", this, &Hooke2DiagPublisher::checkHooke2AccelBrake);

  /* register subscribers */
  can_sub_ = create_subscription<can_msgs::msg::Frame>(
    "/can_driver_node/can_tx", 1,
    std::bind(&Hooke2DiagPublisher::callbackCan, this, std::placeholders::_1));

  /* hooke2-related topics */
  steer_wheel_rpt_sub_ =
    std::make_unique<message_filters::Subscriber<hooke2_msgs::msg::SystemRptFloat>>(
      this, "/hooke2/steering_rpt");
  wheel_speed_rpt_sub_ =
    std::make_unique<message_filters::Subscriber<hooke2_msgs::msg::WheelSpeedRpt>>(
      this, "/hooke2/wheel_speed_rpt");
  accel_rpt_sub_ = std::make_unique<message_filters::Subscriber<hooke2_msgs::msg::SystemRptFloat>>(
    this, "/hooke2/accel_rpt");
  brake_rpt_sub_ = std::make_unique<message_filters::Subscriber<hooke2_msgs::msg::SystemRptFloat>>(
    this, "/hooke2/brake_rpt");
  shift_rpt_sub_ = std::make_unique<message_filters::Subscriber<hooke2_msgs::msg::SystemRptInt>>(
    this, "/hooke2/shift_rpt");
  turn_rpt_sub_ = std::make_unique<message_filters::Subscriber<hooke2_msgs::msg::SystemRptInt>>(
    this, "/hooke2/turn_rpt");
  global_rpt_sub_ = std::make_unique<message_filters::Subscriber<hooke2_msgs::msg::GlobalRpt>>(
    this, "/hooke2/global_rpt");

  hooke2_feedbacks_sync_ =
    std::make_unique<message_filters::Synchronizer<Hooke2FeedbacksSyncPolicy>>(
      Hooke2FeedbacksSyncPolicy(10), *steer_wheel_rpt_sub_, *wheel_speed_rpt_sub_, *accel_rpt_sub_,
      *brake_rpt_sub_, *shift_rpt_sub_, *turn_rpt_sub_, *global_rpt_sub_);

  hooke2_feedbacks_sync_->registerCallback(std::bind(
    &Hooke2DiagPublisher::callbackHooke2Rpt, this, std::placeholders::_1, std::placeholders::_2,
    std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6,
    std::placeholders::_7));

  /* acceleration-related topics */
  current_acc_sub_ = create_subscription<AccelWithCovarianceStamped>(
    "/localization/acceleration", 1,
    std::bind(&Hooke2DiagPublisher::callbackAccel, this, std::placeholders::_1));
  control_cmd_sub_ = create_subscription<Control>(
    "/control/command/control_cmd", 1,
    std::bind(&Hooke2DiagPublisher::callbackControlCmd, this, std::placeholders::_1));
  odom_sub_ = create_subscription<Odometry>(
    "/localization/kinematic_state", 1,
    std::bind(&Hooke2DiagPublisher::callbackOdometry, this, std::placeholders::_1));
}

void Hooke2DiagPublisher::callbackCan(
  [[maybe_unused]] const can_msgs::msg::Frame::ConstSharedPtr can)
{
  last_can_received_time_ = this->now();
}

void Hooke2DiagPublisher::callbackHooke2Rpt(
  const hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr steer_wheel_rpt,
  const hooke2_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt,
  const hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt,
  const hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt,
  const hooke2_msgs::msg::SystemRptInt::ConstSharedPtr shift_rpt,
  const hooke2_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt,
  const hooke2_msgs::msg::GlobalRpt::ConstSharedPtr global_rpt)
{
  last_wd_hooke2_msgs_received_time_ = this->now();
  steer_wheel_rpt_ptr_ = steer_wheel_rpt;
  wheel_speed_rpt_ptr_ = wheel_speed_rpt;
  accel_rpt_ptr_ = accel_rpt;
  brake_rpt_ptr_ = brake_rpt;
  shift_rpt_ptr_ = shift_rpt;
  global_rpt_ptr_ = global_rpt;
  turn_rpt_ptr_ = turn_rpt;
  is_hooke2_rpt_received_ = true;
}

void Hooke2DiagPublisher::callbackAccel(const AccelWithCovarianceStamped::ConstSharedPtr accel)
{
  addValueToQue(acc_que_, accel->accel.accel.linear.x, accel->header.stamp, accel_store_time_);
}

void Hooke2DiagPublisher::callbackControlCmd(
  const Control::ConstSharedPtr control_cmd)
{
  addValueToQue(
    acc_cmd_que_, control_cmd->longitudinal.acceleration, control_cmd->stamp, accel_store_time_);
}

void Hooke2DiagPublisher::callbackOdometry(const Odometry::SharedPtr odom)
{
  odom_ptr_ = odom;
}

void Hooke2DiagPublisher::checkHooke2Msgs(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
  int8_t level = DiagStatus::OK;
  std::string msg = "";

  if (isTimeoutCanMsgs()) {
    level = DiagStatus::ERROR;
    msg = addMsg(msg, "CAN reception timeout.");
  }

  if (isTimeoutHooke2Msgs()) {
    level = DiagStatus::ERROR;
    msg = addMsg(msg, "Hooke2 msgs reception timeout.");
  }

  if (!receivedHooke2Msgs()) {
    if (level == DiagStatus::OK) {
      msg = "OK";
    }
    stat.summary(level, msg);
    // do not receive hooke2 msgs yet.
    return;
  }

  if (isBrakeActuatorAccident()) {
    level = DiagStatus::ERROR;
    msg = addMsg(msg, "Brake actuator failure.");
  }

  if (isBrakeWireAccident()) {
    level = DiagStatus::ERROR;
    msg = addMsg(msg, "Brake wire failure.");
  }

  if (isAccelAccident()) {
    level = DiagStatus::ERROR;
    msg = addMsg(msg, "Accel module failure.");
  }

  if (level == DiagStatus::OK && isOtherAccident()) {
    level = DiagStatus::ERROR;
    msg = addMsg(msg, "Unknown Hooke2 Error.");
  }

  if (level == DiagStatus::OK) {
    msg = "OK";
  }
  stat.summary(level, msg);
}

void Hooke2DiagPublisher::checkHooke2AccelBrake(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
  int8_t level = DiagStatus::OK;
  std::string msg = "OK";

  if (!(global_rpt_ptr_ && steer_wheel_rpt_ptr_ && accel_rpt_ptr_ && brake_rpt_ptr_ && odom_ptr_)) {
    // does not received messages yet.
    stat.summary(level, msg);
    return;
  }

  if (!(global_rpt_ptr_->enabled && steer_wheel_rpt_ptr_->enabled && accel_rpt_ptr_->enabled &&
        brake_rpt_ptr_->enabled)) {
    // manual mode (not autonomous mode )
    stat.summary(level, msg);
    return;
  }

  if (
    !checkEnoughDataStored(acc_que_, accel_store_time_) ||
    !checkEnoughDataStored(acc_cmd_que_, accel_store_time_)) {
    // no enough data
    stat.summary(level, msg);
    return;
  }

  if (checkBrakeFault()) {
    level = DiagStatus::ERROR;
    msg = "Hooke2 Brake Fault. Not decelerating enough.";
  } else if (checkAccelFault()) {
    level = DiagStatus::ERROR;
    msg = "Hooke2 Accel Fault. Not accelerating enough.";
  }

  stat.summary(level, msg);
}

std::string Hooke2DiagPublisher::addMsg(
  const std::string & original_msg, const std::string & additional_msg)
{
  if (original_msg == "") {
    return additional_msg;
  }

  return original_msg + " ; " + additional_msg;
}

void Hooke2DiagPublisher::addValueToQue(
  std::vector<std::pair<builtin_interfaces::msg::Time, double>> & que, const double value,
  const builtin_interfaces::msg::Time timestamp, const double store_time)
{
  if (!(global_rpt_ptr_ && steer_wheel_rpt_ptr_ && accel_rpt_ptr_ && brake_rpt_ptr_ && odom_ptr_)) {
    // if the data is not ready, clear que.
    que.clear();
    return;
  }

  if (!(global_rpt_ptr_->enabled && steer_wheel_rpt_ptr_->enabled && accel_rpt_ptr_->enabled &&
        brake_rpt_ptr_->enabled)) {
    // if the mode is manual (not autonomous), clear que.
    que.clear();
    return;
  }

  if (odom_ptr_->twist.twist.linear.x < accel_brake_fault_check_min_velocity_) {
    // When the vehicle is low speed or moving backward, clear que.
    que.clear();
    return;
  }

  que.emplace_back(std::pair<builtin_interfaces::msg::Time, double>(timestamp, value));

  if (que.size() < 2) return;

  // delete old data
  const auto current_time = get_clock()->now();
  if ((current_time - que.at(1).first).seconds() > store_time) {
    que.erase(que.begin());
  }
}

bool Hooke2DiagPublisher::checkEnoughDataStored(
  const std::vector<std::pair<builtin_interfaces::msg::Time, double>> que, const double store_time)
{
  if (que.empty()) {
    // no data
    return false;
  }

  const auto current_time = get_clock()->now();
  const auto oldest_que_time = (current_time - que.front().first).seconds();
  return (oldest_que_time > store_time);
}

double Hooke2DiagPublisher::getMinValue(
  std::vector<std::pair<builtin_interfaces::msg::Time, double>> que)
{
  auto it = std::min_element(
    que.begin(), que.end(), [](const auto & a, const auto & b) { return a.second < b.second; });

  return it->second;
}

double Hooke2DiagPublisher::getMaxValue(
  std::vector<std::pair<builtin_interfaces::msg::Time, double>> que)
{
  auto it = std::max_element(
    que.begin(), que.end(), [](const auto & a, const auto & b) { return a.second < b.second; });
  return it->second;
}

bool Hooke2DiagPublisher::checkAccelFault()
{
  const auto maximum_acc = getMaxValue(acc_que_);
  const auto minimum_acc_cmd = std::min(getMinValue(acc_cmd_que_), max_accel_);
  if (minimum_acc_cmd > 0.0 && minimum_acc_cmd - maximum_acc > accel_diff_thresh_) {
    // The vehicle acceleration is significantly lower than the acceleration command
    // Acceleration may be not working properly
    return true;
  }
  return false;
}

bool Hooke2DiagPublisher::checkBrakeFault()
{
  const auto minimum_acc = getMinValue(acc_que_);
  const auto maximum_acc_cmd = std::max(getMaxValue(acc_cmd_que_), min_decel_);
  if (maximum_acc_cmd < 0.0 && maximum_acc_cmd - minimum_acc < -accel_diff_thresh_) {
    // The vehicle deceleration is significantly lower than the deceleration command
    // Deceleration may be not working properly
    return true;
  }
  return false;
}

bool Hooke2DiagPublisher::isTimeoutCanMsgs()
{
  const double dt = (this->now() - last_can_received_time_).seconds();
  return dt > can_timeout_sec_;
}

bool Hooke2DiagPublisher::isTimeoutHooke2Msgs()
{
  const double dt = (this->now() - last_wd_hooke2_msgs_received_time_).seconds();
  return dt > wd_hooke2_msgs_timeout_sec_;
}

bool Hooke2DiagPublisher::receivedHooke2Msgs()
{
  return is_hooke2_rpt_received_;
}

bool Hooke2DiagPublisher::isBrakeActuatorAccident()
{
  return global_rpt_ptr_->hooke2_sys_fault_active && brake_rpt_ptr_->hooke2_fault;
}

bool Hooke2DiagPublisher::isBrakeWireAccident()
{
  return global_rpt_ptr_->hooke2_sys_fault_active && brake_rpt_ptr_->command_output_fault;
}

bool Hooke2DiagPublisher::isAccelAccident()
{
  return global_rpt_ptr_->hooke2_sys_fault_active && accel_rpt_ptr_->input_output_fault;
}

bool Hooke2DiagPublisher::isOtherAccident()
{
  return global_rpt_ptr_->hooke2_sys_fault_active;
}
