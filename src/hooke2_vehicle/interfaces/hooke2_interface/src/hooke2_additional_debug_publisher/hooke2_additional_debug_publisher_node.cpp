// Copyright 2020 Tier IV, Inc.
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

#include <hooke2_additional_debug_publisher/hooke2_additional_debug_publisher_node.hpp>


namespace
{
  bool isTargetId(uint32_t id)
  {
    // bms_report:    0x512
    // brake_command: 0x101
    // brake_report:  0x501
    // gear_command:  0x103
    // gear_report:   0x503
    // park_command:  0x104
    // park_report:   0x504
    // steering_command:  0x102
    // steering_report:   0x502
    // throttle_command:  0x100
    // throttle_report:   0x500
    // ultr_sensor_1:     0x507
    // ultr_sensor_2:     0x508
    // ultr_sensor_3:     0x509
    // ultr_sensor_4:     0x510
    // ultr_sensor_5:     0x511
    // vcu_report:        0x505
    // vehicle_mode_command:0x105
    // vin_resp1:         0x514
    // vin_resp2:         0x515
    // vin_resp3:         0x516
    // wheelspeed_report: 0x506

    return id == 0x512 || id == 0x101 || id == 0x501 || id == 0x103 || id == 0x503 || id == 0x104 ||
          id == 0x504 || id == 0x102 || id == 0x502 || id == 0x100 || id == 0x500 || id == 0x507 ||
          id == 0x508 || id == 0x509 || id == 0x510 || id == 0x511 || id == 0x505 || id == 0x105 ||
          id == 0x514 || id == 0x515 || id == 0x516 || id == 0x506;
  }
}  // namespace

Hooke2AdditionalDebugPublisherNode::Hooke2AdditionalDebugPublisherNode()
: Node("hooke2_additional_debug_publisher")
{
  using std::placeholders::_1;

  debug_pub_ = create_publisher<tier4_debug_msgs::msg::Float32MultiArrayStamped>(
    "output/debug", rclcpp::QoS{1});
  
  can_sub_ = create_subscription<can_msgs::msg::Frame>(
    "/can_driver_node/can_tx", rclcpp::QoS{1},
    std::bind(&Hooke2AdditionalDebugPublisherNode::canTxCallback, this, _1));
  debug_value_.data.resize(17);
  
  calibration_active_ = this->declare_parameter("calibration_active", false);
  if (calibration_active_) {
    accel_cal_rpt_pub_ =
      create_publisher<tier4_debug_msgs::msg::Float32MultiArrayStamped>("output/accel_cal_rpt", 1);
    brake_cal_rpt_pub_ =
      create_publisher<tier4_debug_msgs::msg::Float32MultiArrayStamped>("output/brake_cal_rpt", 1);
    steer_cal_rpt_pub_ =
      create_publisher<tier4_debug_msgs::msg::Float32MultiArrayStamped>("output/steer_cal_rpt", 1);
    accel_cal_rpt_.data.resize(3);
    brake_cal_rpt_.data.resize(5);
    steer_cal_rpt_.data.resize(3);
  }
}

// CAN receiver, data parser
void Hooke2AdditionalDebugPublisherNode::canTxCallback(
  const can_msgs::msg::Frame::ConstSharedPtr msg)
{
  if (isTargetId(msg->id)) {
    // float debug1 = 0.0;
    // float debug2 = 0.0;
    // float debug3 = 0.0;
    // float debug4 = 0.0;
    // if (msg->id == 0x500) {
    //   const std::uint8_t* msg_bytes = &msg->data[0];

    //   // throttle_pedal_actual, 0 - 100 %
    //   // int16_t temp = static_cast<int16_t>(msg->data[3]) << 8 | msg->data[4];
    //   Byte t0(msg_bytes + 3);
    //   int32_t x = t0.get_byte(0, 8);

    //   Byte t1(msg_bytes + 4);
    //   int32_t t = t1.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double throttle_pedal_actual = x * 0.100000;

    //   // Throttle_flt2Type, {0: 'THROTTLE_FLT2_NO_FAULT', 1: 'THROTTLE_FLT2_DRIVE_SYSTEM_COMUNICATION_FAULT'}
    //   Byte t2(msg_bytes + 2);
    //   x = t2.get_byte(0, 8);

    //   int16_t flt2_type = static_cast<int16_t>(x);

    //   // Throttle_flt1Type, {0: 'THROTTLE_FLT1_NO_FAULT', 1: 'THROTTLE_FLT1_DRIVE_SYSTEM_HARDWARE_FAULT'}
    //   Byte t3(msg_bytes + 1);
    //   x = t3.get_byte(0, 8);

    //   int16_t flt1_type = static_cast<int16_t>(x);

    //   // Throttle_en_stateType, {0: 'THROTTLE_EN_STATE_MANUAL', 1: 'THROTTLE_EN_STATE_AUTO', 2: 'THROTTLE_EN_STATE_TAKEOVER', 3: 'THROTTLE_EN_STATE_STANDBY'}
    //   Byte t4(msg_bytes + 0);
    //   x = t4.get_byte(0, 2);
      
    //   int16_t Throttle_en_stateType = static_cast<int16_t>(x);

    //   // int16_t temp = 0;
    //   // temp = (static_cast<int16_t>(msg->data[0]) << 8) | msg->data[1];
    //   // accel_cal_rpt_.data.at(0) = static_cast<double>(temp / 1000.0);  // accel_a_volt
    //   // temp = (static_cast<int16_t>(msg->data[2]) << 8) | msg->data[3];
    //   // accel_cal_rpt_.data.at(1) = static_cast<double>(temp / 1000.0);  // accel_b_volt
    //   // temp = (static_cast<int16_t>(msg->data[4]) << 8) | msg->data[5];
    //   // accel_cal_rpt_.data.at(2) = static_cast<double>(temp / 1000.0);  // output
    // } else if (msg->id == 0x501) {
    //   const std::uint8_t* msg_bytes = &msg->data[0];
    //   // brake_pedal_actual, 0 - 100%
    //   Byte t0(msg_bytes + 3);
    //   int32_t x = t0.get_byte(0, 8);

    //   Byte t1(msg_bytes + 4);
    //   int32_t t = t1.get_byte(0, 8);

    //   x <<= 8;
    //   x |= t;

    //   double brake_pedal_actual = x * 0.100000;

    //   // brake_flt2type, {0: 'BRAKE_FLT2_NO_FAULT', 1: 'BRAKE_FLT2_BRAKE_SYSTEM_COMUNICATION_FAULT'}
    //   Byte t2(msg_bytes + 2);
    //   x = t2.get_byte(0, 8);
    //   uint8_t brake_flt2_type = static_cast<uint8_t>(x);

    //   // brake_flt1type, {0: 'BRAKE_FLT1_NO_FAULT', 1: 'BRAKE_FLT1_BRAKE_SYSTEM_HARDWARE_FAULT'}
    //   Byte t3(msg_bytes + 1);
    //   x = t3.get_byte(0, 8);
    //   uint8_t brake_flt1_type = static_cast<uint8_t>(x);

    //   // brake_en_statetype, {0: 'BRAKE_EN_STATE_MANUAL', 1: 'BRAKE_EN_STATE_AUTO', 2: 'BRAKE_EN_STATE_TAKEOVER', 3: 'BRAKE_EN_STATE_STANDBY'}
    //   Byte t4(msg_bytes + 0);
    //   x = t4.get_byte(0, 2);
    //   uint8_t brake_en_state_type = static_cast<uint8_t>(x);

    //   // int16_t temp = 0;
    //   // int8_t temp1 = 0;
    //   // temp = (static_cast<int16_t>(msg->data[0]) << 8) | msg->data[1];
    //   // brake_cal_rpt_.data.at(0) = static_cast<double>(temp / 1000.0);  // sks1_volt
    //   // temp = (static_cast<int16_t>(msg->data[2]) << 8) | msg->data[3];
    //   // brake_cal_rpt_.data.at(1) = static_cast<double>(temp / 1000.0);  // sks2_volt
    //   // temp1 = static_cast<int8_t>(msg->data[4]);
    //   // brake_cal_rpt_.data.at(2) = static_cast<double>(temp1 / 100.0);  // pedal_position
    //   // temp1 = static_cast<int8_t>(msg->data[5]);
    //   // brake_cal_rpt_.data.at(3) = static_cast<double>(temp1 / 100.0);  // brake_cmd
    //   // temp = (static_cast<int16_t>(msg->data[6]) << 8) | msg->data[7];
    //   // brake_cal_rpt_.data.at(4) = static_cast<double>(temp / 1000.0);  // globe_position
    // } else if (msg->id == 0x502) {
    //   const std::uint8_t* msg_bytes = &msg->data[0];
    //   // steer_angle_spd_actual, 0-250 deg/s
    //   Byte t0(msg_bytes + 7);
    //   int32_t x = t0.get_byte(0, 8);
    //   int steer_angle_spd_actual = x;

    //   // Steer_flt2Type, {0: 'STEER_FLT2_NO_FAULT', 1: 'STEER_FLT2_STEER_SYSTEM_COMUNICATION_FAULT'}
    //   Byte t1(msg_bytes + 2);
    //   x = t1.get_byte(0, 8);
    //   uint8_t steer_flt2_type = static_cast<uint8_t>(x);

    //   // Steer_flt1Type, {0: 'STEER_FLT1_NO_FAULT', 1: 'STEER_FLT1_STEER_SYSTEM_HARDWARE_FAULT'}
    //   Byte t2(msg_bytes + 1);
    //   x = t2.get_byte(0, 8);
    //   uint8_t steer_flt1_type = static_cast<uint8_t>(x);

    //   // Steer_en_stateType, {0: 'STEER_EN_STATE_MANUAL', 1: 'STEER_EN_STATE_AUTO', 2: 'STEER_EN_STATE_TAKEOVER', 3: 'STEER_EN_STATE_STANDBY'}
    //   Byte t3(msg_bytes + 0);
    //   x = t3.get_byte(0, 2);
    //   uint8_t steer_en_state_type = static_cast<uint8_t>(x);

    //   // steer_angle_actual, -500 - 500 degree
    //   Byte t4(msg_bytes + 3);
    //   x = t4.get_byte(0, 8);
    //   Byte t5(msg_bytes + 4);
    //   int32_t t = t5.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;
    //   int steer_angle_actual = x + -500.000000;
      
    //   //  收录底盘方向盘状态数据
    //   steer_wheel_rpt_pub_.data.at(0) = static_cast<double>(steer_angle_spd_actual);
    //   steer_wheel_rpt_pub_.data.at(1) = static_cast<double>(steer_flt2_type);
    //   steer_wheel_rpt_pub_.data.at(2) = static_cast<double>(steer_flt1_type);
    //   steer_wheel_rpt_pub_.data.at(3) = static_cast<double>(steer_en_state_type);
    //   steer_wheel_rpt_pub_.data.at(4) = static_cast<double>(steer_angle_actual);

    // } else if (msg->id == 0x503) {
    //   const std::uint8_t* msg_bytes = &msg->data[0];
    //   // Gear_fltType, {0: 'GEAR_FLT_NO_FAULT', 1: 'GEAR_FLT_FAULT'}
    //   Byte t0(msg_bytes + 1);
    //   int32_t x = t0.get_byte(0, 8);

    //   uint8_t gear_flt_type = static_cast<uint8_t>(x);
      
    //   // Gear_actualType, {0: 'GEAR_ACTUAL_INVALID', 1: 'GEAR_ACTUAL_PARK', 2: 'GEAR_ACTUAL_REVERSE', 3: 'GEAR_ACTUAL_NEUTRAL', 4: 'GEAR_ACTUAL_DRIVE'}
    //   Byte t1(msg_bytes + 0);
    //   x = t1.get_byte(0, 3);

    //   uint8_t gear_actual_type = static_cast<uint8_t>(x);

    // } else if (msg->id == 0x504) {
    //   const std::uint8_t* msg_bytes = &msg->data[0];
    //   // Parking_actualType, {0: 'PARKING_ACTUAL_RELEASE', 1: 'PARKING_ACTUAL_PARKING_TRIGGER'}
    //   Byte t0(msg_bytes + 0);
    //   int32_t x = t0.get_byte(0, 1);

    //   uint8_t parking_actual_type = static_cast<uint8_t>(x);

    //   // Park_fltType, {0: 'PARK_FLT_NO_FAULT', 1: 'PARK_FLT_FAULT'}
    //   Byte t1(msg_bytes + 1);
    //   x = t1.get_byte(0, 8);

    //   uint8_t parking_flt_type = static_cast<uint8_t>(x);

    // } else if (msg->id == 0x505) {
    //   const std::uint8_t* msg_bytes = &msg->data[0];
    //   // brake_light_actual, {0: 'BRAKE_LIGHT_ACTUAL_BRAKELIGHT_OFF', 1: 'BRAKE_LIGHT_ACTUAL_BRAKELIGHT_ON'}
    //   Byte t0(msg_bytes + 1);
    //   int32_t x = t0.get_byte(3, 1);

    //   uint8_t brake_light_actual_type = static_cast<uint8_t>(x);

    //   // turn_light_actual, {0: 'TURN_LIGHT_ACTUAL_TURNLAMPSTS_OFF', 1: 'TURN_LIGHT_ACTUAL_LEFT_TURNLAMPSTS_ON', 2: 'TURN_LIGHT_ACTUAL_RIGHT_TURNLAMPSTS_ON', 3: 'TURN_LIGHT_ACTUAL_HAZARD_WARNING_LAMPSTS_ON'}
    //   Byte t1(msg_bytes + 7);
    //   x = t1.get_byte(0, 2);

    //   uint8_t turn_light_actual = static_cast<uint8_t>(x);

    //   // chassis_errcode
    //   Byte t2(msg_bytes + 5);
    //   x = t2.get_byte(0, 8);

    //   int chassis_errcode = x;

    //   // Drive_mode_stsType, {0: 'DRIVE_MODE_STS_THROTTLE_PADDLE_DRIVE_MODE', 1: 'DRIVE_MODE_STS_SPEED_DRIVE_MODE'}
    //   Byte t3(msg_bytes + 4);
    //   x = t3.get_byte(5, 3);

    //   uint8_t drive_mode_sts = static_cast<uint8_t>(x);

    //   // steer_mode_sts, {0: 'STEER_MODE_STS_STANDARD_STEER_MODE', 1: 'STEER_MODE_STS_NON_DIRECTION_STEER_MODE', 2: 'STEER_MODE_STS_SYNC_DIRECTION_STEER_MODE'}
    //   Byte t4(msg_bytes + 1);
    //   x = t4.get_byte(0, 3);

    //   uint8_t steer_mode_sts = static_cast<uint8_t>(x);

    //   // vehicle_mode_state, {0: 'VEHICLE_MODE_STATE_MANUAL_REMOTE_MODE', 1: 'VEHICLE_MODE_STATE_AUTO_MODE', 2: 'VEHICLE_MODE_STATE_EMERGENCY_MODE', 3: 'VEHICLE_MODE_STATE_STANDBY_MODE'}
    //   Byte t5(msg_bytes + 4);
    //   x = t5.get_byte(3, 2);

    //   uint8_t vehicle_mode_state = static_cast<uint8_t>(x);

    //   // frontcrash_state, {0: 'FRONTCRASH_STATE_NO_EVENT', 1: 'FRONTCRASH_STATE_CRASH_EVENT'}
    //   Byte t6(msg_bytes + 4);
    //   x = t6.get_byte(1, 1);

    //   uint8_t frontcrash_state = static_cast<uint8_t>(x);

    //   // backcrash_state, {0: 'BACKCRASH_STATE_NO_EVENT', 1: 'BACKCRASH_STATE_CRASH_EVENT'}
    //   Byte t7(msg_bytes + 4);
    //   x = t7.get_byte(2, 1);

    //   uint8_t backcrash_state = static_cast<uint8_t>(x);

    //   // aeb_state, {0: 'AEB_STATE_INACTIVE', 1: 'AEB_STATE_ACTIVE'}
    //   Byte t8(msg_bytes + 4);
    //   x = t8.get_byte(0, 1);

    //   uint8_t aeb_state = static_cast<uint8_t>(x);

    //   // acc,  -10 - 10 m/s^2
    //   Byte t9(msg_bytes + 0);
    //   x = t9.get_byte(0, 8);

    //   Byte t10(msg_bytes + 1);
    //   int32_t t = t10.get_byte(4, 4);

    //   x <<= 4;
    //   x |= t;

    //   x <<= 20;
    //   x >>= 20;
    //   double vehicle_acc = x * 0.010000;

    //   // speed，-32.768 - 32.767 m/s
    //   Byte t11(msg_bytes + 2);
    //   x = t11.get_byte(0, 8);

    //   Byte t12(msg_bytes + 3);
    //   t = t12.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   x <<= 16;
    //   x >>= 16;

    //   double vehicle_speed = x * 0.001000;

    //   //  收录底盘加速度状态数据
    //   accel_rpt_pub_.data.at(0) = vehicle_acc;
    //   //  收录底盘车轮速度状态数据
    //   wheel_speed_rpt_pub_.data.at(0) = vehicle_speed;

    // } else if (msg->id == 0x506) {
    //   const std::uint8_t* msg_bytes = &msg->data[0];
    //   // rear_right_wheel_speed，0 - 65.535 m/s
    //   Byte t0(msg_bytes + 6);
    //   int32_t x = t0.get_byte(0, 8);

    //   Byte t1(msg_bytes + 7);
    //   int32_t t = t1.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double rear_right_wheel_speed = x * 0.001000;

    //   // rear_left_wheel_speed，0 - 65.535 m/s
    //   Byte t2(msg_bytes + 4);
    //   x = t2.get_byte(0, 8);

    //   Byte t3(msg_bytes + 5);
    //   t = t3.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double rear_left_wheel_speed = x * 0.001000;

    //   // front_right_wheel_speed，0 - 65.535 m/s
    //   Byte t4(msg_bytes + 2);
    //   x = t4.get_byte(0, 8);

    //   Byte t5(msg_bytes + 3);
    //   t = t5.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double front_right_wheel_speed = x * 0.001000;

    //   // front_left_wheel_speed，0 - 65.535 m/s
    //   Byte t6(msg_bytes + 0);
    //   x = t6.get_byte(0, 8);

    //   Byte t7(msg_bytes + 1);
    //   t = t7.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double front_left_wheel_speed = x * 0.001000;

    //   //  收录底盘车轮速度状态数据
    //   wheel_speed_rpt_pub_.rear_right_wheel_speed() = rear_right_wheel_speed;
    //   wheel_speed_rpt_pub_.rear_left_wheel_speed() = rear_left_wheel_speed;
    //   wheel_speed_rpt_pub_.front_right_wheel_speed() = front_right_wheel_speed;
    //   wheel_speed_rpt_pub_.front_left_wheel_speed() = front_left_wheel_speed;

    // } else if (msg->id == 0x507) {
    //   const std::uint8_t* msg_bytes = &msg->data[0];
    //   // uiuss9_tof_direct, 0-65535 cm (ultrasonic distance)
    //   Byte t0(msg_bytes + 2);
    //   int32_t x = t0.get_byte(0, 8);

    //   Byte t1(msg_bytes + 3);
    //   int32_t t = t1.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double uiuss9_tof_direct = x * 0.017240;

    //   // uiuss8_tof_direct, 0-65535 cm (ultrasonic distance)
    //   Byte t2(msg_bytes + 0);
    //   x = t2.get_byte(0, 8);

    //   Byte t3(msg_bytes + 1);
    //   t = t3.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double uiuss8_tof_direct = x * 0.017240;

    //   //  uiuss11_tof_direct, 0-65535 cm (ultrasonic distance)
    //   Byte t4(msg_bytes + 6);
    //   x = t4.get_byte(0, 8);

    //   Byte t5(msg_bytes + 7);
    //   t = t5.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double uiuss11_tof_direct = x * 0.017240;

    //   // uiuss10_tof_direct, 0-65535 cm (ultrasonic distance)
    //   Byte t6(msg_bytes + 4);
    //   x = t6.get_byte(0, 8);

    //   Byte t7(msg_bytes + 5);
    //   t = t7.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double uiuss10_tof_direct = x * 0.017240;

    // } else if (msg->id == 0x508) {
    //   const std::uint8_t* msg_bytes = &msg->data[0];
    //   // uiuss9_tof_indirect, 0-65535 cm (ultrasonic distance)
    //   Byte t0(msg_bytes + 2);
    //   int32_t x = t0.get_byte(0, 8);

    //   Byte t1(msg_bytes + 3);
    //   int32_t t = t1.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double uiuss9_tof_indirect = x * 0.017240;

    //   // uiuss8_tof_indirect, 0-65535 cm (ultrasonic distance)
    //   Byte t2(msg_bytes + 0);
    //   x = t2.get_byte(0, 8);

    //   Byte t3(msg_bytes + 1);
    //   t = t3.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double uiuss8_tof_indirect = x * 0.017240;

    //   //  uiuss11_tof_indirect, 0-65535 cm (ultrasonic distance)
    //   Byte t4(msg_bytes + 6);
    //   x = t4.get_byte(0, 8);

    //   Byte t5(msg_bytes + 7);
    //   t = t5.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double uiuss11_tof_indirect = x * 0.017240;

    //   // uiuss10_tof_indirect, 0-65535 cm (ultrasonic distance)
    //   Byte t6(msg_bytes + 4);
    //   x = t6.get_byte(0, 8);

    //   Byte t7(msg_bytes + 5);
    //   t = t7.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double uiuss10_tof_indirect = x * 0.017240;

    // } else if (msg->id == 0x509) {
    //   const std::uint8_t* msg_bytes = &msg->data[0];
    //   // uiuss5_tof_direct, 0-65535 cm (ultrasonic distance)
    //   Byte t0(msg_bytes + 6);
    //   int32_t x = t0.get_byte(0, 8);

    //   Byte t1(msg_bytes + 7);
    //   int32_t t = t1.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double uiuss5_tof_direct = x * 0.017240;

    //   // uiuss4_tof_direct, 0-65535 cm (ultrasonic distance)
    //   Byte t2(msg_bytes + 4);
    //   x = t2.get_byte(0, 8);

    //   Byte t3(msg_bytes + 5);
    //   t = t3.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double uiuss4_tof_direct = x * 0.017240;

    //   // uiuss3_tof_direct, 0-65535 cm (ultrasonic distance)
    //   Byte t4(msg_bytes + 2);
    //   x = t4.get_byte(0, 8);

    //   Byte t5(msg_bytes + 3);
    //   t = t5.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double uiuss3_tof_direct = x * 0.017240;

    //   // uiuss2_tof_direct, 0-65535 cm (ultrasonic distance)
    //   Byte t6(msg_bytes + 0);
    //   x = t6.get_byte(0, 8);

    //   Byte t7(msg_bytes + 1);
    //   t = t7.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double uiuss2_tof_direct = x * 0.017240;

    // } else if (msg->id == 0x510) {
    //   const std::uint8_t* msg_bytes = &msg->data[0];
    //   // uiuss5_tof_indirect, 0-65535 cm (ultrasonic distance)
    //   Byte t0(msg_bytes + 6);
    //   int32_t x = t0.get_byte(0, 8);

    //   Byte t1(msg_bytes + 7);
    //   int32_t t = t1.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double uiuss5_tof_indirect = x * 0.017240;

    //   // uiuss4_tof_indirect, 0-65535 cm (ultrasonic distance)
    //   Byte t2(msg_bytes + 4);
    //   x = t2.get_byte(0, 8);

    //   Byte t3(msg_bytes + 5);
    //   t = t3.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double uiuss4_tof_indirect = x * 0.017240;

    //   // uiuss3_tof_indirect, 0-65535 cm (ultrasonic distance)
    //   Byte t4(msg_bytes + 2);
    //   x = t4.get_byte(0, 8);

    //   Byte t5(msg_bytes + 3);
    //   t = t5.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double uiuss3_tof_indirect = x * 0.017240;

    //   // uiuss2_tof_indirect, 0-65535 cm (ultrasonic distance)
    //   Byte t6(msg_bytes + 0);
    //   x = t6.get_byte(0, 8);

    //   Byte t7(msg_bytes + 1);
    //   t = t7.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double uiuss2_tof_indirect = x * 0.017240;

    // } else if (msg->id == 0x511) {
    //   const std::uint8_t* msg_bytes = &msg->data[0];
    //   // uiuss7_tof_direct, 0-65535 cm (ultrasonic distance)
    //   Byte t0(msg_bytes + 6);
    //   int32_t x = t0.get_byte(0, 8);

    //   Byte t1(msg_bytes + 7);
    //   int32_t t = t1.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double uiuss7_tof_direct = x * 0.017240;

    //   // uiuss6_tof_direct, 0-65535 cm (ultrasonic distance)
    //   Byte t2(msg_bytes + 4);
    //   x = t2.get_byte(0, 8);

    //   Byte t3(msg_bytes + 5);
    //   t = t3.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double uiuss6_tof_direct = x * 0.017240;

    //   // uiuss1_tof_direct, 0-65535 cm (ultrasonic distance)
    //   Byte t4(msg_bytes + 2);
    //   x = t4.get_byte(0, 8);

    //   Byte t5(msg_bytes + 3);
    //   t = t5.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double uiuss1_tof_direct = x * 0.017240;

    //   // uiuss0_tof_direct, 0-65535 cm (ultrasonic distance)
    //   Byte t6(msg_bytes + 0);
    //   x = t6.get_byte(0, 8);

    //   Byte t7(msg_bytes + 1);
    //   t = t7.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double uiuss0_tof_direct = x * 0.017240;

    // } else if (msg->id == 0x512) {
    //   const std::uint8_t* msg_bytes = &msg->data[0];
    //   // battery_current， -3200 - 3353.5 A
    //   Byte t0(msg_bytes + 2);
    //   int32_t x = t0.get_byte(0, 8);

    //   Byte t1(msg_bytes + 3);
    //   int32_t t = t1.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double battery_current = x * 0.100000 + -3200.000000;

    //   // battery_voltage，0 - 300 V
    //   Byte t2(msg_bytes + 0);
    //   x = t2.get_byte(0, 8);

    //   Byte t3(msg_bytes + 1);
    //   t = t3.get_byte(0, 8);
    //   x <<= 8;
    //   x |= t;

    //   double battery_voltage = x * 0.010000;

    //   // battery_soc， 0 - 100%
    //   Byte t4(msg_bytes + 4);
    //   x = t4.get_byte(0, 8);

    //   int battery_soc = x;

    // } else if (msg->id == 0x514) {
    //   const std::uint8_t* msg_bytes = &msg->data[0];
    //   // vin07， 0 - 255
    //   Byte t0(msg_bytes + 7);
    //   int32_t x = t0.get_byte(0, 8);

    //   int vin07 = x;
    //   // vin06， 0 - 255
    //   Byte t1(msg_bytes + 6);
    //   x = t1.get_byte(0, 8);

    //   int vin06 = x;
    //   // vin05， 0 - 255
    //   Byte t2(msg_bytes + 5);
    //   x = t2.get_byte(0, 8);

    //   int vin05 = x;
    //   // vin04， 0 - 255
    //   Byte t3(msg_bytes + 4);
    //   x = t3.get_byte(0, 8);

    //   int vin04 = x;
    //   // vin03， 0 - 255
    //   Byte t4(msg_bytes + 3);
    //   x = t4.get_byte(0, 8);

    //   int vin03 = x;
    //   // vin02， 0 - 255
    //   Byte t5(msg_bytes + 2);
    //   x = t5.get_byte(0, 8);

    //   int vin02 = x;
    //   // vin01， 0 - 255
    //   Byte t6(msg_bytes + 1);
    //   x = t6.get_byte(0, 8);

    //   int vin01 = x;
    //   // vin00， 0 - 255
    //   Byte t7(msg_bytes + 0);
    //   x = t7.get_byte(0, 8);

    //   int vin00 = x;

    // } else if (msg->id == 0x515) {
    //   const std::uint8_t* msg_bytes = &msg->data[0];
    //   // vin15， 0 - 255
    //   Byte t0(msg_bytes + 7);
    //   int32_t x = t0.get_byte(0, 8);

    //   int vin15 = x;
    //   // vin14， 0 - 255
    //   Byte t1(msg_bytes + 6);
    //   x = t1.get_byte(0, 8);

    //   int vin14 = x;
    //   // vin13， 0 - 255
    //   Byte t2(msg_bytes + 5);
    //   x = t2.get_byte(0, 8);

    //   int vin13 = x;
    //   // vin12， 0 - 255
    //   Byte t3(msg_bytes + 4);
    //   x = t3.get_byte(0, 8);

    //   int vin12 = x;
    //   // vin11， 0 - 255
    //   Byte t4(msg_bytes + 3);
    //   x = t4.get_byte(0, 8);

    //   int vin11 = x;
    //   // vin10， 0 - 255
    //   Byte t5(msg_bytes + 2);
    //   x = t5.get_byte(0, 8);

    //   int vin10 = x;
    //   // vin09， 0 - 255
    //   Byte t6(msg_bytes + 1);
    //   x = t6.get_byte(0, 8);

    //   int vin09 = x;
    //   // vin08， 0 - 255
    //   Byte t7(msg_bytes + 0);
    //   x = t7.get_byte(0, 8);

    //   int vin08 = x;

    // } else if (msg->id == 0x516) {
    //   const std::uint8_t* msg_bytes = &msg->data[0];
    //   // vin16， 0 - 255
    //   Byte t0(msg_bytes + 0);
    //   int32_t x = t0.get_byte(0, 8);

    //   int vin16 = x;

    // } else 
    // {
      // union Data {
      //   uint32_t uint32_value;
      //   float float_value;
      // } temp;
      // ????目的不清，需要调查
      // temp.uint32_value = (static_cast<int32_t>(msg->data[3]) << 24) |
      //                     (static_cast<int32_t>(msg->data[2]) << 16) |
      //                     (static_cast<int32_t>(msg->data[1]) << 8) | msg->data[0];
      // debug1 = temp.float_value;
      // temp.uint32_value = (static_cast<int32_t>(msg->data[7]) << 24) |
      //                     (static_cast<int32_t>(msg->data[6]) << 16) |
      //                     (static_cast<int32_t>(msg->data[5]) << 8) | msg->data[4];
      // debug2 = temp.float_value;
    // }

    // switch (msg->id) {
    //   case 0x32C:
    //     debug_value_.data.at(0) = debug1;  // steering pos
    //     debug_value_.data.at(1) = debug2;  // steering_eps_assist
    //     debug_value_.data.at(2) = debug3;  // steering rate
    //     debug_value_.data.at(3) = debug4;  // steering eps input
    //     break;
    //   case 0x451:
    //     debug_value_.data.at(4) = debug1;  // pid command
    //     debug_value_.data.at(5) = debug2;  // pid output
    //     break;
    //   case 0x452:
    //     debug_value_.data.at(6) = debug1;  // pid_error
    //     debug_value_.data.at(7) = debug2;  // pid_output
    //     break;
    //   case 0x453:
    //     debug_value_.data.at(8) = debug1;  // pid_p_term
    //     debug_value_.data.at(9) = debug2;  // pid_i_term
    //     break;
    //   case 0x454:
    //     debug_value_.data.at(10) = debug1;  // pid_d_term
    //     debug_value_.data.at(11) = debug2;  // pid_filtered_rate
    //     break;
    //   case 0x455:
    //     debug_value_.data.at(12) = debug1;  // lugre
    //     debug_value_.data.at(13) = debug2;  // rtz
    //     break;
    //   case 0x456:
    //     debug_value_.data.at(14) = debug1;  // lugre_rtz_filtered_rate
    //     debug_value_.data.at(15) = debug2;  // ctrl_dt
    //     break;
    //   case 0x457:
    //     debug_value_.data.at(16) = debug1;  // rpt_dt
    //     break;
    //   default:
    //     break;
    // }

    debug_value_.stamp = this->now();
    debug_pub_->publish(debug_value_);
    if (calibration_active_) {
      accel_cal_rpt_.stamp = this->now();
      brake_cal_rpt_.stamp = this->now();
      steer_cal_rpt_.stamp = this->now();
      accel_cal_rpt_pub_->publish(accel_cal_rpt_);
      brake_cal_rpt_pub_->publish(brake_cal_rpt_);
      steer_cal_rpt_pub_->publish(steer_cal_rpt_);
    }

    // //  发布车辆底盘反馈状态
    // steer_wheel_rpt_pub_->publish(steer_wheel_rpt_pub_);

  }
}
