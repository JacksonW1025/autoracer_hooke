#ifndef COMMON__STEERING_COMMAND_102_HPP_
#define COMMON__STEERING_COMMAND_102_HPP_

#pragma once

#include "common/protocol_data.hpp"
#include <can_msgs/msg/frame.hpp>

#include <hooke2_msgs/msg/steering_cmd102.hpp>

namespace hooke2::common {

using hooke2_msgs::msg::SteeringCmd102;

class Steeringcommand102
    : public ::hooke2::common::ProtocolData<can_msgs::msg::Frame> {
 public:

  static const int32_t ID;

  Steeringcommand102();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'Steer_EN_CTRL', 'uint8': {0:
  // 'STEER_EN_CTRL_DISABLE', 1: 'STEER_EN_CTRL_ENABLE'}, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]',
  // 'bit': 0, 'type': 'uint8', 'order': 'motorola', 'physical_unit': ''}
  Steeringcommand102* set_steer_en_ctrl(uint8_t steer_en_ctrl);

  // config detail: {'name': 'Steer_ANGLE_Target', 'offset': -500.0,
  // 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range':
  // '[-360|360]', 'bit': 31, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': 'deg'}
  Steeringcommand102* set_steer_angle_target(int steer_angle_target);

  // config detail: {'name': 'Steer_ANGLE_SPD', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|250]', 'bit': 15,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': 'deg/s'}
  Steeringcommand102* set_steer_angle_spd(int steer_angle_spd);

  // config detail: {'name': 'CheckSum_102', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Steeringcommand102* set_checksum_102(int checksum_102);

 private:
  // config detail: {'name': 'Steer_EN_CTRL', 'uint8': {0:
  // 'STEER_EN_CTRL_DISABLE', 1: 'STEER_EN_CTRL_ENABLE'}, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]',
  // 'bit': 0, 'type': 'uint8', 'order': 'motorola', 'physical_unit': ''}
  void set_p_steer_en_ctrl(
      uint8_t* data, uint8_t steer_en_ctrl);

  // config detail: {'name': 'Steer_ANGLE_Target', 'offset': -500.0,
  // 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range':
  // '[-360|360]', 'bit': 31, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': 'deg'}
  void set_p_steer_angle_target(uint8_t* data, int steer_angle_target);

  // config detail: {'name': 'Steer_ANGLE_SPD', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|250]', 'bit': 15,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': 'deg/s'}
  void set_p_steer_angle_spd(uint8_t* data, int steer_angle_spd);

  // config detail: {'name': 'CheckSum_102', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_checksum_102(uint8_t* data, int checksum_102);

 private:
  uint8_t steer_en_ctrl_;
  int steer_angle_target_;
  int steer_angle_spd_;
  int checksum_102_;
};

}  // namespace hooke2::common

#endif  // COMMON__STEERING_COMMAND_102_HPP_