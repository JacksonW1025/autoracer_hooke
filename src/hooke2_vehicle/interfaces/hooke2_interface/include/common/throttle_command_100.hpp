#ifndef COMMON__THROTTLE_COMMAND_100_HPP_
#define COMMON__THROTTLE_COMMAND_100_HPP_

#pragma once

#include "common/protocol_data.hpp"
#include <can_msgs/msg/frame.hpp>

#include <hooke2_msgs/msg/throttle_cmd100.hpp>

namespace hooke2::common {

using hooke2_msgs::msg::ThrottleCmd100;

class Throttlecommand100
    : public ::hooke2::common::ProtocolData<can_msgs::msg::Frame> {
 public:

  static const int32_t ID;

  Throttlecommand100();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'Vel_Target', 'offset': 0.0, 'precision': 0.01,
  // 'len': 10, 'is_signed_var': False, 'physical_range': '[0|10.23]', 'bit':
  // 47, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s'}
  Throttlecommand100* set_vel_target(double vel_target);

  // config detail: {'name': 'Throttle_Acc', 'offset': 0.0, 'precision': 0.01,
  // 'len': 10, 'is_signed_var': False, 'physical_range': '[0|10]', 'bit': 15,
  // 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s^2'}
  Throttlecommand100* set_throttle_acc(double throttle_acc);

  // config detail: {'name': 'CheckSum_100', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Throttlecommand100* set_checksum_100(int checksum_100);

  // config detail: {'name': 'Throttle_Pedal_Target', 'offset': 0.0,
  // 'precision': 0.1, 'len': 16, 'is_signed_var': False, 'physical_range':
  // '[0|100]', 'bit': 31, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': '%'}
  Throttlecommand100* set_throttle_pedal_target(double throttle_pedal_target);

  // config detail: {'name': 'Throttle_EN_CTRL', 'uint8': {0:
  // 'THROTTLE_EN_CTRL_DISABLE', 1: 'THROTTLE_EN_CTRL_ENABLE'},
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 0, 'type': 'uint8', 'order': 'motorola',
  // 'physical_unit': ''}
  Throttlecommand100* set_throttle_en_ctrl(uint8_t throttle_en_ctrl);

 private:
  // config detail: {'name': 'Vel_Target', 'offset': 0.0, 'precision': 0.01,
  // 'len': 10, 'is_signed_var': False, 'physical_range': '[0|10.23]', 'bit':
  // 47, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s'}
  void set_p_vel_target(uint8_t* data, double vel_target);

  // config detail: {'name': 'Throttle_Acc', 'offset': 0.0, 'precision': 0.01,
  // 'len': 10, 'is_signed_var': False, 'physical_range': '[0|10]', 'bit': 15,
  // 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s^2'}
  void set_p_throttle_acc(uint8_t* data, double throttle_acc);

  // config detail: {'name': 'CheckSum_100', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_checksum_100(uint8_t* data, int checksum_100);

  // config detail: {'name': 'Throttle_Pedal_Target', 'offset': 0.0,
  // 'precision': 0.1, 'len': 16, 'is_signed_var': False, 'physical_range':
  // '[0|100]', 'bit': 31, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': '%'}
  void set_p_throttle_pedal_target(uint8_t* data, double throttle_pedal_target);

  // config detail: {'name': 'Throttle_EN_CTRL', 'uint8': {0:
  // 'THROTTLE_EN_CTRL_DISABLE', 1: 'THROTTLE_EN_CTRL_ENABLE'},
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 0, 'type': 'uint8', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_throttle_en_ctrl(
      uint8_t* data, uint8_t throttle_en_ctrl);

 private:
  double vel_target_;
  double throttle_acc_;
  int checksum_100_;
  double throttle_pedal_target_;
  uint8_t throttle_en_ctrl_;
};

}  // namespace hooke2::common

#endif  // COMMON__THROTTLE_COMMAND_100_HPP_
