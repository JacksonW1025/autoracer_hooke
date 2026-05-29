#ifndef COMMON__BRAKE_COMMAND_101_HPP_
#define COMMON__BRAKE_COMMAND_101_HPP_

#pragma once

#include "common/protocol_data.hpp"
#include <can_msgs/msg/frame.hpp>

#include <hooke2_msgs/msg/brake_cmd101.hpp>

namespace hooke2::common {

using hooke2_msgs::msg::BrakeCmd101;

class Brakecommand101
    : public ::hooke2::common::ProtocolData<can_msgs::msg::Frame> {
 public:
  static const int32_t ID;

  Brakecommand101();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'AEB_EN_CTRL', 'uint8': {0:
  // 'AEB_EN_CTRL_DISABLE_AEB', 1: 'AEB_EN_CTRL_ENABLE_AEB'}, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]',
  // 'bit': 1, 'type': 'uint8', 'order': 'motorola', 'physical_unit': ''}
  Brakecommand101* set_aeb_en_ctrl(
      uint8_t aeb_en_ctrl);

  // config detail: {'name': 'Brake_Dec', 'offset': 0.0, 'precision': 0.01,
  // 'len': 10, 'is_signed_var': False, 'physical_range': '[0|10]', 'bit': 15,
  // 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s^2'}
  Brakecommand101* set_brake_dec(double brake_dec);

  // config detail: {'name': 'CheckSum_101', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Brakecommand101* set_checksum_101(int checksum_101);

  // config detail: {'name': 'Brake_Pedal_Target', 'offset': 0.0, 'precision':
  // 0.1, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit':
  // 31, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
  Brakecommand101* set_brake_pedal_target(double brake_pedal_target);

  // config detail: {'name': 'Brake_EN_CTRL', 'uint8': {0:
  // 'BRAKE_EN_CTRL_DISABLE', 1: 'BRAKE_EN_CTRL_ENABLE'}, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]',
  // 'bit': 0, 'type': 'uint8', 'order': 'motorola', 'physical_unit': ''}
  Brakecommand101* set_brake_en_ctrl(
      uint8_t brake_en_ctrl);

 private:
  // config detail: {'name': 'AEB_EN_CTRL', 'uint8': {0:
  // 'AEB_EN_CTRL_DISABLE_AEB', 1: 'AEB_EN_CTRL_ENABLE_AEB'}, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]',
  // 'bit': 1, 'type': 'uint8', 'order': 'motorola', 'physical_unit': ''}
  void set_p_aeb_en_ctrl(uint8_t* data,
                         uint8_t aeb_en_ctrl);

  // config detail: {'name': 'Brake_Dec', 'offset': 0.0, 'precision': 0.01,
  // 'len': 10, 'is_signed_var': False, 'physical_range': '[0|10]', 'bit': 15,
  // 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s^2'}
  void set_p_brake_dec(uint8_t* data, double brake_dec);

  // config detail: {'name': 'CheckSum_101', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_checksum_101(uint8_t* data, int checksum_101);

  // config detail: {'name': 'Brake_Pedal_Target', 'offset': 0.0, 'precision':
  // 0.1, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit':
  // 31, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
  void set_p_brake_pedal_target(uint8_t* data, double brake_pedal_target);

  // config detail: {'name': 'Brake_EN_CTRL', 'uint8': {0:
  // 'BRAKE_EN_CTRL_DISABLE', 1: 'BRAKE_EN_CTRL_ENABLE'}, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]',
  // 'bit': 0, 'type': 'uint8', 'order': 'motorola', 'physical_unit': ''}
  void set_p_brake_en_ctrl(uint8_t* data,
                           uint8_t brake_en_ctrl);

 private:
  uint8_t aeb_en_ctrl_;
  double brake_dec_;
  int checksum_101_;
  double brake_pedal_target_;
  uint8_t brake_en_ctrl_;
};

}  // namespace hooke2::common

#endif  // COMMON__BRAKE_COMMAND_101_HPP_
