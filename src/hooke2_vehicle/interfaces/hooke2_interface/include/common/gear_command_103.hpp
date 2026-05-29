#ifndef COMMON__GEAR_COMMAND_103_HPP_
#define COMMON__GEAR_COMMAND_103_HPP_

#pragma once

#include "common/protocol_data.hpp"
#include <can_msgs/msg/frame.hpp>

#include <hooke2_msgs/msg/gear_cmd103.hpp>

namespace hooke2::common {

using hooke2_msgs::msg::GearCmd103;

class Gearcommand103
    : public ::hooke2::common::ProtocolData<can_msgs::msg::Frame> {
 public:
  static const int32_t ID;

  Gearcommand103();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'Gear_Target', 'uint8': {0: 'GEAR_TARGET_INVALID',
  // 1: 'GEAR_TARGET_PARK', 2: 'GEAR_TARGET_REVERSE', 3: 'GEAR_TARGET_NEUTRAL',
  // 4: 'GEAR_TARGET_DRIVE'}, 'precision': 1.0, 'len': 3, 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 10, 'type': 'uint8',
  // 'order': 'motorola', 'physical_unit': ''}
  Gearcommand103* set_gear_target(
      uint8_t gear_target);

  // config detail: {'name': 'Gear_EN_CTRL', 'uint8': {0: 'GEAR_EN_CTRL_DISABLE',
  // 1: 'GEAR_EN_CTRL_ENABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'uint8',
  // 'order': 'motorola', 'physical_unit': ''}
  Gearcommand103* set_gear_en_ctrl(
      uint8_t gear_en_ctrl);

  // config detail: {'name': 'CheckSum_103', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Gearcommand103* set_checksum_103(int checksum_103);

 private:
  // config detail: {'name': 'Gear_Target', 'uint8': {0: 'GEAR_TARGET_INVALID',
  // 1: 'GEAR_TARGET_PARK', 2: 'GEAR_TARGET_REVERSE', 3: 'GEAR_TARGET_NEUTRAL',
  // 4: 'GEAR_TARGET_DRIVE'}, 'precision': 1.0, 'len': 3, 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 10, 'type': 'uint8',
  // 'order': 'motorola', 'physical_unit': ''}
  void set_p_gear_target(uint8_t* data,
                         uint8_t gear_target);

  // config detail: {'name': 'Gear_EN_CTRL', 'uint8': {0: 'GEAR_EN_CTRL_DISABLE',
  // 1: 'GEAR_EN_CTRL_ENABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'uint8',
  // 'order': 'motorola', 'physical_unit': ''}
  void set_p_gear_en_ctrl(uint8_t* data,
                          uint8_t gear_en_ctrl);

  // config detail: {'name': 'CheckSum_103', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_checksum_103(uint8_t* data, int checksum_103);

 private:
  uint8_t gear_target_;
  uint8_t gear_en_ctrl_;
  int checksum_103_;
};

}  // namespace hooke2::common

#endif  // COMMON__GEAR_COMMAND_103_HPP_
