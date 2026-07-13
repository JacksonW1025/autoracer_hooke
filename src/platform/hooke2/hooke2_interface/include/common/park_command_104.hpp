#ifndef COMMON__PARK_COMMAND_104_HPP_
#define COMMON__PARK_COMMAND_104_HPP_

#pragma once

#include "common/protocol_data.hpp"
#include <can_msgs/msg/frame.hpp>

#include <hooke2_msgs/msg/park_cmd104.hpp>

namespace hooke2::common {

using hooke2_msgs::msg::ParkCmd104;

class Parkcommand104
    : public ::hooke2::common::ProtocolData<can_msgs::msg::Frame> {
 public:
  static const int32_t ID;

  Parkcommand104();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'CheckSum_104', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Parkcommand104* set_checksum_104(int checksum_104);

  // config detail: {'name': 'Park_Target', 'uint8': {0: 'PARK_TARGET_RELEASE',
  // 1: 'PARK_TARGET_PARKING_TRIGGER'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8,
  // 'type': 'uint8', 'order': 'motorola', 'physical_unit': ''}
  Parkcommand104* set_park_target(uint8_t park_target);

  // config detail: {'name': 'Park_EN_CTRL', 'uint8': {0: 'PARK_EN_CTRL_DISABLE',
  // 1: 'PARK_EN_CTRL_ENABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'uint8',
  // 'order': 'motorola', 'physical_unit': ''}
  Parkcommand104* set_park_en_ctrl(uint8_t park_en_ctrl);

 private:
  // config detail: {'name': 'CheckSum_104', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_checksum_104(uint8_t* data, int checksum_104);

  // config detail: {'name': 'Park_Target', 'uint8': {0: 'PARK_TARGET_RELEASE',
  // 1: 'PARK_TARGET_PARKING_TRIGGER'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8,
  // 'type': 'uint8', 'order': 'motorola', 'physical_unit': ''}
  void set_p_park_target(uint8_t* data, uint8_t park_target);

  // config detail: {'name': 'Park_EN_CTRL', 'uint8': {0: 'PARK_EN_CTRL_DISABLE',
  // 1: 'PARK_EN_CTRL_ENABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'uint8',
  // 'order': 'motorola', 'physical_unit': ''}
  void set_p_park_en_ctrl(uint8_t* data, uint8_t park_en_ctrl);

 private:
  int checksum_104_;
  uint8_t park_target_;
  uint8_t park_en_ctrl_;
};

}  // namespace hooke2::common

#endif  // COMMON__PARK_COMMAND_104_HPP_