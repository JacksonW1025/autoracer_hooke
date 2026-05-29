#ifndef COMMON__VEHICLE_MODE_COMMAND_105_HPP_
#define COMMON__VEHICLE_MODE_COMMAND_105_HPP_

#pragma once

#include "common/protocol_data.hpp"
#include <can_msgs/msg/frame.hpp>

#include <hooke2_msgs/msg/vehicle_mode_cmd105.hpp>

namespace hooke2::common {

using hooke2_msgs::msg::VehicleModeCmd105;

class Vehiclemodecommand105
    : public ::hooke2::common::ProtocolData<can_msgs::msg::Frame> {
 public:
  static const int32_t ID;

  Vehiclemodecommand105();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'CheckSum_105', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Vehiclemodecommand105* set_checksum_105(int checksum_105);

  // config detail: {'name': 'Turn_Light_CTRL', 'uint8': {0:
  // 'TURN_LIGHT_CTRL_TURNLAMP_OFF', 1: 'TURN_LIGHT_CTRL_LEFT_TURNLAMP_ON', 2:
  // 'TURN_LIGHT_CTRL_RIGHT_TURNLAMP_ON', 3:
  // 'TURN_LIGHT_CTRL_HAZARD_WARNING_LAMPSTS_ON'}, 'precision': 1.0, 'len': 2,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit':
  // 17, 'type': 'uint8', 'order': 'motorola', 'physical_unit': ''}
  Vehiclemodecommand105* set_turn_light_ctrl(uint8_t turn_light_ctrl);

  // config detail: {'name': 'VIN_Req', 'uint8': {0: 'VIN_REQ_VIN_REQ_DISABLE',
  // 1: 'VIN_REQ_VIN_REQ_ENABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 24, 'type': 'uint8',
  // 'order': 'motorola', 'physical_unit': ''}
  Vehiclemodecommand105* set_vin_req(uint8_t vin_req);

  // config detail: {'name': 'Drive_Mode_CTRL', 'uint8': {0:
  // 'DRIVE_MODE_CTRL_THROTTLE_PADDLE_DRIVE', 1: 'DRIVE_MODE_CTRL_SPEED_DRIVE'},
  // 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|7]', 'bit': 10, 'type': 'uint8', 'order': 'motorola',
  // 'physical_unit': ''}
  Vehiclemodecommand105* set_drive_mode_ctrl(uint8_t drive_mode_ctrl);

  // config detail: {'name': 'Steer_Mode_CTRL', 'uint8': {0:
  // 'STEER_MODE_CTRL_STANDARD_STEER', 1: 'STEER_MODE_CTRL_NON_DIRECTION_STEER',
  // 2: 'STEER_MODE_CTRL_SYNC_DIRECTION_STEER'}, 'precision': 1.0, 'len': 3,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 2,
  // 'type': 'uint8', 'order': 'motorola', 'physical_unit': ''}
  Vehiclemodecommand105* set_steer_mode_ctrl(uint8_t steer_mode_ctrl);

 private:
  // config detail: {'name': 'CheckSum_105', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_checksum_105(uint8_t* data, int checksum_105);

  // config detail: {'name': 'Turn_Light_CTRL', 'uint8': {0:
  // 'TURN_LIGHT_CTRL_TURNLAMP_OFF', 1: 'TURN_LIGHT_CTRL_LEFT_TURNLAMP_ON', 2:
  // 'TURN_LIGHT_CTRL_RIGHT_TURNLAMP_ON', 3:
  // 'TURN_LIGHT_CTRL_HAZARD_WARNING_LAMPSTS_ON'}, 'precision': 1.0, 'len': 2,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit':
  // 17, 'type': 'uint8', 'order': 'motorola', 'physical_unit': ''}
  void set_p_turn_light_ctrl(uint8_t* data, uint8_t turn_light_ctrl);

  // config detail: {'name': 'VIN_Req', 'uint8': {0: 'VIN_REQ_VIN_REQ_DISABLE',
  // 1: 'VIN_REQ_VIN_REQ_ENABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 24, 'type': 'uint8',
  // 'order': 'motorola', 'physical_unit': ''}
  void set_p_vin_req(uint8_t* data, uint8_t vin_req);

  // config detail: {'name': 'Drive_Mode_CTRL', 'uint8': {0:
  // 'DRIVE_MODE_CTRL_THROTTLE_PADDLE_DRIVE', 1: 'DRIVE_MODE_CTRL_SPEED_DRIVE'},
  // 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|7]', 'bit': 10, 'type': 'uint8', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_drive_mode_ctrl(uint8_t* data, uint8_t drive_mode_ctrl);

  // config detail: {'name': 'Steer_Mode_CTRL', 'uint8': {0:
  // 'STEER_MODE_CTRL_STANDARD_STEER', 1: 'STEER_MODE_CTRL_NON_DIRECTION_STEER',
  // 2: 'STEER_MODE_CTRL_SYNC_DIRECTION_STEER'}, 'precision': 1.0, 'len': 3,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 2,
  // 'type': 'uint8', 'order': 'motorola', 'physical_unit': ''}
  void set_p_steer_mode_ctrl(uint8_t* data, uint8_t steer_mode_ctrl);

 private:
  int checksum_105_;
  uint8_t turn_light_ctrl_;
  uint8_t vin_req_;
  uint8_t drive_mode_ctrl_;
  uint8_t steer_mode_ctrl_;
};

}  // namespace hooke2::common

#endif  // COMMON__VEHICLE_MODE_COMMAND_105_HPP_