#include "common/vehicle_mode_command_105.hpp"

#include <wd_byte/byte.hpp>

namespace hooke2::common {

using wd_byte::Byte;

const int32_t Vehiclemodecommand105::ID = 0x105;

// public
Vehiclemodecommand105::Vehiclemodecommand105() { Reset(); }

uint32_t Vehiclemodecommand105::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Vehiclemodecommand105::UpdateData(uint8_t* data) {
  set_p_turn_light_ctrl(data, turn_light_ctrl_);
  set_p_vin_req(data, vin_req_);
  set_p_drive_mode_ctrl(data, drive_mode_ctrl_);
  set_p_steer_mode_ctrl(data, steer_mode_ctrl_);
  // checksum_105_ =
  //     data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6];
  set_p_checksum_105(data, checksum_105_);
}

void Vehiclemodecommand105::Reset() {
  // TODO(All) :  you should check this manually
  checksum_105_ = 0;
  turn_light_ctrl_ = VehicleModeCmd105::TURN_LIGHT_CTRL_TURNLAMP_OFF;
  vin_req_ = VehicleModeCmd105::VIN_REQ_VIN_REQ_DISABLE;
  drive_mode_ctrl_ =
      VehicleModeCmd105::DRIVE_MODE_CTRL_THROTTLE_PADDLE_DRIVE;
  steer_mode_ctrl_ = VehicleModeCmd105::STEER_MODE_CTRL_STANDARD_STEER;
}

Vehiclemodecommand105* Vehiclemodecommand105::set_checksum_105(
    int checksum_105) {
  checksum_105_ = checksum_105;
  return this;
}

// config detail: {'name': 'CheckSum_105', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Vehiclemodecommand105::set_p_checksum_105(uint8_t* data,
                                               int checksum_105) {
  checksum_105 = ProtocolData::BoundedValue(0, 255, checksum_105);
  int x = checksum_105;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}

Vehiclemodecommand105* Vehiclemodecommand105::set_turn_light_ctrl(
    uint8_t turn_light_ctrl) {
  turn_light_ctrl_ = turn_light_ctrl;
  return this;
}

// config detail: {'name': 'Turn_Light_CTRL', 'uint8': {0:
// 'TURN_LIGHT_CTRL_TURNLAMP_OFF', 1: 'TURN_LIGHT_CTRL_LEFT_TURNLAMP_ON', 2:
// 'TURN_LIGHT_CTRL_RIGHT_TURNLAMP_ON', 3:
// 'TURN_LIGHT_CTRL_HAZARD_WARNING_LAMPSTS_ON'}, 'precision': 1.0, 'len': 2,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 17,
// 'type': 'uint8', 'order': 'motorola', 'physical_unit': ''}
void Vehiclemodecommand105::set_p_turn_light_ctrl(
    uint8_t* data,
    uint8_t turn_light_ctrl) {
  int x = turn_light_ctrl;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 2);
}

Vehiclemodecommand105* Vehiclemodecommand105::set_vin_req(uint8_t vin_req) {
  vin_req_ = vin_req;
  return this;
}

// config detail: {'name': 'VIN_Req', 'uint8': {0: 'VIN_REQ_VIN_REQ_DISABLE', 1:
// 'VIN_REQ_VIN_REQ_ENABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 24, 'type': 'uint8',
// 'order': 'motorola', 'physical_unit': ''}
void Vehiclemodecommand105::set_p_vin_req(uint8_t* data, uint8_t vin_req) {
  int x = vin_req;

  Byte to_set(data + 3);
  to_set.set_value(x, 0, 1);
}

Vehiclemodecommand105* Vehiclemodecommand105::set_drive_mode_ctrl(uint8_t drive_mode_ctrl) {
  drive_mode_ctrl_ = drive_mode_ctrl;
  return this;
}

// config detail: {'name': 'Drive_Mode_CTRL', 'uint8': {0:
// 'DRIVE_MODE_CTRL_THROTTLE_PADDLE_DRIVE', 1: 'DRIVE_MODE_CTRL_SPEED_DRIVE'},
// 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|7]', 'bit': 10, 'type': 'uint8', 'order': 'motorola',
// 'physical_unit': ''}
void Vehiclemodecommand105::set_p_drive_mode_ctrl(
    uint8_t* data, uint8_t drive_mode_ctrl) {
  int x = drive_mode_ctrl;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 3);
}

Vehiclemodecommand105* Vehiclemodecommand105::set_steer_mode_ctrl(uint8_t steer_mode_ctrl) {
  steer_mode_ctrl_ = steer_mode_ctrl;
  return this;
}

// config detail: {'name': 'Steer_Mode_CTRL', 'uint8': {0:
// 'STEER_MODE_CTRL_STANDARD_STEER', 1: 'STEER_MODE_CTRL_NON_DIRECTION_STEER',
// 2: 'STEER_MODE_CTRL_SYNC_DIRECTION_STEER'}, 'precision': 1.0, 'len': 3,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 2,
// 'type': 'uint8', 'order': 'motorola', 'physical_unit': ''}
void Vehiclemodecommand105::set_p_steer_mode_ctrl(uint8_t* data,
    uint8_t steer_mode_ctrl) {
  int x = steer_mode_ctrl;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 3);
}

}  // namespace hooke2::common
