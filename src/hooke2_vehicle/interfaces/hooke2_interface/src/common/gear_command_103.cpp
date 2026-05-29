#include "common/gear_command_103.hpp"

#include <wd_byte/byte.hpp>

namespace hooke2::common {

using wd_byte::Byte;

const int32_t Gearcommand103::ID = 0x103;

// public
Gearcommand103::Gearcommand103() { Reset(); }

uint32_t Gearcommand103::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Gearcommand103::UpdateData(uint8_t* data) {
  set_p_gear_target(data, gear_target_);
  set_p_gear_en_ctrl(data, gear_en_ctrl_);
  // checksum_103_ =
  //     data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6];
  set_p_checksum_103(data, checksum_103_);
}

void Gearcommand103::Reset() {
  // TODO(All) :  you should check this manually
  gear_target_ = GearCmd103::GEAR_TARGET_INVALID;
  gear_en_ctrl_ = GearCmd103::GEAR_EN_CTRL_DISABLE;
  checksum_103_ = 0;
}

Gearcommand103* Gearcommand103::set_gear_target(
    uint8_t gear_target) {
  gear_target_ = gear_target;
  return this;
}

// config detail: {'name': 'Gear_Target', 'uint8': {0: 'GEAR_TARGET_INVALID', 1:
// 'GEAR_TARGET_PARK', 2: 'GEAR_TARGET_REVERSE', 3: 'GEAR_TARGET_NEUTRAL', 4:
// 'GEAR_TARGET_DRIVE'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 10, 'type': 'uint8', 'order':
// 'motorola', 'physical_unit': ''}
void Gearcommand103::set_p_gear_target(
    uint8_t* data, uint8_t gear_target) {
  int x = gear_target;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 3);
}

Gearcommand103* Gearcommand103::set_gear_en_ctrl(
    uint8_t gear_en_ctrl) {
  gear_en_ctrl_ = gear_en_ctrl;
  return this;
}

// config detail: {'name': 'Gear_EN_CTRL', 'uint8': {0: 'GEAR_EN_CTRL_DISABLE',
// 1: 'GEAR_EN_CTRL_ENABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'uint8',
// 'order': 'motorola', 'physical_unit': ''}
void Gearcommand103::set_p_gear_en_ctrl(
    uint8_t* data, uint8_t gear_en_ctrl) {
  int x = gear_en_ctrl;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 1);
}

Gearcommand103* Gearcommand103::set_checksum_103(int checksum_103) {
  checksum_103_ = checksum_103;
  return this;
}

// config detail: {'name': 'CheckSum_103', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Gearcommand103::set_p_checksum_103(uint8_t* data, int checksum_103) {
  checksum_103 = ProtocolData::BoundedValue(0, 255, checksum_103);
  int x = checksum_103;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}

}  // namespace hooke2::common