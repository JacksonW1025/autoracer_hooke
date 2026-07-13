#include "common/throttle_command_100.hpp"

#include <wd_byte/byte.hpp>

namespace hooke2::common {

using wd_byte::Byte;

const int32_t Throttlecommand100::ID = 0x100;

// public
Throttlecommand100::Throttlecommand100() { Reset(); }

uint32_t Throttlecommand100::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Throttlecommand100::UpdateData(uint8_t* data) {
  set_p_throttle_en_ctrl(data, throttle_en_ctrl_);
  set_p_vel_target(data, vel_target_);
  set_p_throttle_acc(data, throttle_acc_);
  set_p_throttle_pedal_target(data, throttle_pedal_target_);
  // checksum_100_ =
  //     data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6];
  set_p_checksum_100(data, checksum_100_);
}

void Throttlecommand100::Reset() {
  // TODO(All) :  you should check this manually
  throttle_en_ctrl_ = ThrottleCmd100::THROTTLE_EN_CTRL_DISABLE;
  vel_target_ = 0.0;
  throttle_acc_ = 0.0;
  throttle_pedal_target_ = 0.0;
  checksum_100_ = 0;
}

Throttlecommand100* Throttlecommand100::set_vel_target(double vel_target) {
  vel_target_ = vel_target;
  return this;
}

// config detail: {'name': 'Vel_Target', 'offset': 0.0, 'precision': 0.01,
// 'len': 10, 'is_signed_var': False, 'physical_range': '[0|10.23]', 'bit': 47,
// 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s'}
void Throttlecommand100::set_p_vel_target(uint8_t* data, double vel_target) {
  vel_target = ProtocolData::BoundedValue(0.0, 10.23, vel_target / 4.0);
  int x = vel_target / 0.010000;
  uint8_t t = 0;

  t = x & 0x3;
  Byte to_set0(data + 6);
  to_set0.set_value(t, 6, 2);
  x >>= 2;

  t = x & 0xFF;
  Byte to_set1(data + 5);
  to_set1.set_value(t, 0, 8);
}

Throttlecommand100* Throttlecommand100::set_throttle_acc(double throttle_acc) {
  throttle_acc_ = throttle_acc;
  return this;
}

// config detail: {'name': 'Throttle_Acc', 'offset': 0.0, 'precision': 0.01,
// 'len': 10, 'is_signed_var': False, 'physical_range': '[0|10]', 'bit': 15,
// 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s^2'}
void Throttlecommand100::set_p_throttle_acc(uint8_t* data,
                                            double throttle_acc) {
  throttle_acc = ProtocolData::BoundedValue(0.0, 10.0, throttle_acc);
  int x = throttle_acc / 0.010000;
  uint8_t t = 0;

  t = x & 0x3;
  Byte to_set0(data + 2);
  to_set0.set_value(t, 6, 2);
  x >>= 2;

  t = x & 0xFF;
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);
}

Throttlecommand100* Throttlecommand100::set_checksum_100(int checksum_100) {
  checksum_100_ = checksum_100;
  return this;
}

// config detail: {'name': 'CheckSum_100', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Throttlecommand100::set_p_checksum_100(uint8_t* data, int checksum_100) {
  checksum_100 = ProtocolData::BoundedValue(0, 255, checksum_100);
  int x = checksum_100;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}

Throttlecommand100* Throttlecommand100::set_throttle_pedal_target(
    double throttle_pedal_target) {
  throttle_pedal_target_ = throttle_pedal_target;
  return this;
}

// config detail: {'name': 'Throttle_Pedal_Target', 'offset': 0.0, 'precision':
// 0.1, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit':
// 31, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
void Throttlecommand100::set_p_throttle_pedal_target(
    uint8_t* data, double throttle_pedal_target) {
  throttle_pedal_target =
      ProtocolData::BoundedValue(0.0, 100.0, throttle_pedal_target);
  int x = throttle_pedal_target / 0.100000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 4);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 3);
  to_set1.set_value(t, 0, 8);
}

Throttlecommand100* Throttlecommand100::set_throttle_en_ctrl(
    uint8_t throttle_en_ctrl) {
  throttle_en_ctrl_ = throttle_en_ctrl;
  return this;
}

// config detail: {'name': 'Throttle_EN_CTRL', 'uint8': {0:
// 'THROTTLE_EN_CTRL_DISABLE', 1: 'THROTTLE_EN_CTRL_ENABLE'}, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]',
// 'bit': 0, 'type': 'uint8', 'order': 'motorola', 'physical_unit': ''}
void Throttlecommand100::set_p_throttle_en_ctrl(
    uint8_t* data, uint8_t throttle_en_ctrl) {
  int x = static_cast<int32_t>(throttle_en_ctrl);

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 1);
}

}  // namespace hooke2::common
