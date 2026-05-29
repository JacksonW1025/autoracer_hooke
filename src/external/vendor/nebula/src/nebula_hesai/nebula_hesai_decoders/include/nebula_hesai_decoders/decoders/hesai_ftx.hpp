// Copyright 2026 TIER IV, Inc.
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

#pragma once

#include "nebula_hesai_decoders/decoders/angle_corrector_calibration_based_solid_state.hpp"
#include "nebula_hesai_decoders/decoders/functional_safety.hpp"
#include "nebula_hesai_decoders/decoders/hesai_packet.hpp"
#include "nebula_hesai_decoders/decoders/hesai_sensor.hpp"

#include <nebula_core_common/util/bitfield.hpp>

#include <vector>

namespace nebula::drivers
{

namespace hesai_packet
{

#pragma pack(push, 1)

struct TailFTX
{
  uint8_t reserved1[7];
  uint8_t column_id;
  uint8_t row_id;
  uint8_t frame_id;  // counter, 0-255; incremented at each new scan
  uint8_t operational_mode;
  uint8_t return_mode;    // 0x33 = First
  uint16_t frame_period;  // 100, ms (sensor works @ 10Hz)
  SecondsSinceEpoch date_time;
  uint32_t timestamp;
  uint8_t factory_information;  // fixed
  uint32_t udp_sequence;
};

struct PacketFTX : public hesai_packet::PacketBase<1, 96, 1, 1>
{
  // Use a generic PacketBase for memory sizing, though FT series have varying columns and rows
  // defined by subclasses below
  using body_t = Body<NoAzimuthBlock<Unit5B, PacketFTX::n_channels>, PacketFTX::n_blocks>;

  Header25B header;
  body_t body;
  TailFTX tail;  // tail contains column_id and row_id

  // Cyber security padding
  uint8_t cyber_security[20];
};

#pragma pack(pop)

}  // namespace hesai_packet

class FTX140 : public HesaiSensor<hesai_packet::PacketFTX, AngleCorrectionType::CALIBRATION>
{
private:
public:
  using packet_t = hesai_packet::PacketFTX;
  using angle_corrector_t =
    AngleCorrectorCalibrationBasedSolidState<192, 256, packet_t::n_channels>;
  using correction_data_t = HesaiCalibrationConfiguration;

  static constexpr float min_range = 0.05;
  static constexpr float max_range = 300.0;  // Depending on mode
  static constexpr int32_t col_N = 256;
  static constexpr int32_t row_N = 192;
  static constexpr size_t max_scan_buffer_points = col_N * row_N * 2;  // For dual return
  static constexpr FieldOfView<int32_t, MilliDegrees> fov_mdeg{
    {20'000, 160'000}, {-52'500, 52'500}};
  static constexpr AnglePair<int32_t, MilliDegrees> peak_resolution_mdeg{
    140'000 / col_N,  // ~0.5 degree approx (Manual says 140x105)
    105'000 / row_N,
  };

  int get_packet_relative_point_time_offset(
    uint32_t block_id, uint32_t channel_id, const hesai_packet::PacketFTX & packet) override
  {
    (void)block_id;
    (void)channel_id;
    (void)packet;
    return 0;  // Solid state, all block measurements at the same time
  }

  [[nodiscard]] point_filters::BlockageState get_blockage_type(uint16_t distance) const override
  {
    (void)distance;
    return point_filters::BlockageState::UNSURE;
  }

  [[nodiscard]] point_filters::DitherTransform get_dither_transform() const override
  {
    return point_filters::g_default_dither_transform;
  }

  ReturnType get_return_type(
    hesai_packet::return_mode::ReturnMode return_mode, unsigned int return_idx,
    const std::vector<const hesai_packet::PacketFTX::body_t::block_t::unit_t *> & return_units)
    override
  {
    (void)return_units;

    switch (return_mode) {
      case hesai_packet::return_mode::SINGLE_FIRST:
        return ReturnType::FIRST;
      case hesai_packet::return_mode::SINGLE_STRONGEST:
        return ReturnType::STRONGEST;

      case hesai_packet::return_mode::DUAL_FIRST_STRONGEST:
        return return_idx == 1 ? ReturnType::FIRST : ReturnType::STRONGEST;

      default:
        return ReturnType::UNKNOWN;
    }
  }
};

class FTX180 : public HesaiSensor<hesai_packet::PacketFTX, AngleCorrectionType::CALIBRATION>
{
private:
public:
  using packet_t = hesai_packet::PacketFTX;
  using angle_corrector_t =
    AngleCorrectorCalibrationBasedSolidState<192, 224, packet_t::n_channels>;
  using correction_data_t = HesaiCalibrationConfiguration;

  static constexpr float min_range = 0.05;
  static constexpr float max_range = 300.0;
  static constexpr int32_t col_N = 224;  // 192x224 channel array
  static constexpr int32_t row_N = 192;
  static constexpr size_t max_scan_buffer_points = col_N * row_N * 2;
  static constexpr FieldOfView<int32_t, MilliDegrees> fov_mdeg{
    {0, 180'000}, {-52'500, 52'500}};  // Approx
  static constexpr AnglePair<int32_t, MilliDegrees> peak_resolution_mdeg{
    180'000 / col_N,
    105'000 / row_N,
  };

  int get_packet_relative_point_time_offset(
    uint32_t block_id, uint32_t channel_id, const hesai_packet::PacketFTX & packet) override
  {
    (void)block_id;
    (void)channel_id;
    (void)packet;
    return 0;
  }

  [[nodiscard]] point_filters::BlockageState get_blockage_type(uint16_t distance) const override
  {
    (void)distance;
    return point_filters::BlockageState::UNSURE;
  }

  [[nodiscard]] point_filters::DitherTransform get_dither_transform() const override
  {
    return point_filters::g_default_dither_transform;
  }

  ReturnType get_return_type(
    hesai_packet::return_mode::ReturnMode return_mode, unsigned int return_idx,
    const std::vector<const hesai_packet::PacketFTX::body_t::block_t::unit_t *> & return_units)
    override
  {
    (void)return_units;

    switch (return_mode) {
      case hesai_packet::return_mode::SINGLE_FIRST:
        return ReturnType::FIRST;
      case hesai_packet::return_mode::SINGLE_STRONGEST:
        return ReturnType::STRONGEST;

      case hesai_packet::return_mode::DUAL_FIRST_STRONGEST:
        return return_idx == 1 ? ReturnType::FIRST : ReturnType::STRONGEST;

      default:
        return ReturnType::UNKNOWN;
    }
  }
};

}  // namespace nebula::drivers
