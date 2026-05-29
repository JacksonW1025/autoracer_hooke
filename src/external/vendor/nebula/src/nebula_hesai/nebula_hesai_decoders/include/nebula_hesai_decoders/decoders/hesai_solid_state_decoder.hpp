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

#include "nebula_core_decoders/point_filters/blockage_mask.hpp"
#include "nebula_core_decoders/point_filters/downsample_mask.hpp"
#include "nebula_hesai_decoders/decoders/angle_corrector.hpp"
#include "nebula_hesai_decoders/decoders/functional_safety.hpp"
#include "nebula_hesai_decoders/decoders/hesai_packet.hpp"
#include "nebula_hesai_decoders/decoders/hesai_scan_decoder.hpp"
#include "nebula_hesai_decoders/decoders/packet_loss_detector.hpp"

#include <nebula_core_common/loggers/logger.hpp>
#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/point_types.hpp>
#include <nebula_core_common/util/stopwatch.hpp>
#include <nebula_hesai_common/hesai_common.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace nebula::drivers
{

template <typename SensorT>
class HesaiSolidStateDecoder : public HesaiScanDecoder  // for Solid State sensor
{
private:
  struct ScanCutAngles
  {
    float fov_min;
    float fov_max;
    float scan_emit_angle;
  };

  struct DecodeFrame
  {
    NebulaPointCloudPtr pointcloud;
    uint64_t scan_timestamp_ns{0};
    std::optional<point_filters::BlockageMask> blockage_mask;
  };

  /// @brief Configuration for this decoder
  const std::shared_ptr<const drivers::HesaiSensorConfiguration> sensor_configuration_;

  /// @brief The sensor definition, used for return mode and time offset handling
  SensorT sensor_{};

  /// @brief A function that is called on each decoded pointcloud frame
  pointcloud_callback_t pointcloud_callback_;

  /// @brief Decodes azimuth/elevation angles given calibration/correction data
  typename SensorT::angle_corrector_t angle_corrector_;

  /// @brief Decodes functional safety data for supported sensors
  std::shared_ptr<FunctionalSafetyDecoderTypedBase<typename SensorT::packet_t>>
    functional_safety_decoder_;

  std::shared_ptr<PacketLossDetectorTypedBase<typename SensorT::packet_t>> packet_loss_detector_;

  /// @brief The last decoded packet
  typename SensorT::packet_t packet_;

  ScanCutAngles scan_cut_angles_;
  uint32_t last_frame_id_ = 0;

  std::shared_ptr<loggers::Logger> logger_;

  std::optional<point_filters::DownsampleMaskFilter> mask_filter_;

  std::shared_ptr<point_filters::BlockageMaskPlugin> blockage_mask_plugin_;

  /// @brief Decoded data of the frame currently being decoded to
  DecodeFrame decode_frame_;
  /// @brief Decoded data of the frame currently being output
  DecodeFrame output_frame_;

  /// @brief Validates and parse PandarPacket. Checks size and, if present, CRC checksums.
  /// @param packet The incoming PandarPacket
  /// @return Whether the packet was parsed successfully
  bool parse_packet(const std::vector<uint8_t> & packet)
  {
    if (packet.size() < sizeof(typename SensorT::packet_t)) {
      NEBULA_LOG_STREAM(
        logger_->error, "Packet size mismatch: " << packet.size() << " | Expected at least: "
                                                 << sizeof(typename SensorT::packet_t));
      return false;
    }

    if (!std::memcpy(&packet_, packet.data(), sizeof(typename SensorT::packet_t))) {
      logger_->error("Packet memcopy failed");
      return false;
    }

    return true;
  }

  /// @brief Converts each channel in the packet to a NebulaPoint and appends it to the point cloud
  /// @param start_block_id Unused for FTX (always 0)
  /// @param n_returns Unused for FTX (always single-return)
  void convert_returns(size_t start_block_id, size_t /*n_returns*/)
  {
    (void)start_block_id;

    uint64_t packet_timestamp_ns = hesai_packet::get_timestamp_ns(packet_);
    
    // For FT series (FTX140, FTX180, etc.), the packet tail contains row_id and column_id
    // which represent the "packet row" and "packet col" (pk_row, pk_col).
    const uint32_t pk_row = packet_.tail.row_id;
    const uint32_t pk_col = packet_.tail.column_id;

    // If the blockage mask plugin is not present, we can return early if distance checks fail
    const bool filters_can_return_early = !blockage_mask_plugin_;

    std::vector<const typename SensorT::packet_t::body_t::block_t::unit_t *> dummy_return_units;
    const auto return_type = sensor_.get_return_type(
      static_cast<hesai_packet::return_mode::ReturnMode>(packet_.tail.return_mode),
      1 /* single return for FTX */, dummy_return_units);

    for (size_t ch = 0; ch < SensorT::packet_t::n_channels; ++ch) {
      // Calculate true pixel row and column from the 1-indexed channel id (as in the manual):
      //   ch_row = pk_row * 6 + (ch - 1) % 6
      //   ch_col = pk_col * 16 + (ch - 1) / 6
      // ch is 0-indexed in code, so ch_1 = ch + 1 is the 1-indexed channel.
      const uint32_t ch_1 = ch + 1;
      const uint32_t ch_row = pk_row * 6 + (ch_1 - 1) % 6;
      const uint32_t ch_col = pk_col * 16 + (ch_1 - 1) / 6;

      // Bounds check against sensor's maximum dimensions
      if (ch_row >= SensorT::row_N || ch_col >= SensorT::col_N) {
        continue;
      }

      auto & unit = packet_.body.blocks[0].units[ch];

      const CorrectedAngleData corrected_angle_data =
        angle_corrector_.get_corrected_angle_data(ch_row, ch_col);

      bool point_is_valid = true;

      if (unit.distance == 0) {
        point_is_valid = false;
      }

      float distance = get_distance(unit);

      if (
        distance < SensorT::min_range || SensorT::max_range < distance ||
        distance < sensor_configuration_->min_range ||
        sensor_configuration_->max_range < distance) {
        point_is_valid = false;
      }

      if (filters_can_return_early && !point_is_valid) {
        continue;
      }

      float azimuth = corrected_angle_data.azimuth_rad;

      const float max_angle = static_cast<float>(2. * M_PI);
      const float azimuth_norm = normalize_angle(azimuth, max_angle);
      const float fov_min_norm = normalize_angle(scan_cut_angles_.fov_min, max_angle);
      const float fov_max_norm = normalize_angle(scan_cut_angles_.fov_max, max_angle);

      const bool in_fov = angle_is_between(fov_min_norm, fov_max_norm, azimuth_norm);
      if (!in_fov) {
        continue;
      }

      bool in_current_scan = true;

      auto & frame = in_current_scan ? decode_frame_ : output_frame_;

      if (frame.blockage_mask) {
        frame.blockage_mask->update(azimuth, ch, sensor_.get_blockage_type(unit.distance));
      }

      if (!point_is_valid) {
        continue;
      }

      NebulaPoint point;
      point.distance = distance;
      point.intensity = unit.reflectivity;
      point.time_stamp = packet_timestamp_ns - frame.scan_timestamp_ns;

      point.return_type = static_cast<uint8_t>(return_type);
      point.channel = ch;

      // Use sin/cos functions from calibration data from corrected_angle_data
      const float xy_distance = distance * corrected_angle_data.cos_elevation;
      point.x = xy_distance * corrected_angle_data.sin_azimuth;
      point.y = xy_distance * corrected_angle_data.cos_azimuth;
      point.z = distance * corrected_angle_data.sin_elevation;

      // The driver wrapper converts to degrees, expects radians
      point.azimuth = corrected_angle_data.azimuth_rad;
      point.elevation = corrected_angle_data.elevation_rad;

      if (!mask_filter_ || !mask_filter_->excluded(point)) {
        frame.pointcloud->emplace_back(point);
      }
    }
  }

  /// @brief Get the distance of the given unit in meters
  float get_distance(const typename SensorT::packet_t::body_t::block_t::unit_t & unit)
  {
    return unit.distance * (static_cast<double>(packet_.header.dis_unit) / 1000.0);
  }

  /// @brief Get timestamp of point in nanoseconds, relative to scan timestamp. Includes firing time
  /// offset correction for channel and block
  /// @param scan_timestamp_ns Start timestamp of the current scan in nanoseconds
  /// @param packet_timestamp_ns The timestamp of the current PandarPacket in nanoseconds
  /// @param block_id The block index of the point
  /// @param channel_id The channel index of the point
  uint32_t get_point_time_relative(
    uint64_t scan_timestamp_ns, uint64_t packet_timestamp_ns, size_t block_id, size_t channel_id)
  {
    (void)block_id;
    (void)channel_id;

    // this is a flash solid state LIDAR, point_to_packet_offset_ns is 0 as measurements comes from
    // the same light emission and there is non need to correct packet_to_scan_offset_ns
    auto packet_to_scan_offset_ns = static_cast<uint32_t>(packet_timestamp_ns - scan_timestamp_ns);
    return packet_to_scan_offset_ns;
  }

  DecodeFrame initialize_frame() const
  {
    DecodeFrame frame = {std::make_shared<NebulaPointCloud>(), 0, std::nullopt};
    frame.pointcloud->reserve(SensorT::max_scan_buffer_points);

    if (blockage_mask_plugin_) {
      frame.blockage_mask = point_filters::BlockageMask(
        SensorT::fov_mdeg.azimuth, blockage_mask_plugin_->get_bin_width_mdeg(),
        SensorT::packet_t::n_channels);
    }

    return frame;
  }

  /// @brief Called when a scan is complete, published and then clears the output frame.
  void on_scan_complete()
  {
    double scan_timestamp_s = static_cast<double>(output_frame_.scan_timestamp_ns) * 1e-9;

    if (pointcloud_callback_) {
      pointcloud_callback_(output_frame_.pointcloud, scan_timestamp_s);
    }

    if (blockage_mask_plugin_ && output_frame_.blockage_mask) {
      blockage_mask_plugin_->callback_and_reset(
        output_frame_.blockage_mask.value(), scan_timestamp_s);
    }

    output_frame_.pointcloud->clear();
  }

public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  /// @param correction_data Calibration data for this decoder
  explicit HesaiSolidStateDecoder(
    const std::shared_ptr<const HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const typename SensorT::angle_corrector_t::correction_data_t> &
      correction_data,
    const std::shared_ptr<loggers::Logger> & logger,
    const std::shared_ptr<FunctionalSafetyDecoderTypedBase<typename SensorT::packet_t>> &
      functional_safety_decoder,
    const std::shared_ptr<PacketLossDetectorTypedBase<typename SensorT::packet_t>> &
      packet_loss_detector,
    std::shared_ptr<point_filters::BlockageMaskPlugin> blockage_mask_plugin)
  : sensor_configuration_(sensor_configuration),
    angle_corrector_(correction_data),
    functional_safety_decoder_(functional_safety_decoder),
    packet_loss_detector_(packet_loss_detector),
    scan_cut_angles_(
      {static_cast<float>(deg2rad(sensor_configuration_->cloud_min_angle)),
       static_cast<float>(deg2rad(sensor_configuration_->cloud_max_angle)),
       static_cast<float>(deg2rad(sensor_configuration_->cut_angle))}),
    logger_(logger),
    blockage_mask_plugin_(std::move(blockage_mask_plugin)),
    decode_frame_(initialize_frame()),
    output_frame_(initialize_frame())
  {
    if (sensor_configuration->downsample_mask_path) {
      mask_filter_ = point_filters::DownsampleMaskFilter(
        sensor_configuration->downsample_mask_path.value(), SensorT::fov_mdeg.azimuth,
        SensorT::peak_resolution_mdeg.azimuth, SensorT::packet_t::n_channels,
        logger_->child("Downsample Mask"), true, sensor_.get_dither_transform());
    }
  }

  void set_pointcloud_callback(pointcloud_callback_t callback) override
  {
    pointcloud_callback_ = std::move(callback);
  }

  PacketDecodeResult unpack(const std::vector<uint8_t> & packet) override
  {
    util::Stopwatch decode_watch;

    if (!parse_packet(packet)) {
      return {PerformanceCounters{decode_watch.elapsed_ns()}, DecodeError::PACKET_PARSE_FAILED};
    }
    if (packet_loss_detector_) {
      packet_loss_detector_->update(packet_);
    }

    // Even if the checksums of other parts of the packet are invalid, functional safety info
    // is still checked. This is a null-op for sensors that do not support functional safety.
    if (functional_safety_decoder_) {
      functional_safety_decoder_->update(packet_);
    }

    // FYI: This is where the CRC would be checked. Since this caused performance issues in the
    // past, and since the frame check sequence of the packet is already checked by the NIC, we skip
    // it here.

    // This is the first scan, set scan timestamp to whatever packet arrived first
    // It is valid for a flash LIDAR sensor as the FT120
    if (decode_frame_.scan_timestamp_ns == 0) {
      decode_frame_.scan_timestamp_ns = hesai_packet::get_timestamp_ns(packet_);
      last_frame_id_ = packet_.tail.frame_id;
    }

    bool did_scan_complete = false;
    const auto current_frame_id = packet_.tail.frame_id;

    // We have a new scan when frame_id changes
    if (last_frame_id_ != current_frame_id && last_frame_id_ != 0) {
      // Swapping decode_frame_ to output_frame_ so it can be published
      std::swap(decode_frame_, output_frame_);
      did_scan_complete = true;

      // The new scan starts with this packet
      decode_frame_.scan_timestamp_ns = hesai_packet::get_timestamp_ns(packet_);
      decode_frame_.pointcloud->clear();
    }

    if (decode_frame_.scan_timestamp_ns == 0) {
      decode_frame_.scan_timestamp_ns = hesai_packet::get_timestamp_ns(packet_);
    }

    convert_returns(0, 1);

    last_frame_id_ = current_frame_id;

    uint64_t decode_duration_ns = decode_watch.elapsed_ns();
    uint64_t callbacks_duration_ns = 0;

    if (did_scan_complete) {
      util::Stopwatch callback_watch;
      on_scan_complete();
      callbacks_duration_ns += callback_watch.elapsed_ns();
    }

    PacketMetadata metadata;
    metadata.packet_timestamp_ns = hesai_packet::get_timestamp_ns(packet_);
    metadata.did_scan_complete = did_scan_complete;
    return {PerformanceCounters{decode_duration_ns - callbacks_duration_ns}, metadata};
  }
};

}  // namespace nebula::drivers
