// Copyright 2024 TIER IV, Inc.
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

#ifndef NEBULA_HESAI_COMMON_H
#define NEBULA_HESAI_COMMON_H

#include "nebula_core_common/nebula_common.hpp"
#include "nebula_core_common/nebula_status.hpp"
#include "nebula_core_common/util/string_conversions.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
namespace nebula
{
namespace drivers
{

bool supports_functional_safety(const SensorModel & sensor_model);

struct AdvancedFunctionalSafetyConfiguration
{
  std::string error_definitions_path;
  std::vector<uint16_t> ignored_error_codes;

  friend std::ostream & operator<<(
    std::ostream & os, const AdvancedFunctionalSafetyConfiguration & arg)
  {
    os << "advanced\n  error definitions: " << arg.error_definitions_path;
    os << "\n  ignored codes: ";
    if (!arg.ignored_error_codes.empty()) {
      for (size_t i = 0; i < arg.ignored_error_codes.size(); ++i) {
        if (i > 0) os << ", ";
        os << "0x" << std::hex << arg.ignored_error_codes[i];
      }
    } else {
      os << "none";
    }
    return os;
  }
};

/// @brief Converts String to PTP Profile
/// @param ptp_profile Profile as String
/// @return Corresponding PtpProfile
inline PtpProfile ptp_profile_from_string(const std::string & ptp_profile)
{
  // Hesai
  auto tmp_str = ptp_profile;
  std::transform(tmp_str.begin(), tmp_str.end(), tmp_str.begin(), [](unsigned char c) {
    return std::tolower(c);
  });
  if (tmp_str == "1588v2") return PtpProfile::IEEE_1588v2;
  if (tmp_str == "802.1as") return PtpProfile::IEEE_802_1AS;
  if (tmp_str == "automotive") return PtpProfile::IEEE_802_1AS_AUTO;

  return PtpProfile::UNKNOWN_PROFILE;
}

/// @brief Convert PtpProfile enum to string (Overloading the << operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(std::ostream & os, nebula::drivers::PtpProfile const & arg)
{
  switch (arg) {
    case PtpProfile::IEEE_1588v2:
      os << "IEEE_1588v2";
      break;
    case PtpProfile::IEEE_802_1AS:
      os << "IEEE_802.1AS";
      break;
    case PtpProfile::IEEE_802_1AS_AUTO:
      os << "IEEE_802.1AS Automotive";
      break;
    case PtpProfile::UNKNOWN_PROFILE:
      os << "UNKNOWN";
      break;
  }
  return os;
}

/// @brief Converts String to PTP TransportType
/// @param transport_type Transport as String
/// @return Corresponding PtpTransportType
inline PtpTransportType ptp_transport_type_from_string(const std::string & transport_type)
{
  // Hesai
  auto tmp_str = transport_type;
  std::transform(tmp_str.begin(), tmp_str.end(), tmp_str.begin(), [](unsigned char c) {
    return std::tolower(c);
  });
  if (tmp_str == "udp") return PtpTransportType::UDP_IP;
  if (tmp_str == "l2") return PtpTransportType::L2;

  return PtpTransportType::UNKNOWN_TRANSPORT;
}

/// @brief Convert PtpTransportType enum to string (Overloading the << operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(std::ostream & os, nebula::drivers::PtpTransportType const & arg)
{
  switch (arg) {
    case PtpTransportType::UDP_IP:
      os << "UDP/IP";
      break;
    case PtpTransportType::L2:
      os << "L2";
      break;
    case PtpTransportType::UNKNOWN_TRANSPORT:
      os << "UNKNOWN";
      break;
  }
  return os;
}

/// @brief Converts String to PTP SwitchType
/// @param switch_type Switch as String
/// @return Corresponding PtpSwitchType
inline PtpSwitchType ptp_switch_type_from_string(const std::string & switch_type)
{
  // Hesai
  auto tmp_str = switch_type;
  std::transform(tmp_str.begin(), tmp_str.end(), tmp_str.begin(), [](unsigned char c) {
    return std::tolower(c);
  });
  if (tmp_str == "tsn") return PtpSwitchType::TSN;
  if (tmp_str == "non_tsn") return PtpSwitchType::NON_TSN;

  return PtpSwitchType::UNKNOWN_SWITCH;
}

/// @brief Convert PtpSwitchType enum to string (Overloading the << operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(std::ostream & os, nebula::drivers::PtpSwitchType const & arg)
{
  switch (arg) {
    case PtpSwitchType::TSN:
      os << "TSN";
      break;
    case PtpSwitchType::NON_TSN:
      os << "NON_TSN";
      break;
    case PtpSwitchType::UNKNOWN_SWITCH:
      os << "UNKNOWN";
      break;
  }
  return os;
}

/// @brief struct for Hesai sensor configuration
struct HesaiSensorConfiguration : public LidarConfigurationBase
{
  std::string multicast_ip;
  uint16_t gnss_port{};
  size_t udp_socket_receive_buffer_size_bytes{};
  uint16_t sync_angle{};
  double cut_angle{};
  double dual_return_distance_threshold{};
  std::string calibration_path;
  bool calibration_download_enabled;
  uint16_t rotation_speed;
  uint16_t cloud_min_angle;
  uint16_t cloud_max_angle;
  PtpProfile ptp_profile;
  uint8_t ptp_domain;
  PtpTransportType ptp_transport_type;
  PtpSwitchType ptp_switch_type;
  uint8_t ptp_lock_threshold;
  std::optional<std::string> downsample_mask_path;
  bool hires_mode;
  std::optional<uint32_t> blockage_mask_horizontal_bin_size_mdeg;
  std::optional<std::string> sync_diagnostics_topic;
  std::optional<AdvancedFunctionalSafetyConfiguration> functional_safety;
};
/// @brief Convert HesaiSensorConfiguration to string (Overloading the << operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(std::ostream & os, HesaiSensorConfiguration const & arg)
{
  os << "Hesai Sensor Configuration:" << '\n';
  os << static_cast<const LidarConfigurationBase &>(arg) << '\n';
  os << "Multicast: "
     << (arg.multicast_ip.empty() ? "disabled" : "enabled, group: " + arg.multicast_ip) << '\n';
  os << "GNSS Port: " << arg.gnss_port << '\n';
  os << "UDP Socket Receive Buffer Size: " << arg.udp_socket_receive_buffer_size_bytes << " B"
     << '\n';
  os << "Rotation Speed: " << arg.rotation_speed << '\n';
  os << "Sync Angle: " << arg.sync_angle << '\n';
  os << "Cut Angle: " << arg.cut_angle << '\n';
  os << "FoV Start: " << arg.cloud_min_angle << '\n';
  os << "FoV End: " << arg.cloud_max_angle << '\n';
  os << "Dual Return Distance Threshold: " << arg.dual_return_distance_threshold << '\n';
  os << "Calibration Path: " << arg.calibration_path << '\n';
  os << "Calibration Download: " << (arg.calibration_download_enabled ? "enabled" : "disabled")
     << '\n';
  os << "PTP Profile: " << arg.ptp_profile << '\n';
  os << "PTP Domain: " << std::to_string(arg.ptp_domain) << '\n';
  os << "PTP Transport Type: " << arg.ptp_transport_type << '\n';
  os << "PTP Switch Type: " << arg.ptp_switch_type << '\n';
  os << "High Resolution Mode: " << arg.hires_mode << '\n';
  os << "PTP Lock Threshold: " << std::to_string(arg.ptp_lock_threshold) << '\n';
  os << "High Resolution Mode: " << (arg.hires_mode ? "enabled" : "disabled") << '\n';
  os << "Downsample Filter: "
     << (arg.downsample_mask_path ? "enabled, path: " + arg.downsample_mask_path.value()
                                  : "disabled")
     << '\n';
  os << "Blockage Mask Output: "
     << (arg.blockage_mask_horizontal_bin_size_mdeg
           ? "enabled, horizontal bin size: " +
               std::to_string(arg.blockage_mask_horizontal_bin_size_mdeg.value()) + " mdeg"
           : "disabled")
     << '\n';
  os << "Synchronization Diagnostics: "
     << (arg.sync_diagnostics_topic ? ("enabled, topic: " + arg.sync_diagnostics_topic.value())
                                    : "disabled");

  if (supports_functional_safety(arg.sensor_model)) {
    os << '\n';
    os << "Functional Safety: ";
    if (arg.functional_safety) {
      os << *arg.functional_safety;
    } else {
      os << "basic";
    }
  }
  return os;
}

struct HesaiCalibrationConfigurationBase : public CalibrationConfigurationBase
{
  virtual nebula::Status load_from_bytes(const std::vector<uint8_t> & buf) = 0;
  virtual nebula::Status load_from_file(const std::string & calibration_file) = 0;
  virtual nebula::Status save_to_file_from_bytes(
    const std::string & calibration_file, const std::vector<uint8_t> & buf) = 0;

  [[nodiscard]] virtual std::tuple<float, float> get_fov_padding() const = 0;
};

/// @brief struct for Hesai calibration configuration
struct HesaiCalibrationConfiguration : public HesaiCalibrationConfigurationBase
{
  std::map<size_t, float> elev_angle_map;
  std::map<size_t, float> azimuth_offset_map;

  inline nebula::Status load_from_file(const std::string & calibration_file) override
  {
    std::ifstream ifs(calibration_file);
    if (!ifs) {
      return Status::INVALID_CALIBRATION_FILE;
    }
    std::ostringstream ss;
    ss << ifs.rdbuf();  // reading data
    ifs.close();
    return load_from_string(ss.str());
  }

  nebula::Status load_from_bytes(const std::vector<uint8_t> & buf) override
  {
    std::string calibration_string = std::string(buf.begin(), buf.end());
    return load_from_string(calibration_string);
  }

  /// @brief Loading calibration data
  /// @param calibration_content
  /// @return Resulting status
  inline nebula::Status load_from_string(const std::string & calibration_content)
  {
    std::stringstream ss;
    ss << calibration_content;
    std::string line;
    constexpr size_t expected_cols = 3;
    while (std::getline(ss, line)) {
      boost::char_separator<char> sep(",");
      boost::tokenizer<boost::char_separator<char>> tok(line, sep);

      std::vector<std::string> actual_tokens(tok.begin(), tok.end());
      if (actual_tokens.size() < expected_cols || actual_tokens.size() > expected_cols) {
        std::cerr << "Ignoring line with unexpected data: " << line << std::endl;
        continue;
      }

      try {
        int laser_id = std::stoi(actual_tokens[0]);
        float elevation = std::stof(actual_tokens[1]);
        float azimuth = std::stof(actual_tokens[2]);
        elev_angle_map[laser_id - 1] = elevation;
        azimuth_offset_map[laser_id - 1] = azimuth;
      } catch (const std::invalid_argument & ia) {
        continue;
      }
    }
    return Status::OK;
  }

  /// @brief Saving calibration data (not used)
  /// @param calibration_file
  /// @return Resulting status
  inline nebula::Status save_to_file(const std::string & calibration_file)
  {
    std::ofstream ofs(calibration_file);
    if (!ofs) {
      return Status::CANNOT_SAVE_FILE;
    }
    ofs << "Laser id,Elevation,Azimuth" << std::endl;
    for (const auto & pair : elev_angle_map) {
      auto laser_id = pair.first + 1;
      float elevation = pair.second;
      float azimuth = azimuth_offset_map[pair.first];
      ofs << laser_id << "," << elevation << "," << azimuth << std::endl;
    }
    ofs.close();

    return Status::OK;
  }

  nebula::Status save_to_file_from_bytes(
    const std::string & calibration_file, const std::vector<uint8_t> & buf) override
  {
    std::string calibration_string = std::string(buf.begin(), buf.end());
    return save_file_from_string(calibration_file, calibration_string);
  }

  /// @brief Saving calibration data from string
  /// @param calibration_file path
  /// @param calibration_string calibration string
  /// @return Resulting status
  inline nebula::Status save_file_from_string(
    const std::string & calibration_file, const std::string & calibration_string)
  {
    std::ofstream ofs(calibration_file);
    if (!ofs) {
      return Status::CANNOT_SAVE_FILE;
    }
    ofs << calibration_string;
    ofs.close();
    return Status::OK;
  }

  [[nodiscard]] std::tuple<float, float> get_fov_padding() const override
  {
    float min = INFINITY;
    float max = -INFINITY;

    for (const auto & item : azimuth_offset_map) {
      min = std::min(min, item.second);
      max = std::max(max, item.second);
    }

    // NOTE: Slightly widen the FOV padding because some LiDARs do not transmit blocks near the end
    // of the FOV. If these blocks are missing, the point cloud may not be published at the desired
    // timing.
    min -= 1.0f;
    max += 1.0f;

    return {-max, -min};
  }
};

/// @brief struct for Hesai correction configuration (for AT)
struct HesaiCorrection : public HesaiCalibrationConfigurationBase
{
  uint16_t delimiter;
  uint8_t versionMajor;
  uint8_t versionMinor;
  uint8_t channelNumber;
  uint8_t mirrorNumber;
  uint8_t frameNumber;
  uint8_t frameConfig[8];
  uint8_t resolution;

  uint32_t startFrame[8];
  uint32_t endFrame[8];
  int32_t azimuth[128];
  int32_t elevation[128];
  int8_t azimuthOffset[36000];
  int8_t elevationOffset[36000];
  uint8_t SHA256[32];

  /// @brief Load correction data from file
  /// @param buf Binary buffer
  /// @return Resulting status
  inline nebula::Status load_from_bytes(const std::vector<uint8_t> & buf) override
  {
    size_t index;
    for (index = 0; index < buf.size() - 1; index++) {
      if (buf[index] == 0xee && buf[index + 1] == 0xff) break;
    }
    delimiter = (buf[index] & 0xff) << 8 | ((buf[index + 1] & 0xff));
    versionMajor = buf[index + 2] & 0xff;
    versionMinor = buf[index + 3] & 0xff;
    channelNumber = buf[index + 4] & 0xff;
    mirrorNumber = buf[index + 5] & 0xff;
    frameNumber = buf[index + 6] & 0xff;
    index += 7;
    for (uint8_t i = 0; i < 8; i++) {
      frameConfig[i] = buf[index] & 0xff;
      index++;
    }
    resolution = buf[index] & 0xff;
    index++;
    switch (versionMinor) {
      case 5:
        for (uint8_t i = 0; i < mirrorNumber; i++) {
          startFrame[i] = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 |
                          ((buf[index + 2] & 0xff) << 16) | ((buf[index + 3] & 0xff) << 24);
          index += 4;
        }
        for (uint8_t i = 0; i < mirrorNumber; i++) {
          endFrame[i] = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 |
                        ((buf[index + 2] & 0xff) << 16) | ((buf[index + 3] & 0xff) << 24);
          index += 4;
        }
        for (uint8_t i = 0; i < channelNumber; i++) {
          azimuth[i] = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 |
                       ((buf[index + 2] & 0xff) << 16) | ((buf[index + 3] & 0xff) << 24);
          index += 4;
        }
        for (uint8_t i = 0; i < channelNumber; i++) {
          elevation[i] = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 |
                         ((buf[index + 2] & 0xff) << 16) | ((buf[index + 3] & 0xff) << 24);
          index += 4;
        }
        for (int i = 0; i < channelNumber * 180; i++) {
          azimuthOffset[i] = buf[index] & 0xff;
          index++;
        }
        for (int i = 0; i < channelNumber * 180; i++) {
          elevationOffset[i] = buf[index] & 0xff;
          index++;
        }

        for (uint8_t i = 0; i < mirrorNumber; i++) {
          startFrame[i] *= resolution;
          endFrame[i] *= resolution;
        }
        for (uint8_t i = 0; i < channelNumber; i++) {
          azimuth[i] *= resolution;
          elevation[i] *= resolution;
        }
        for (int i = 0; i < channelNumber * 180; i++) {
          azimuthOffset[i] *= resolution;
          elevationOffset[i] *= resolution;
        }
        break;

      case 3:  // not worked...
        for (uint8_t i = 0; i < mirrorNumber; i++) {
          startFrame[i] = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8;
          index += 2;
        }
        for (uint8_t i = 0; i < mirrorNumber; i++) {
          endFrame[i] = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8;
          index += 2;
        }
        for (uint8_t i = 0; i < channelNumber; i++) {
          azimuth[i] = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8;
          index += 2;
        }
        for (uint8_t i = 0; i < channelNumber; i++) {
          elevation[i] = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8;
          index += 2;
        }
        for (int i = 0; i < 36000; i++) {
          azimuthOffset[i] = buf[index] & 0xff;
          index++;
        }
        for (int i = 0; i < 36000; i++) {
          elevationOffset[i] = buf[index] & 0xff;
          index++;
        }

        break;

      default:
        break;
    }
    return Status::OK;
  }

  /// @brief Load correction data from file
  /// @param correction_file path
  /// @return Resulting status
  inline nebula::Status load_from_file(const std::string & correction_file) override
  {
    std::ifstream ifs(correction_file, std::ios::in | std::ios::binary);
    if (!ifs) {
      return Status::INVALID_CALIBRATION_FILE;
    }
    std::vector<unsigned char> buf;
    //    int cnt = 0;
    while (!ifs.eof()) {
      unsigned char c;
      ifs.read(reinterpret_cast<char *>(&c), sizeof(unsigned char));
      buf.emplace_back(c);
    }
    load_from_bytes(buf);

    ifs.close();
    return Status::OK;
  }

  /// @brief Save correction data from binary buffer
  /// @param correction_file path
  /// @param buf correction binary
  /// @return Resulting status
  inline nebula::Status save_to_file_from_bytes(
    const std::string & correction_file, const std::vector<uint8_t> & buf) override
  {
    std::ofstream ofs(correction_file, std::ios::trunc | std::ios::binary);
    if (!ofs) {
      std::cerr << "Could not create file: " << correction_file << "\n";
      return Status::CANNOT_SAVE_FILE;
    }
    bool sop_received = false;
    for (const auto & byte : buf) {
      if (!sop_received) {
        if (byte == 0xEE) {
          sop_received = true;
        }
      }
      if (sop_received) {
        ofs << byte;
      }
    }
    ofs.close();
    if (sop_received) return Status::OK;
    return Status::INVALID_CALIBRATION_FILE;
  }

  static const int g_step3 = 200 * 256;

  /// @brief Get azimuth adjustment for channel and precision azimuth
  /// @param ch The channel id
  /// @param azi The precision azimuth in (0.01 / 256) degree unit
  /// @return The azimuth adjustment in 0.01 degree unit
  [[nodiscard]] int8_t get_azimuth_adjust_v3(uint8_t ch, uint32_t azi) const
  {
    unsigned int i = std::floor(1.f * azi / g_step3);
    unsigned int l = azi - i * g_step3;
    float k = 1.f * l / g_step3;
    return round((1 - k) * azimuthOffset[ch * 180 + i] + k * azimuthOffset[ch * 180 + i + 1]);
  }

  /// @brief Get elevation adjustment for channel and precision azimuth
  /// @param ch The channel id
  /// @param azi The precision azimuth in (0.01 / 256) degree unit
  /// @return The elevation adjustment in 0.01 degree unit
  [[nodiscard]] int8_t get_elevation_adjust_v3(uint8_t ch, uint32_t azi) const
  {
    unsigned int i = std::floor(1.f * azi / g_step3);
    unsigned int l = azi - i * g_step3;
    float k = 1.f * l / g_step3;
    return round((1 - k) * elevationOffset[ch * 180 + i] + k * elevationOffset[ch * 180 + i + 1]);
  }

  [[nodiscard]] std::tuple<float, float> get_fov_padding() const override
  {
    // TODO(mojomex): calculate instead of hard-coding
    // The reason this is tricky is that an upper bound over all azimuth/elevation combinations has
    // to be found. For other sensors, this is only a function of elevation, so the search space is
    // tiny compared to AT128. We should be able to find an upper bound of `getAzimuthAdjustV3` but
    // I have not invested the time for now.
    return {-5, 5};
  }
};

namespace FT
{
static constexpr int FT2_CORRECTION_LEN = 256;
static constexpr int FT2_ROW_MAX = 192;  // Max rows (FTX180 192x224, FTX140 192x256)
static constexpr int FT2_COL_MAX = 256;

#pragma pack(push, 1)
struct CorrectionDis
{
  float x;
  float y;
  float z;
};

struct CorrectionV3
{
  uint16_t m_u16Delimiter;   // 0xee 0xff
  uint8_t m_u8VersionMajor;  // 0x07
  uint8_t m_u8VersionMinor;  // 0x03
  uint8_t m_u8Reserved0;
  uint8_t m_u8Reserved1;
  uint16_t m_u16TotalRow;
  uint16_t m_u16TotalCol;
  float f;
  float cx;
  float cy;
  float k1;
  float k2;
  float k3;
  float k4;
  float p1;
  float p2;
  uint8_t m_u8Sha256[32];

  float distortion_func(float theta) const
  {
    return theta + k1 * std::pow(theta, 3) + k2 * std::pow(theta, 5) + k3 * std::pow(theta, 7) +
           k4 * std::pow(theta, 9);
  }

  std::vector<std::vector<float>> generate_pixel_vectors() const
  {
    float phi, phi_rad, alpha_rad;
    alpha_rad = 2 * atan(p1);
    if (alpha_rad < 0) {
      alpha_rad = -alpha_rad;
      phi = p2 + M_PI;
    } else {
      phi = p2;
    }
    phi_rad = (phi / M_PI / 2 - int(phi / M_PI / 2)) * 2 * M_PI + M_PI;

    float R[3][3];  // rotation matrix
    float cos_a = cos(alpha_rad);
    float sin_a = sin(alpha_rad);
    float cos_p = cos(phi_rad);
    float sin_p = sin(phi_rad);
    R[0][0] = cos_a + (1 - cos_a) * cos_p * cos_p;
    R[0][1] = (1 - cos_a) * cos_p * sin_p;
    R[0][2] = sin_a * sin_p;
    R[1][0] = (1 - cos_a) * cos_p * sin_p;
    R[1][1] = cos_a + (1 - cos_a) * sin_p * sin_p;
    R[1][2] = -sin_a * cos_p;
    R[2][0] = -sin_a * sin_p;
    R[2][1] = sin_a * cos_p;
    R[2][2] = cos_a;

    float theta_th =
      std::pow(std::pow(std::max(cx, 767.f - cx), 2) + std::pow(std::max(cy, 575.f - cy), 2), 0.5) /
      f;
    float tan_theta_th = 750.0 / f;

    int n_interp = 1024;
    std::vector<float> theta_arr(n_interp, 0.0);
    std::vector<float> theta_d_arr(n_interp, 0.0);
    for (int i = 0; i < n_interp; ++i) {
      theta_arr[i] = i * tan_theta_th / n_interp;
      theta_d_arr[i] = distortion_func(theta_arr[i]);
      if (i > 0) {
        if (theta_d_arr[i] < theta_d_arr[i - 1]) {
          n_interp = i;
          break;
        }
      }
    }
    theta_arr.resize(n_interp);
    theta_d_arr.resize(n_interp);

    std::vector<std::vector<float>> pixel_vectors(
      m_u16TotalRow * m_u16TotalCol, std::vector<float>(3, 0));

    int init_row = (192 - m_u16TotalRow) >> 1;
    int init_col = (256 - m_u16TotalCol) >> 1;

    for (int i = init_row; i < m_u16TotalRow + init_row; ++i) {
      for (int j = init_col; j < (m_u16TotalCol + init_col); ++j) {
        float x = (j * 3 + 1 - cx) / f;
        float y = (575 - (i * 3 + 1) - cy) / f;
        float x1 = x * R[0][0] + y * R[0][1] + 0;
        float y1 = x * R[1][0] + y * R[1][1] + 0;
        float z1 = x * R[2][0] + y * R[2][1] + 0;
        float x2 = -x1 / (z1 - 1);
        float y2 = -y1 / (z1 - 1);
        float theta_d = sqrt(pow(x2, 2) + pow(y2, 2));

        std::vector<float> pvec = {0, 0, 0};
        if (theta_d == 0) {
          pvec = {0.0, 0.0, 1.0};
        } else if (theta_d > theta_th) {
          pvec = {0.0, 0.0, 0.0};
        } else {
          auto it = std::lower_bound(theta_d_arr.begin(), theta_d_arr.end(), theta_d);
          int pos = std::distance(theta_d_arr.begin(), it) - 1;
          if (pos < 0 || pos >= (int)theta_d_arr.size() - 1) {
            pvec = {0.0, 0.0, 0.0};
          } else {
            float theta = (theta_arr[pos + 1] - theta_arr[pos]) /
                            (theta_d_arr[pos + 1] - theta_d_arr[pos]) *
                            (theta_d - theta_d_arr[pos]) +
                          theta_arr[pos];
            float tan_theta = tan(theta);
            pvec = {x2, y2, static_cast<float>(sqrt(x2 * x2 + y2 * y2) / tan_theta)};
            float pvec_norm = sqrt(pow(pvec[0], 2) + pow(pvec[1], 2) + pow(pvec[2], 2));
            pvec[0] /= pvec_norm;
            pvec[1] /= pvec_norm;
            pvec[2] /= pvec_norm;
          }
        }
        pixel_vectors[(i - init_row) * m_u16TotalCol + (j - init_col)] = {
          pvec[2], -pvec[0], -pvec[1]};
      }
    }
    return pixel_vectors;
  }
};
#pragma pack(pop)
}  // namespace FT

/// @brief struct for Hesai correction configuration (FTX series)
struct HesaiCorrectionFTX : public HesaiCalibrationConfiguration
{
  FT::CorrectionV3 correction_v3_{};
  std::array<FT::CorrectionDis, FT::FT2_CORRECTION_LEN * FT::FT2_CORRECTION_LEN> xyz_corrections_{};
  bool valid_ = false;

  /// @brief Load correction data from byte array downloaded from sensor over TCP
  /// @param buf Binary buffer
  /// @return Resulting status
  inline nebula::Status load_from_bytes(const std::vector<uint8_t> & buf) override
  {
    if (buf.size() < 2 || buf[0] != 0xEE || buf[1] != 0xFF) {
      if (buf.size() >= FT::FT2_ROW_MAX * FT::FT2_COL_MAX * sizeof(float) * 3) {
        // Raw float array (fallback)
        const uint8_t * correction_string = buf.data();
        memset(&correction_v3_, 0, sizeof(FT::CorrectionV3));
        correction_v3_.m_u16TotalRow = FT::FT2_ROW_MAX;
        correction_v3_.m_u16TotalCol = FT::FT2_COL_MAX;
        for (int row = 0; row < FT::FT2_ROW_MAX; row++) {
          for (int col = 0; col < FT::FT2_COL_MAX; col++) {
            xyz_corrections_[row * FT::FT2_CORRECTION_LEN + col].x =
              *((const float *)correction_string);
            correction_string += sizeof(float);
            xyz_corrections_[row * FT::FT2_CORRECTION_LEN + col].y =
              *((const float *)correction_string);
            correction_string += sizeof(float);
            xyz_corrections_[row * FT::FT2_CORRECTION_LEN + col].z =
              *((const float *)correction_string);
            correction_string += sizeof(float);
          }
        }
        valid_ = true;
        return Status::OK;
      }
      return Status::INVALID_CALIBRATION_FILE;
    }

    if (buf.size() < sizeof(FT::CorrectionV3)) {
      return Status::INVALID_CALIBRATION_FILE;
    }

    memcpy(&correction_v3_, buf.data(), sizeof(FT::CorrectionV3));
    if (correction_v3_.m_u8VersionMajor != 0x07 || correction_v3_.m_u8VersionMinor != 0x03) {
      return Status::INVALID_CALIBRATION_FILE;
    }

    auto pixel_vectors = correction_v3_.generate_pixel_vectors();
    for (int i = 0; i < correction_v3_.m_u16TotalRow; i++) {
      for (int j = 0; j < correction_v3_.m_u16TotalCol; j++) {
        xyz_corrections_[i * FT::FT2_CORRECTION_LEN + j].x =
          pixel_vectors[i * correction_v3_.m_u16TotalCol + j][0];
        xyz_corrections_[i * FT::FT2_CORRECTION_LEN + j].y =
          pixel_vectors[i * correction_v3_.m_u16TotalCol + j][1];
        xyz_corrections_[i * FT::FT2_CORRECTION_LEN + j].z =
          pixel_vectors[i * correction_v3_.m_u16TotalCol + j][2];
      }
    }
    valid_ = true;

    return Status::OK;
  }

  /// @brief Load correction data from 5-column CSV string
  /// @param calibration_content String formatted data
  /// @return Resulting status
  inline nebula::Status load_from_string(const std::string & calibration_content)
  {
    std::stringstream ifs(calibration_content);
    std::string line;
    std::getline(ifs, line);  // header

    int rowIdMax = 0, cloumnIdMax = 0;
    while (std::getline(ifs, line)) {
      if (line.empty() || line.length() < 9) {
        continue;
      }
      float x, y, z;
      int rowId = 0, cloumnId = 0;
      std::stringstream ss(line);
      std::string subline;
      std::getline(ss, subline, ',');
      if (subline.empty()) continue;
      rowId = std::stoi(subline);
      std::getline(ss, subline, ',');
      cloumnId = std::stoi(subline);
      std::getline(ss, subline, ',');
      x = std::stof(subline);
      std::getline(ss, subline, ',');
      y = std::stof(subline);
      std::getline(ss, subline, ',');
      z = std::stof(subline);

      if (rowId > FT::FT2_CORRECTION_LEN || cloumnId > FT::FT2_CORRECTION_LEN) {
        return Status::INVALID_CALIBRATION_FILE;
      }
      if (rowId % 2 == 0 && cloumnId % 2 == 0) {
        rowId /= 2;
        cloumnId /= 2;
        xyz_corrections_[rowId * FT::FT2_CORRECTION_LEN + cloumnId].x = x;
        xyz_corrections_[rowId * FT::FT2_CORRECTION_LEN + cloumnId].y = y;
        xyz_corrections_[rowId * FT::FT2_CORRECTION_LEN + cloumnId].z = z;
        rowIdMax = std::max(rowIdMax, rowId);
        cloumnIdMax = std::max(cloumnIdMax, cloumnId);
      }
    }

    correction_v3_.m_u16TotalRow = rowIdMax + 1;
    correction_v3_.m_u16TotalCol = cloumnIdMax + 1;
    valid_ = true;

    return Status::OK;
  }

  /// @brief Load correction data from file
  /// @param correction_file path
  /// @return Resulting status
  inline nebula::Status load_from_file(const std::string & correction_file) override
  {
    if (
      correction_file.substr(correction_file.find_last_of(".") + 1) == "csv" ||
      correction_file.substr(correction_file.find_last_of(".") + 1) == "CSV") {
      std::ifstream ifs(correction_file);
      if (!ifs) return Status::INVALID_CALIBRATION_FILE;
      std::ostringstream ss;
      ss << ifs.rdbuf();
      ifs.close();
      return load_from_string(ss.str());
    }

    std::ifstream ifs(correction_file, std::ios::in | std::ios::binary);
    if (!ifs) {
      return Status::INVALID_CALIBRATION_FILE;
    }
    ifs.seekg(0, std::ios::end);
    int len = static_cast<int>(ifs.tellg());
    ifs.seekg(0, std::ios::beg);
    std::vector<uint8_t> buf(len);
    ifs.read(reinterpret_cast<char *>(buf.data()), len);
    ifs.close();

    return load_from_bytes(buf);
  }

  /// @brief Save correction data from binary buffer
  /// @param correction_file path
  /// @param buf correction binary
  /// @return Resulting status
  inline nebula::Status save_to_file_from_bytes(
    const std::string & correction_file, const std::vector<uint8_t> & buf) override
  {
    std::ofstream ofs(correction_file, std::ios::trunc | std::ios::binary);
    if (!ofs) {
      std::cerr << "Could not create file: " << correction_file << "\n";
      return Status::CANNOT_SAVE_FILE;
    }
    bool sop_received = false;
    for (const auto & byte : buf) {
      if (!sop_received) {
        if (byte == 0xEE) {
          sop_received = true;
        }
      }
      if (sop_received) {
        ofs << byte;
      }
    }
    ofs.close();
    if (sop_received) return Status::OK;
    return Status::INVALID_CALIBRATION_FILE;
  }
};

/*
<option value="0">Last Return</option>
<option value="1">Strongest Return</option>
<option value="3">First Return</option>
<option value="2">Last Return + Strongest Return</option>
<option value="4">First Return + Last Return</option>
<option value="5">First Return + Strongest Return</option>
*/
/*
<option value="0">Last Return</option>
<option value="1">Strongest Return</option>
<option value="3">First Return</option>
<option value="2">Last Return + Strongest Return</option>
<option value="4">First Return + Strongest Return</option>
<option value="5">First Return + Last Return</option>
<option value="6">First Return + Last Return + Strongest Return</option>
*/

/// @brief Convert return mode name to ReturnMode enum (Hesai-specific return_mode_from_string)
/// @param return_mode Return mode name (Upper and lower case letters must match)
/// @param sensor_model Model for correct conversion
/// @return Corresponding ReturnMode
inline ReturnMode return_mode_from_string_hesai(
  const std::string & return_mode, const SensorModel & sensor_model)
{
  switch (sensor_model) {
    case SensorModel::HESAI_PANDARXT16:
    case SensorModel::HESAI_PANDARXT32:
    case SensorModel::HESAI_PANDARXT32M:
    case SensorModel::HESAI_PANDAR128_E3X:
    case SensorModel::HESAI_PANDAR128_E4X:
    case SensorModel::HESAI_PANDARQT128:
      if (return_mode == "Last") return ReturnMode::LAST;
      if (return_mode == "Strongest") return ReturnMode::STRONGEST;
      if (return_mode == "Dual" || return_mode == "LastStrongest")
        return ReturnMode::DUAL_LAST_STRONGEST;
      if (return_mode == "First") return ReturnMode::FIRST;
      if (return_mode == "LastFirst") return ReturnMode::DUAL_LAST_FIRST;
      if (return_mode == "FirstStrongest") return ReturnMode::DUAL_FIRST_STRONGEST;
      break;
    case SensorModel::HESAI_PANDARJT128:
      if (return_mode == "Last") return ReturnMode::LAST;
      if (return_mode == "Strongest") return ReturnMode::STRONGEST;
      if (return_mode == "Dual" || return_mode == "LastStrongest")
        return ReturnMode::DUAL_LAST_STRONGEST;
      if (return_mode == "First") return ReturnMode::FIRST;
      if (return_mode == "LastFirst") return ReturnMode::DUAL_LAST_FIRST;
      if (return_mode == "FirstStrongest") return ReturnMode::DUAL_FIRST_STRONGEST;
      break;
    case SensorModel::HESAI_FTX140:
    case SensorModel::HESAI_FTX180:
      if (return_mode == "Last") return ReturnMode::LAST;
      if (return_mode == "Strongest") return ReturnMode::STRONGEST;
      if (return_mode == "Dual" || return_mode == "LastStrongest")
        return ReturnMode::DUAL_LAST_STRONGEST;
      if (return_mode == "First") return ReturnMode::FIRST;
      if (return_mode == "LastFirst") return ReturnMode::DUAL_LAST_FIRST;
      if (return_mode == "FirstStrongest") return ReturnMode::DUAL_FIRST_STRONGEST;
      break;
    case SensorModel::HESAI_PANDARQT64:
      if (return_mode == "Last") return ReturnMode::LAST;
      if (return_mode == "Dual" || return_mode == "LastFirst") return ReturnMode::DUAL_LAST_FIRST;
      if (return_mode == "First") return ReturnMode::FIRST;
      break;
    case SensorModel::HESAI_PANDARAT128:
    case SensorModel::HESAI_PANDAR64:
    case SensorModel::HESAI_PANDAR40P:
      if (return_mode == "Last") return ReturnMode::LAST;
      if (return_mode == "Strongest") return ReturnMode::STRONGEST;
      if (return_mode == "Dual" || return_mode == "LastStrongest")
        return ReturnMode::DUAL_LAST_STRONGEST;
      break;
    default:
      throw std::runtime_error("Unsupported sensor model: " + util::to_string(sensor_model));
  }

  return ReturnMode::UNKNOWN;
}

/// @brief Convert return mode number to ReturnMode enum
/// @param return_mode Return mode number from the hardware response
/// @param sensor_model Model for correct conversion
/// @return Corresponding ReturnMode
inline ReturnMode return_mode_from_int_hesai(
  const int return_mode, const SensorModel & sensor_model)
{
  switch (sensor_model) {
    case SensorModel::HESAI_PANDARXT16:
    case SensorModel::HESAI_PANDARXT32:
    case SensorModel::HESAI_PANDARXT32M:
    case SensorModel::HESAI_PANDAR128_E3X:
    case SensorModel::HESAI_PANDAR128_E4X:
    case SensorModel::HESAI_PANDARQT128:
      if (return_mode == 0) return ReturnMode::LAST;
      if (return_mode == 1) return ReturnMode::STRONGEST;
      if (return_mode == 2) return ReturnMode::DUAL_LAST_STRONGEST;
      if (return_mode == 3) return ReturnMode::FIRST;
      if (return_mode == 4) return ReturnMode::DUAL_LAST_FIRST;
      if (return_mode == 5) return ReturnMode::DUAL_FIRST_STRONGEST;
      break;
    case SensorModel::HESAI_PANDARJT128:
      if (return_mode == 0) return ReturnMode::LAST;
      if (return_mode == 1) return ReturnMode::STRONGEST;
      if (return_mode == 2) return ReturnMode::DUAL_LAST_STRONGEST;
      if (return_mode == 3) return ReturnMode::FIRST;
      if (return_mode == 4) return ReturnMode::DUAL_LAST_FIRST;
      if (return_mode == 5) return ReturnMode::DUAL_FIRST_STRONGEST;
      break;
    case SensorModel::HESAI_FTX140:
    case SensorModel::HESAI_FTX180:
      if (return_mode == 1 || return_mode == 0x37) return ReturnMode::STRONGEST;
      if (return_mode == 0 || return_mode == 0x38) return ReturnMode::LAST;
      if (return_mode == 3 || return_mode == 0x33) return ReturnMode::FIRST;
      if (sensor_model == SensorModel::HESAI_PANDARQT64) {
        if (return_mode == 2 || return_mode == 0x39) return ReturnMode::DUAL_LAST_FIRST;
      } else {
        if (return_mode == 2 || return_mode == 0x39) return ReturnMode::DUAL_LAST_STRONGEST;
        if (return_mode == 4 || return_mode == 0x3b) return ReturnMode::DUAL_LAST_FIRST;
        if (return_mode == 5 || return_mode == 0x3c) return ReturnMode::DUAL_FIRST_STRONGEST;
      }
      break;
    case SensorModel::HESAI_PANDARQT64:
      if (return_mode == 0) return ReturnMode::LAST;
      if (return_mode == 2) return ReturnMode::DUAL_LAST_FIRST;
      if (return_mode == 3) return ReturnMode::FIRST;
      break;
    case SensorModel::HESAI_PANDARAT128:
    case SensorModel::HESAI_PANDAR64:
    case SensorModel::HESAI_PANDAR40P:
      if (return_mode == 0) return ReturnMode::LAST;
      if (return_mode == 1) return ReturnMode::STRONGEST;
      if (return_mode == 2) return ReturnMode::DUAL_LAST_STRONGEST;
      break;
    default:
      throw std::runtime_error("Unsupported sensor model: " + util::to_string(sensor_model));
  }

  return ReturnMode::UNKNOWN;
}

/// @brief Convert ReturnMode enum to return mode number
/// @param return_mode target ReturnMode
/// @param sensor_model Model for correct conversion
/// @return Corresponding return mode number for the hardware
inline int int_from_return_mode_hesai(
  const ReturnMode return_mode, const SensorModel & sensor_model)
{
  switch (sensor_model) {
    case SensorModel::HESAI_PANDARXT16:
    case SensorModel::HESAI_PANDARXT32:
    case SensorModel::HESAI_PANDARXT32M:
    case SensorModel::HESAI_PANDAR128_E3X:
    case SensorModel::HESAI_PANDAR128_E4X:
    case SensorModel::HESAI_PANDARQT128:
      if (return_mode == ReturnMode::LAST) return 0;
      if (return_mode == ReturnMode::STRONGEST) return 1;
      if (return_mode == ReturnMode::DUAL || return_mode == ReturnMode::DUAL_LAST_STRONGEST)
        return 2;
      if (return_mode == ReturnMode::FIRST) return 3;
      if (return_mode == ReturnMode::DUAL_LAST_FIRST) return 4;
      if (return_mode == ReturnMode::DUAL_FIRST_STRONGEST) return 5;
      break;
    case SensorModel::HESAI_PANDARJT128:
      if (return_mode == ReturnMode::LAST) return 0;
      if (return_mode == ReturnMode::STRONGEST) return 1;
      if (return_mode == ReturnMode::DUAL || return_mode == ReturnMode::DUAL_LAST_STRONGEST)
        return 2;
      if (return_mode == ReturnMode::FIRST) return 3;
      if (return_mode == ReturnMode::DUAL_LAST_FIRST) return 4;
      if (return_mode == ReturnMode::DUAL_FIRST_STRONGEST) return 5;
      break;      
    case SensorModel::HESAI_PANDARQT64:
      if (return_mode == ReturnMode::LAST) return 0;
      if (return_mode == ReturnMode::DUAL || return_mode == ReturnMode::DUAL_LAST_FIRST) return 2;
      if (return_mode == ReturnMode::FIRST) return 3;
      break;
    case SensorModel::HESAI_PANDARAT128:
    case SensorModel::HESAI_PANDAR64:
    case SensorModel::HESAI_PANDAR40P:
      if (return_mode == ReturnMode::LAST) return 0;
      if (return_mode == ReturnMode::STRONGEST) return 1;
      if (return_mode == ReturnMode::DUAL || return_mode == ReturnMode::DUAL_LAST_STRONGEST)
        return 2;
      break;
    default:
      throw std::runtime_error("Unsupported sensor model: " + util::to_string(sensor_model));
  }

  return -1;
}

/// @brief Whether the given sensor model supports lidar monitor requests
/// @param sensor_model Sensor model
/// @return True if the sensor model supports lidar monitor, false otherwise
inline bool supports_lidar_monitor(const SensorModel & sensor_model)
{
  switch (sensor_model) {
    case drivers::SensorModel::HESAI_PANDARAT128:
    case drivers::SensorModel::HESAI_FTX140:
    case drivers::SensorModel::HESAI_FTX180:
    case drivers::SensorModel::HESAI_PANDAR40P:
    case drivers::SensorModel::HESAI_PANDAR64:
      return false;
    default:
      return true;
  }
}

/// @brief Whether the given sensor model supports functional safety
/// @param sensor_model Sensor model
/// @return True if the sensor model supports functional safety, false otherwise
inline bool supports_functional_safety(const SensorModel & sensor_model)
{
  switch (sensor_model) {
    case SensorModel::HESAI_PANDAR128_E3X:
    case SensorModel::HESAI_PANDAR128_E4X:
    case SensorModel::HESAI_PANDARQT128:
      return true;
    default:
      return false;
  }
}

/// @brief Whether the given sensor model supports packet loss detection
/// @param sensor_model Sensor model
/// @return True if the sensor model supports packet loss detection, false otherwise
inline bool supports_packet_loss_detection(const SensorModel & sensor_model)
{
  switch (sensor_model) {
    case SensorModel::HESAI_PANDAR64:
    case SensorModel::HESAI_PANDAR40P:
    case SensorModel::HESAI_PANDAR40M:
    case SensorModel::HESAI_PANDARQT128:
    case SensorModel::HESAI_PANDARAT128:
    case SensorModel::HESAI_FTX140:
    case SensorModel::HESAI_FTX180:
    case SensorModel::HESAI_PANDAR128_E3X:
    case SensorModel::HESAI_PANDAR128_E4X:
      return true;
    default:
      return false;
  }
}

/// @brief Whether the given sensor model supports blockage mask output
///
/// Blockage mask output is only supported for OT128.
///
/// The QT128 datasheet mentions support for blockage detection, but its output does not distinguish
/// between sky (no return) and blockage (too-close return). Thus, the output is not usable in the
/// same way as OT128's.
///
/// @param sensor_model Sensor model
/// @return True if the sensor model supports blockage mask output, false otherwise
inline bool supports_blockage_mask(const SensorModel & sensor_model)
{
  switch (sensor_model) {
    case SensorModel::HESAI_PANDAR128_E4X:
      return true;
    default:
      return false;
  }
}

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_HESAI_COMMON_H
