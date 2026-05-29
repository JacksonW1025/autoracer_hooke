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

#pragma once

#include "nebula_core_decoders/angles.hpp"
#include "nebula_hesai_common/hesai_common.hpp"
#include "nebula_hesai_decoders/decoders/angle_corrector.hpp"

#include <nebula_core_common/nebula_common.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <ostream>
#include <utility>

namespace nebula::drivers
{

template <size_t RowN, size_t ColumnN, size_t ReturnChannelN>
class AngleCorrectorCalibrationBasedSolidState
: public AngleCorrector<HesaiCalibrationConfiguration, ReturnChannelN>
{
private:
  std::array<std::array<CorrectedAngleData, RowN>, ColumnN> correctedAngleData;

public:
  explicit AngleCorrectorCalibrationBasedSolidState(
    const std::shared_ptr<const HesaiCalibrationConfiguration> & sensor_calibration)
  {
    if (sensor_calibration == nullptr) {
      throw std::runtime_error(
        "Cannot instantiate AngleCorrectorCalibrationBasedSolidState without calibration data");
    }

    // ////////////////////////////////////////
    // Build lookup table
    // ////////////////////////////////////////

    // Check if we have FTX-style XYZ corrections
    auto ftx_corrections = std::dynamic_pointer_cast<const HesaiCorrectionFTX>(sensor_calibration);
    if (ftx_corrections && ftx_corrections->valid_) {
      for (size_t j = 0; j < ColumnN; j++)  // column
      {
        for (size_t i = 0; i < RowN; i++)  // row
        {
          const auto & corr =
            ftx_corrections
              ->xyz_corrections_[i * 256 + j];  // ALWAYS stride 256 for FT binary calibration
          auto C = CorrectedAngleData();

          // Normalize the input vectors to ensure they are unit vectors.
          // This prevents "stretched" pointclouds if the calibration stores non-normalized vectors.
          float norm =
            static_cast<float>(std::sqrt(corr.x * corr.x + corr.y * corr.y + corr.z * corr.z));
          float cx = corr.x / (norm > 1e-6f ? norm : 1.0f);
          float cy = corr.y / (norm > 1e-6f ? norm : 1.0f);
          float cz = corr.z / (norm > 1e-6f ? norm : 1.0f);

          C.azimuth_rad = static_cast<float>(std::atan2(cx, cy));
          C.elevation_rad = static_cast<float>(std::asin(cz));

          C.sin_elevation = cz;
          C.cos_elevation = static_cast<float>(std::sqrt(cx * cx + cy * cy));

          if (C.cos_elevation > 1e-6f) {
            C.sin_azimuth = cx / C.cos_elevation;
            C.cos_azimuth = cy / C.cos_elevation;
          } else {
            C.sin_azimuth = 0.f;
            C.cos_azimuth = 1.f;
          }

          correctedAngleData[j][i] = C;
        }
      }
      return;
    }

    size_t calib_i = 0;

    // Res coefficient is usually not used here as CSV gives values in degrees
    const double res_coeff = M_PI / 180.;

    for (size_t j = 0; j < ColumnN; j++)  // column
    {
      for (size_t i = 0; i < RowN; i++)  // row
      {
        // Calibration format CSV: ID, Elevation, Azimuth
        // The ID is 1-indexed and maps to calib_i (0 to ColumnN*RowN - 1)
        if (
          sensor_calibration->elev_angle_map.find(calib_i) ==
          sensor_calibration->elev_angle_map.end()) {
          calib_i++;
          continue;  // Missing calibration point? Usually it contains all
        }

        const double azi = sensor_calibration->azimuth_offset_map.at(calib_i) * res_coeff;
        const double ele = sensor_calibration->elev_angle_map.at(calib_i) * res_coeff;

        ++calib_i;

        auto C = CorrectedAngleData();

        C.azimuth_rad = static_cast<float>(azi);
        C.elevation_rad = static_cast<float>(ele);
        C.sin_azimuth = static_cast<float>(sin(azi));
        C.cos_azimuth = static_cast<float>(cos(azi));
        C.sin_elevation = static_cast<float>(sin(ele));
        C.cos_elevation = static_cast<float>(cos(ele));

        correctedAngleData[j][i] = C;
      }
    }
  }

  [[nodiscard]] CorrectedAngleData get_corrected_angle_data(uint32_t row_id, uint32_t col_id) const
  {
    return correctedAngleData[col_id][row_id];
  }

  // This base method is not used for solid state sensor, as all angles came from
  // get_corrected_angle_data
  [[nodiscard]] CorrectedAzimuths<ReturnChannelN, float> get_corrected_azimuths(
    uint32_t block_azimuth) const override
  {
    // not used parameters
    (void)block_azimuth;
    return CorrectedAzimuths<ReturnChannelN, float>();
  };

  static bool passed_emit_angle(uint32_t last_frame_id, uint32_t current_frame_id)
  {
    // Generally FTX produces one packet or stream per frame; true frame breaks happen
    // when packet frame_id changes
    return last_frame_id != current_frame_id;
  }

  static bool passed_timestamp_reset_angle(uint32_t last_frame_id, uint32_t current_frame_id)
  {
    return last_frame_id != current_frame_id;
  }
};

}  // namespace nebula::drivers
