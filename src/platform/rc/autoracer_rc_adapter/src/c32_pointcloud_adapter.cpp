#include "autoracer_rc_adapter/c32_pointcloud_adapter.hpp"

#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoracer_rc_adapter
{
namespace
{

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

constexpr std::uint32_t kOutputPointStep = 16U;
constexpr std::uint8_t kSingleStrongestReturn = 1U;

const PointField & require_field(
  const PointCloud2 & cloud, const std::string & name,
  const std::uint8_t datatype, const std::size_t byte_size)
{
  const auto field = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [&name](const PointField & candidate) {return candidate.name == name;});
  if (field == cloud.fields.end()) {
    throw std::invalid_argument("C32 pointcloud is missing field: " + name);
  }
  if (field->datatype != datatype || field->count != 1U) {
    throw std::invalid_argument("C32 pointcloud field has incompatible type: " + name);
  }
  if (static_cast<std::uint64_t>(field->offset) + byte_size > cloud.point_step) {
    throw std::invalid_argument("C32 pointcloud field exceeds point_step: " + name);
  }
  return *field;
}

PointField make_field(
  const std::string & name, const std::uint32_t offset, const std::uint8_t datatype)
{
  PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1U;
  return field;
}

std::uint8_t normalize_intensity(const float intensity)
{
  if (!std::isfinite(intensity)) {
    return 0U;
  }
  const auto clamped = std::clamp(intensity, 0.0F, 255.0F);
  return static_cast<std::uint8_t>(std::lround(clamped));
}

}  // namespace

PointCloud2 convert_c32_pointcloud(const PointCloud2 & input)
{
  if (input.is_bigendian) {
    throw std::invalid_argument("Big-endian C32 pointclouds are not supported");
  }
  if (input.point_step == 0U && input.width != 0U && input.height != 0U) {
    throw std::invalid_argument("C32 pointcloud point_step is zero");
  }

  const auto & x_field = require_field(input, "x", PointField::FLOAT32, sizeof(float));
  const auto & y_field = require_field(input, "y", PointField::FLOAT32, sizeof(float));
  const auto & z_field = require_field(input, "z", PointField::FLOAT32, sizeof(float));
  const auto & intensity_field =
    require_field(input, "intensity", PointField::FLOAT32, sizeof(float));
  const auto & ring_field =
    require_field(input, "ring", PointField::UINT16, sizeof(std::uint16_t));

  const std::uint64_t minimum_input_row_step =
    static_cast<std::uint64_t>(input.width) * input.point_step;
  if (minimum_input_row_step > input.row_step) {
    throw std::invalid_argument("C32 pointcloud row_step is smaller than one row");
  }
  const std::uint64_t required_input_bytes = input.height == 0U ? 0U :
    static_cast<std::uint64_t>(input.height - 1U) * input.row_step + minimum_input_row_step;
  if (required_input_bytes > input.data.size()) {
    throw std::invalid_argument("C32 pointcloud data is shorter than its dimensions");
  }

  const std::uint64_t output_row_step =
    static_cast<std::uint64_t>(input.width) * kOutputPointStep;
  const std::uint64_t output_data_size =
    static_cast<std::uint64_t>(input.height) * output_row_step;
  if (output_row_step > std::numeric_limits<std::uint32_t>::max() ||
    output_data_size > std::numeric_limits<std::size_t>::max())
  {
    throw std::invalid_argument("C32 pointcloud dimensions overflow normalized layout");
  }

  PointCloud2 output;
  output.header = input.header;
  output.height = input.height;
  output.width = input.width;
  output.fields = {
    make_field("x", 0U, PointField::FLOAT32),
    make_field("y", 4U, PointField::FLOAT32),
    make_field("z", 8U, PointField::FLOAT32),
    make_field("intensity", 12U, PointField::UINT8),
    make_field("return_type", 13U, PointField::UINT8),
    make_field("channel", 14U, PointField::UINT16),
  };
  output.is_bigendian = false;
  output.point_step = kOutputPointStep;
  output.row_step = static_cast<std::uint32_t>(output_row_step);
  output.data.resize(static_cast<std::size_t>(output_data_size));
  output.is_dense = input.is_dense;

  for (std::uint32_t row = 0U; row < input.height; ++row) {
    for (std::uint32_t column = 0U; column < input.width; ++column) {
      const std::size_t input_offset =
        static_cast<std::size_t>(row) * input.row_step +
        static_cast<std::size_t>(column) * input.point_step;
      const std::size_t output_offset =
        static_cast<std::size_t>(row) * output.row_step +
        static_cast<std::size_t>(column) * output.point_step;

      std::memcpy(&output.data[output_offset], &input.data[input_offset + x_field.offset], 4U);
      std::memcpy(&output.data[output_offset + 4U], &input.data[input_offset + y_field.offset], 4U);
      std::memcpy(&output.data[output_offset + 8U], &input.data[input_offset + z_field.offset], 4U);

      float intensity = 0.0F;
      std::uint16_t channel = 0U;
      std::memcpy(&intensity, &input.data[input_offset + intensity_field.offset], sizeof(intensity));
      std::memcpy(&channel, &input.data[input_offset + ring_field.offset], sizeof(channel));
      output.data[output_offset + 12U] = normalize_intensity(intensity);
      output.data[output_offset + 13U] = kSingleStrongestReturn;
      std::memcpy(&output.data[output_offset + 14U], &channel, sizeof(channel));
    }
  }

  return output;
}

}  // namespace autoracer_rc_adapter
