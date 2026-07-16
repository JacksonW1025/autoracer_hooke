#include "autoracer_rc_adapter/c32_pointcloud_adapter.hpp"

#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_field.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

PointField field(const std::string & name, std::uint32_t offset, std::uint8_t datatype)
{
  PointField result;
  result.name = name;
  result.offset = offset;
  result.datatype = datatype;
  result.count = 1U;
  return result;
}

template<typename T>
void write_value(PointCloud2 & cloud, std::size_t point, std::size_t offset, const T & value)
{
  std::memcpy(&cloud.data[point * cloud.point_step + offset], &value, sizeof(T));
}

template<typename T>
T read_value(const PointCloud2 & cloud, std::size_t point, std::size_t offset)
{
  T value{};
  std::memcpy(&value, &cloud.data[point * cloud.point_step + offset], sizeof(T));
  return value;
}

PointCloud2 make_input()
{
  PointCloud2 cloud;
  cloud.header.frame_id = "lidar_top";
  cloud.header.stamp.sec = 123;
  cloud.header.stamp.nanosec = 456U;
  cloud.height = 1U;
  cloud.width = 3U;
  cloud.fields = {
    field("x", 0U, PointField::FLOAT32),
    field("y", 4U, PointField::FLOAT32),
    field("z", 8U, PointField::FLOAT32),
    field("intensity", 16U, PointField::FLOAT32),
    field("ring", 20U, PointField::UINT16),
    field("time", 24U, PointField::FLOAT32),
  };
  cloud.is_bigendian = false;
  cloud.point_step = 32U;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.row_step);
  cloud.is_dense = true;

  for (std::size_t point = 0U; point < cloud.width; ++point) {
    write_value(cloud, point, 0U, static_cast<float>(point + 1U));
    write_value(cloud, point, 4U, -static_cast<float>(point + 1U));
    write_value(cloud, point, 8U, static_cast<float>(point) + 0.5F);
    write_value(cloud, point, 20U, static_cast<std::uint16_t>(point + 7U));
  }
  write_value(cloud, 0U, 16U, 12.6F);
  write_value(cloud, 1U, 16U, 999.0F);
  write_value(cloud, 2U, 16U, std::numeric_limits<float>::quiet_NaN());
  return cloud;
}

TEST(C32PointcloudAdapter, ProducesExactPointXyzircLayout)
{
  const auto input = make_input();
  const auto output = autoracer_rc_adapter::convert_c32_pointcloud(input);

  EXPECT_EQ(output.header, input.header);
  EXPECT_EQ(output.height, 1U);
  EXPECT_EQ(output.width, 3U);
  EXPECT_EQ(output.point_step, 16U);
  EXPECT_EQ(output.row_step, 48U);
  EXPECT_EQ(output.data.size(), 48U);
  EXPECT_FALSE(output.is_bigendian);
  EXPECT_TRUE(output.is_dense);

  ASSERT_EQ(output.fields.size(), 6U);
  const char * names[] = {"x", "y", "z", "intensity", "return_type", "channel"};
  const std::uint32_t offsets[] = {0U, 4U, 8U, 12U, 13U, 14U};
  const std::uint8_t datatypes[] = {
    PointField::FLOAT32, PointField::FLOAT32, PointField::FLOAT32,
    PointField::UINT8, PointField::UINT8, PointField::UINT16};
  for (std::size_t index = 0U; index < output.fields.size(); ++index) {
    EXPECT_EQ(output.fields[index].name, names[index]);
    EXPECT_EQ(output.fields[index].offset, offsets[index]);
    EXPECT_EQ(output.fields[index].datatype, datatypes[index]);
    EXPECT_EQ(output.fields[index].count, 1U);
  }
}

TEST(C32PointcloudAdapter, PreservesCoordinatesAndNormalizesC32Metadata)
{
  const auto output = autoracer_rc_adapter::convert_c32_pointcloud(make_input());

  EXPECT_FLOAT_EQ(read_value<float>(output, 0U, 0U), 1.0F);
  EXPECT_FLOAT_EQ(read_value<float>(output, 1U, 4U), -2.0F);
  EXPECT_FLOAT_EQ(read_value<float>(output, 2U, 8U), 2.5F);
  EXPECT_EQ(read_value<std::uint8_t>(output, 0U, 12U), 13U);
  EXPECT_EQ(read_value<std::uint8_t>(output, 1U, 12U), 255U);
  EXPECT_EQ(read_value<std::uint8_t>(output, 2U, 12U), 0U);
  for (std::size_t point = 0U; point < output.width; ++point) {
    EXPECT_EQ(read_value<std::uint8_t>(output, point, 13U), 1U);
    EXPECT_EQ(read_value<std::uint16_t>(output, point, 14U), point + 7U);
  }
}

TEST(C32PointcloudAdapter, RejectsIncompatibleInputInsteadOfGuessing)
{
  auto input = make_input();
  input.is_bigendian = true;
  EXPECT_THROW(
    autoracer_rc_adapter::convert_c32_pointcloud(input), std::invalid_argument);

  input = make_input();
  input.fields[3].datatype = PointField::UINT8;
  EXPECT_THROW(
    autoracer_rc_adapter::convert_c32_pointcloud(input), std::invalid_argument);

  input = make_input();
  input.data.resize(input.data.size() - 1U);
  EXPECT_THROW(
    autoracer_rc_adapter::convert_c32_pointcloud(input), std::invalid_argument);
}

}  // namespace
