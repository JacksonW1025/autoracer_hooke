#ifndef AUTORACER_RC_ADAPTER__C32_POINTCLOUD_ADAPTER_HPP_
#define AUTORACER_RC_ADAPTER__C32_POINTCLOUD_ADAPTER_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace autoracer_rc_adapter
{

sensor_msgs::msg::PointCloud2 convert_c32_pointcloud(
  const sensor_msgs::msg::PointCloud2 & input);

}  // namespace autoracer_rc_adapter

#endif  // AUTORACER_RC_ADAPTER__C32_POINTCLOUD_ADAPTER_HPP_
