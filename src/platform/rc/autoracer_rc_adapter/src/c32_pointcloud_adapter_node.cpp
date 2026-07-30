#include "autoracer_rc_adapter/c32_pointcloud_adapter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <exception>
#include <memory>

namespace autoracer_rc_adapter
{

class C32PointcloudAdapterNode : public rclcpp::Node
{
public:
  C32PointcloudAdapterNode()
  : Node("c32_pointcloud_adapter")
  {
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "output", rclcpp::SensorDataQoS());
    auto raw_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    // The C32 publishes large raw frames reliably. Preserve every frame before normalization.
    raw_qos.reliable().durability_volatile();
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", raw_qos,
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr input) {
        try {
          publisher_->publish(convert_c32_pointcloud(*input));
        } catch (const std::exception & error) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Rejected C32 pointcloud: %s", error.what());
        }
      });
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace autoracer_rc_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autoracer_rc_adapter::C32PointcloudAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
