#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuQosAdapterNode final : public rclcpp::Node
{
public:
  ImuQosAdapterNode()
  : Node("imu_qos_adapter")
  {
    const auto fallback_stddev =
      declare_parameter<double>("fallback_angular_velocity_stddev_radps", 0.0);
    if (!std::isfinite(fallback_stddev) || fallback_stddev <= 0.0) {
      throw std::invalid_argument(
              "fallback_angular_velocity_stddev_radps must be finite and positive");
    }
    fallback_angular_velocity_variance_ = fallback_stddev * fallback_stddev;

    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    output_qos.reliable().durability_volatile();
    publisher_ = create_publisher<sensor_msgs::msg::Imu>("output", output_qos);

    auto input_qos = rclcpp::SensorDataQoS();
    input_qos.best_effort().durability_volatile();
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "input", input_qos,
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr message) {
        auto output = *message;
        normalize_angular_velocity_covariance(output.angular_velocity_covariance);
        publisher_->publish(output);
      });

    RCLCPP_INFO(
      get_logger(),
      "adapting native IMU messages from BEST_EFFORT/VOLATILE to "
      "RELIABLE/VOLATILE with angular-velocity covariance floor variance %.9g",
      fallback_angular_velocity_variance_);
  }

private:
  void normalize_angular_velocity_covariance(std::array<double, 9> & covariance)
  {
    const bool unavailable = covariance[0] == -1.0 || std::all_of(
      covariance.cbegin(), covariance.cend(), [](const double value) {return value == 0.0;});
    if (unavailable) {
      covariance.fill(0.0);
    }

    for (const std::size_t index : {std::size_t{0}, std::size_t{4}, std::size_t{8}}) {
      if (!std::isfinite(covariance[index]) || covariance[index] <= 0.0) {
        covariance[index] = fallback_angular_velocity_variance_;
      }
    }

    for (const double value : covariance) {
      if (!std::isfinite(value)) {
        throw std::runtime_error("IMU angular-velocity covariance contains non-finite off-diagonal");
      }
    }

    if (unavailable && !reported_covariance_fallback_) {
      RCLCPP_WARN(
        get_logger(),
        "native IMU angular-velocity covariance is unavailable; applying the configured RC "
        "covariance floor while preserving header, frame, timestamp and measurements");
      reported_covariance_fallback_ = true;
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  double fallback_angular_velocity_variance_{0.0};
  bool reported_covariance_fallback_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuQosAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
