#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuQosAdapterNode final : public rclcpp::Node
{
public:
  ImuQosAdapterNode()
  : Node("imu_qos_adapter")
  {
    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    output_qos.reliable().durability_volatile();
    publisher_ = create_publisher<sensor_msgs::msg::Imu>("output", output_qos);

    auto input_qos = rclcpp::SensorDataQoS();
    input_qos.best_effort().durability_volatile();
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "input", input_qos,
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr message) {
        publisher_->publish(*message);
      });

    RCLCPP_INFO(
      get_logger(),
      "adapting native IMU messages from BEST_EFFORT/VOLATILE to "
      "RELIABLE/VOLATILE without changing message contents");
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuQosAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
