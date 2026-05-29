#include "can_driver.hpp"

#include <chrono>

namespace can_driver
{

CanDriver::~CanDriver()
{
  is_running_.exchange(false);
  if (can_rx_thread_.joinable()) {
    can_rx_thread_.join();
  }
}

CanDriver::CanDriver(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("can_driver_node", node_options),
  brand_(declare_parameter("brand", 2)),
  type_(declare_parameter("type", 0)),
  channel_id_(declare_parameter("channel_id", 0)),
  interface_(declare_parameter("interface", 0)),
  baudrate_(declare_parameter("baudrate", 500000)),
  dev_id_(declare_parameter("dev_id", 0)),
  frame_id_(declare_parameter("frame_id", "can_driver")),
  can_rx_topic_(declare_parameter("can_rx_topic", "can_rx")),
  can_tx_topic_(declare_parameter("can_tx_topic", "can_tx")),
  is_running_(false)
{
  if (brand_ == CanCardBrand::SOCKET) {
    can_client_ =
      std::make_unique<SocketCanClient>(this, brand_, type_, channel_id_, interface_, baudrate_);
    RCLCPP_INFO(this->get_logger(), "Socket CAN card detected!");
#ifdef INNODISK_CAN
  } else if (brand_ == CanCardBrand::INNODISK) {
    can_client_ = std::make_unique<InnodiskCanClient>(
      this, brand_, type_, channel_id_, interface_, baudrate_, dev_id_);
    RCLCPP_INFO(this->get_logger(), "Innodisk CAN card detected!");
#elif defined(KVASER_CAN)
  } else if (brand_ == CanCardBrand::KVASER) {
      can_client_ = std::make_unique<KvaserCanClient>(
        this, brand_, type_, channel_id_, interface_, baudrate_);
    RCLCPP_INFO(this->get_logger(), "Kvaser CAN card detected!");
#elif defined(ESD_CAN)
  } else if (brand_ == CanCardBrand::ESD) {
    can_client_ = std::make_unique<EsdCanClient>(
      this, brand_, type_, channel_id_, interface_, baudrate_);
    RCLCPP_INFO(this->get_logger(), "ESD CAN card detected!");
#endif      
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown can card brand!");
    exit(EXIT_FAILURE);
  }

  if (!can_client_->init()) {
    RCLCPP_ERROR(this->get_logger(), "Can client init failed!");
    exit(EXIT_FAILURE);
  }
  
  can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
    can_rx_topic_, 10, [this](const can_msgs::msg::Frame & msg) {
      std::vector<can_msgs::msg::Frame> frames;
      frames.push_back(msg);
      can_client_->send(frames, frames.size());
    });

  can_pub_ = this->create_publisher<can_msgs::msg::Frame>(can_tx_topic_, rclcpp::QoS(rclcpp::KeepLast(30)));
  can_rx_thread_ = std::thread(&CanDriver::can_rx_thread_callback, this);
}

void CanDriver::can_rx_thread_callback(CanDriver * can_driver)
{
  // CanDriver * can_driver = static_cast<CanDriver *>(arg);
  auto default_period = 1000 * 10;
  std::vector<can_msgs::msg::Frame> frames;
  int32_t frame_num = MAX_CAN_RECV_FRAME_LEN;
  can_driver->is_running_.exchange(true);

  while (rclcpp::ok() && can_driver->is_running_.load()) {
    if (!can_driver->can_client_->recv(frames, frame_num)) {
      // RCLCPP_WARN_THROTTLE(can_driver->get_logger(), *can_driver->get_clock(), 1000, "Failed to receive CAN frames");
      std::this_thread::sleep_for(std::chrono::microseconds(default_period));
    }
    if (frames.size() != static_cast<size_t>(frame_num)) {
      // RCLCPP_WARN_THROTTLE(can_driver->get_logger(), *can_driver->get_clock(), 1000, "Received CAN frames size mismatch");
    }
    if (frames.size() == 0) {
      // RCLCPP_WARN_THROTTLE(can_driver->get_logger(), *can_driver->get_clock(), 1000, "Received empty CAN frames");
      std::this_thread::sleep_for(std::chrono::microseconds(default_period));
      continue;
    }
    for (auto & frame : frames) {
      if (frame.header.frame_id.empty()) {
        frame.header.frame_id = can_driver->frame_id_;
      }
      can_driver->can_pub_->publish(frame);
    }
    frames.clear();
    std::this_thread::yield();
  }
}

}  // namespace can_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(can_driver::CanDriver)