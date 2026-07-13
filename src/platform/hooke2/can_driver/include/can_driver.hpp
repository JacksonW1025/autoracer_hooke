#pragma once

#include <iostream>
#include <string>
#include <thread>
#include <atomic>

// #include <rclcpp/rclcpp.hpp>
// #include "can_msgs/msg/frame.hpp"

// #include "can_client.hpp"
#include "innodisk/innodisk_can_client.hpp"
#include "socket/socket_can_client.hpp"

#ifdef KVASER_CAN
#include "kvaser/kvaser_can_client.hpp"
#elif defined(ESD_CAN)
#include "esd/esd_can_client.hpp"
#endif

#define MAX_CAN_RECV_FRAME_LEN 10

namespace can_driver
{

class CanDriver : public rclcpp::Node
{
public:
  explicit CanDriver(const rclcpp::NodeOptions & node_options);
  ~CanDriver();

private:
  static void can_rx_thread_callback(CanDriver * can_driver);

private:
  uint32_t brand_;
  uint32_t type_;
  uint32_t channel_id_;
  uint32_t interface_;
  uint32_t baudrate_;
  uint32_t dev_id_;
  std::string frame_id_;

  std::string can_rx_topic_;
  std::string can_tx_topic_;

  std::unique_ptr<CanClient> can_client_;

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

  std::thread can_rx_thread_;
  std::atomic<bool> is_running_;
};
}  // namespace can_driver