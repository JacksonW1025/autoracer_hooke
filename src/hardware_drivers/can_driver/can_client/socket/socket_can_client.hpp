#pragma once

#include "can_client.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <iostream>
#include <string>
#include <vector>

namespace can_driver
{
class SocketCanClient : public CanClient
{
public:
  SocketCanClient(
    rclcpp::Node * node, uint32_t brand, uint32_t type, uint32_t channel_id, uint32_t interface,
    uint32_t baudrate);
  ~SocketCanClient();
  bool init() override;
  void stop() override;
  bool send(const std::vector<can_msgs::msg::Frame> & frames, int32_t const frame_num) override;
  bool recv(std::vector<can_msgs::msg::Frame> & frames, int32_t const frame_num) override;

private:
  int dev_handler_;

  struct can_frame send_frames_[MAX_CAN_SEND_FRAME_LEN];
  struct can_frame recv_frames_[MAX_CAN_RECV_FRAME_LEN];
};

SocketCanClient::SocketCanClient(
  rclcpp::Node * node, uint32_t brand, uint32_t type, uint32_t channel_id, uint32_t interface,
  uint32_t baudrate)
: CanClient(node, brand, type, channel_id, interface, baudrate), dev_handler_(-1)
{
  memset(send_frames_, 0, sizeof(send_frames_));
  memset(recv_frames_, 0, sizeof(recv_frames_));
}

SocketCanClient::~SocketCanClient()
{
  stop();
}

bool SocketCanClient::init()
{
  if (this->is_started_) {
    RCLCPP_WARN(this->node_->get_logger(), "CanClient is already started");
    return true;
  }

  dev_handler_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (dev_handler_ < 0) {
    RCLCPP_ERROR(this->node_->get_logger(), "Error while opening socket");
    return false;
  }

  if (this->interface_ != CanInterface::VIRTUAL) {
    // set a scope for each EID instead of a single filter rule for each EID
    struct can_filter filter;
    filter.can_id = 0x000;
    filter.can_mask = CAN_ID_MASK;
    if (setsockopt(dev_handler_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
      RCLCPP_ERROR(this->node_->get_logger(), "Error while setting socket options");
      return false;
    }
  }

  int enable = 1;
  if (setsockopt(dev_handler_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) < 0) {
    RCLCPP_ERROR(this->node_->get_logger(), "Error while setting socket options");
    return false;
  }

  struct ifreq ifr;
  std::string interface_prefix;
  if (this->interface_ == CanInterface::VIRTUAL) {
    interface_prefix = "vcan";
  } else if (this->interface_ == CanInterface::SLCAN) {
    interface_prefix = "slcan";
  } else {
    interface_prefix = "can";
  }
  interface_prefix += std::to_string(this->channel_id_);
  strcpy(ifr.ifr_name, interface_prefix.c_str());
  if (ioctl(dev_handler_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(this->node_->get_logger(), "Error in ioctl");
    return false;
  }

  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(dev_handler_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    RCLCPP_ERROR(this->node_->get_logger(), "Error in socket bind");
    return false;
  }
  this->is_started_ = true;
  return true;
}

void SocketCanClient::stop()
{
  if (this->is_started_) {
    this->is_started_ = false;
    if (close(dev_handler_)) {
      RCLCPP_ERROR(this->node_->get_logger(), "Error in socket close");
    } else {
      dev_handler_ = -1;
      RCLCPP_INFO(this->node_->get_logger(), "Socket closed");
    }
  }
}

bool SocketCanClient::send(
  const std::vector<can_msgs::msg::Frame> & frames, int32_t const frame_num)
{
  if (!this->is_started_) {
    RCLCPP_ERROR(this->node_->get_logger(), "can client has not started!");
    return false;
  }

  if (frames.size() == 0 || static_cast<int32_t>(frames.size()) != frame_num) {
    RCLCPP_ERROR(this->node_->get_logger(), "frames size is not equal to frame_num");
    return false;
  }

  if (frame_num > MAX_CAN_SEND_FRAME_LEN) {
    RCLCPP_ERROR(this->node_->get_logger(), "frame_num is larger than MAX_CAN_SEND_FRAME_LEN");
    return false;
  }

  for (auto i = 0; i < frame_num; i++) {
    send_frames_[i].can_id = frames[i].id;
    send_frames_[i].can_dlc = frames[i].dlc;
    for (auto j = 0; j < frames[i].dlc; j++) {
      send_frames_[i].data[j] = frames[i].data[j];
    }
    if (write(dev_handler_, &send_frames_[i], sizeof(send_frames_[i])) < 0) {
      // RCLCPP_ERROR(this->node_->get_logger(), "Error in socket write");
      return false;
    }
  }
  return true;
}

bool SocketCanClient::recv(std::vector<can_msgs::msg::Frame> & frames, int32_t const frame_num)
{
  if (!this->is_started_) {
    RCLCPP_ERROR(this->node_->get_logger(), "can client has not started!");
    return false;
  }

  if (frame_num <= 0) {
    RCLCPP_ERROR(this->node_->get_logger(), "frane_num is less than 0");
    return false;
  }

  if (frame_num > MAX_CAN_RECV_FRAME_LEN) {
    RCLCPP_ERROR(this->node_->get_logger(), "frame_num is larger than MAX_CAN_RECV_FRAME_LEN");
    return false;
  }
  
  for (auto i = 0; i < frame_num; i++) {
    if (read(dev_handler_, &recv_frames_[i], sizeof(recv_frames_[i])) < 0) {
      RCLCPP_ERROR(this->node_->get_logger(), "Error in socket read");
      return false;
    }
    can_msgs::msg::Frame frame_msg;
    frame_msg.header.stamp = this->node_->now();
    frame_msg.id = recv_frames_[i].can_id;
    frame_msg.dlc = static_cast<uint8_t>(recv_frames_[i].can_dlc);
    for (auto j = 0; j < recv_frames_[i].can_dlc; j++) {
      frame_msg.data[j] = recv_frames_[i].data[j];
    }
    frames.push_back(frame_msg);
  }
  return true;
}

}  // namespace can_driver