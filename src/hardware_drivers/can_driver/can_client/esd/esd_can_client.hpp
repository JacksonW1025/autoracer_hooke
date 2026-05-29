#pragma once

#include "can_client.hpp"
#include "third_lib/esd_can/include/ntcan.h"

namespace can_driver
{
class EsdCanClient : public CanClient
{
public:
  EsdCanClient(
    rclcpp::Node * node, uint32_t brand, uint32_t type, uint32_t channel_id, uint32_t interface,
    uint32_t bitrate);
  ~EsdCanClient();
  bool init() override;
  void stop() override;
  bool send(const std::vector<can_msgs::msg::Frame> & frames, int32_t const frame_num) override;
  bool recv(std::vector<can_msgs::msg::Frame> & frames, int32_t const frame_num) override;
private:
  NTCAN_HANDLE dev_handler_;

  CMSG send_msgs_[MAX_CAN_SEND_FRAME_LEN];
  CMSG recv_msgs_[MAX_CAN_RECV_FRAME_LEN];
};

EsdCanClient::EsdCanClient(
  rclcpp::Node * node, uint32_t brand, uint32_t type, uint32_t channel_id, uint32_t interface,
  uint32_t bitrate)
: CanClient(node, brand, type, channel_id, interface, bitrate),
  dev_handler_(-1)
{
  memset(send_msgs_, 0, sizeof(send_msgs_));
  memset(recv_msgs_, 0, sizeof(recv_msgs_));
}

EsdCanClient::~EsdCanClient()
{
  stop();
}

void EsdCanClient::stop()
{
  if (this->is_started_) {
    this->is_started_ = false;
    int32_t ret = canClose(dev_handler_);
    if (ret != NTCAN_SUCCESS) {
      RCLCPP_ERROR(this->node_->get_logger(), "canClose failed, ret = %d", ret);
    }
    RCLCPP_INFO(this->node_->get_logger(), "canClose success");
  }
}
inline uint32_t baudRateSelect(uint32_t baudrate)
{
  switch (baudrate) {
    case static_cast<uint32_t>(CanBitrate::BITRATE_10K):
      return NTCAN_BAUD_10;
    case static_cast<uint32_t>(CanBitrate::BITRATE_20K):
      return NTCAN_BAUD_20;
    case static_cast<uint32_t>(CanBitrate::BITRATE_50K):
      return NTCAN_BAUD_50;
    case static_cast<uint32_t>(CanBitrate::BITRATE_100K):
      return NTCAN_BAUD_100;
    case static_cast<uint32_t>(CanBitrate::BITRATE_125K):
      return NTCAN_BAUD_125;
    case static_cast<uint32_t>(CanBitrate::BITRATE_250K):
      return NTCAN_BAUD_250;
    case static_cast<uint32_t>(CanBitrate::BITRATE_500K):
      return NTCAN_BAUD_500;
    case static_cast<uint32_t>(CanBitrate::BITRATE_800K):
      return NTCAN_BAUD_800;
    case static_cast<uint32_t>(CanBitrate::BITRATE_1M):
      return NTCAN_BAUD_1000;
    default:
      return NTCAN_BAUD_500;
  }
}

bool EsdCanClient::init()
{
  if (this->is_started_) {
    RCLCPP_WARN(this->node_->get_logger(), "can client has already started!");
    return true;
  }

  uint32_t mode = 0;
  if (ESD_CAN_EXTENDED_FRAME) {
    mode = NTCAN_MODE_NO_RTR;
  }

  int32_t ret = canOpen(this->channel_id_, mode, NTCAN_MAX_TX_QUEUESIZE, NTCAN_MAX_TX_QUEUESIZE, 5, 5, &dev_handler_);
  if (ret != NTCAN_SUCCESS) {
    RCLCPP_ERROR(this->node_->get_logger(), "CanOpen failed, ret = %d", ret);
    return false;
  } else {
    RCLCPP_INFO(this->node_->get_logger(), "CanOpen success");
  }

  if (ESD_CAN_EXTENDED_FRAME) {
    int32_t id_count = 0x1FFFFFFE;
    ret = canIdRegionAdd(dev_handler_, 0x20000000, &id_count);
  } else {
    int32_t id_count = 0x800;
    ret = canIdRegionAdd(dev_handler_, 0, &id_count);
  }
  if (ret != NTCAN_SUCCESS) {
    RCLCPP_ERROR(this->node_->get_logger(), "canIdRegionAdd failed, ret = %d", ret);
    return false;
  } else {
    RCLCPP_INFO(this->node_->get_logger(), "canIdRegionAdd success");
  }

  ret = canSetBaudrate(dev_handler_, baudRateSelect(this->baudrate_));
  if (ret != NTCAN_SUCCESS) {
    RCLCPP_ERROR(this->node_->get_logger(), "canSetBaudrate failed, ret = %d", ret);
    return false;
  } else {
    RCLCPP_INFO(this->node_->get_logger(), "canSetBaudrate success, %d", this->baudrate_);
  }

  this->is_started_ = true;
  return true;
}

bool EsdCanClient::send(const std::vector<can_msgs::msg::Frame> & frames, int32_t const frame_num)
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

  for (int32_t i = 0; i < frame_num; ++i) {
    send_msgs_[i].id = frames[i].id;
    send_msgs_[i].len = frames[i].dlc;
    // send_msgs_[i].msgtype = frames[i].is_error ? NTCAN_MSGTYPE_ERROR_FRAME : NTCAN_MSGTYPE_STANDARD;
    send_msgs_[i].data[0] = frames[i].data[0];
    send_msgs_[i].data[1] = frames[i].data[1];
    send_msgs_[i].data[2] = frames[i].data[2];
    send_msgs_[i].data[3] = frames[i].data[3];
    send_msgs_[i].data[4] = frames[i].data[4];
    send_msgs_[i].data[5] = frames[i].data[5];
    send_msgs_[i].data[6] = frames[i].data[6];
    send_msgs_[i].data[7] = frames[i].data[7];
  }

  int32_t ret = canWrite(dev_handler_, send_msgs_, const_cast<int*>(&frame_num), NULL);
  if (ret != NTCAN_SUCCESS) {
    RCLCPP_ERROR(this->node_->get_logger(), "canWrite failed, ret = %d", ret);
    return false;
  }

  return true;
}

bool EsdCanClient::recv(std::vector<can_msgs::msg::Frame> & frames, int32_t const frame_num)
{
  if (!this->is_started_) {
    RCLCPP_ERROR(this->node_->get_logger(), "can client has not started!");
    return false;
  }

  if (frame_num <= 0) {
    RCLCPP_ERROR(this->node_->get_logger(), "frame_num is less than 0");
    return false;
  }

  if (frame_num > MAX_CAN_RECV_FRAME_LEN) {
    RCLCPP_ERROR(this->node_->get_logger(), "frame_num is larger than MAX_CAN_RECV_FRAME_LEN");
    return false;
  }

  int32_t ret = canRead(dev_handler_, recv_msgs_, const_cast<int*>(&frame_num), NULL);
  if (ret != NTCAN_SUCCESS) {
    // RCLCPP_ERROR(this->node_->get_logger(), "canRead failed, ret = %d", ret);
    return false;
  }

  for (int32_t i = 0; i < frame_num; ++i) {
    can_msgs::msg::Frame frame;
    frame.header.stamp = this->node_->now();
    frame.id = recv_msgs_[i].id;
    frame.dlc = recv_msgs_[i].len;
    // frame.is_error = (recv_msgs_[i].msgtype == NTCAN_MSGTYPE_ERROR_FRAME);
    frame.data[0] = recv_msgs_[i].data[0];
    frame.data[1] = recv_msgs_[i].data[1];
    frame.data[2] = recv_msgs_[i].data[2];
    frame.data[3] = recv_msgs_[i].data[3];
    frame.data[4] = recv_msgs_[i].data[4];
    frame.data[5] = recv_msgs_[i].data[5];
    frame.data[6] = recv_msgs_[i].data[6];
    frame.data[7] = recv_msgs_[i].data[7];

    frames.push_back(frame);
  }

  return true;
}


}  // namespace can_driver