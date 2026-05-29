#pragma once

#include "can_client.hpp"
#include "third_lib/innodisk_can/include/lib_emuc_2.h"

#define SUCCESS 0
#define FAILURE 1
#define PORT_NUM 2

namespace can_driver
{

class InnodiskCanClient : public CanClient
{
public:
  InnodiskCanClient(
    rclcpp::Node * node, uint32_t brand, uint32_t type, uint32_t channel_id, uint32_t interface,
    uint32_t bitrate, uint32_t dev_id);
  ~InnodiskCanClient();
  bool init() override;
  void stop() override;
  bool send(const std::vector<can_msgs::msg::Frame> & frames, int32_t const frame_num) override;
  bool recv(std::vector<can_msgs::msg::Frame> & frames, int32_t const frame_num) override;

private:
  CAN_FRAME_INFO send_frame_;
  CAN_FRAME_INFO recv_frame_;
};

InnodiskCanClient::InnodiskCanClient(
  rclcpp::Node * node, uint32_t brand, uint32_t type, uint32_t channel_id, uint32_t interface,
  uint32_t bitrate, uint32_t dev_id)
: CanClient(node, brand, type, channel_id, interface, bitrate, dev_id)
{
  memset(&send_frame_, 0, sizeof(send_frame_));
  memset(&recv_frame_, 0, sizeof(recv_frame_));
}

InnodiskCanClient::~InnodiskCanClient()
{
  stop();
}

void InnodiskCanClient::stop()
{
  if (this->is_started_) {
    EMUCCloseDevice(this->dev_id_);
    this->is_started_ = false;
  }
}

bool InnodiskCanClient::init()
{
  if (this->is_started_) {
    RCLCPP_WARN(this->node_->get_logger(), "CanClient is already started");
    return true;
  }

  int ret;
  ret = EMUCOpenSocketCAN(this->dev_id_);
  if (ret != SUCCESS) {
    RCLCPP_ERROR(this->node_->get_logger(), "Error while opening socket");
    return false;
  }

  ret = EMUCInitCAN(this->dev_id_, EMUC_INACTIVE, EMUC_INACTIVE);
  if (ret != SUCCESS) {
    RCLCPP_ERROR(this->node_->get_logger(), "Error while INACTIVE CAN");
    return false;
  }

  // 3. show can version
  VER_INFO ver_info;
  ret = EMUCShowVer(this->dev_id_, &ver_info);
  if (ret != SUCCESS) {
    RCLCPP_ERROR(this->node_->get_logger(), "EMUC get version error code: ");
  } else {
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "EMUC show version successfully !");
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "FW ver:" << ver_info.fw);
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "LIB ver:" << ver_info.api);
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "model:" << ver_info.model);
  }

  ret = EMUCSetBaudRate(this->dev_id_, this->baudrate_, this->baudrate_);
  if (ret != SUCCESS) {
    RCLCPP_ERROR(this->node_->get_logger(), "Error while setting bitrate");
    return false;
  } else {
    RCLCPP_INFO_STREAM(
      this->node_->get_logger(), "EMUC set bitrate successfully ! bitrate: " << this->baudrate_);
  }

  CFG_INFO cfg_info;
  ret = EMUCGetCfg(this->dev_id_, &cfg_info);
  if (ret != SUCCESS) {
    RCLCPP_ERROR(this->node_->get_logger(), "Error while getting config");
  } else {
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "EMUC get config successfully !");
    for (size_t i = 0; i < PORT_NUM; i++) {
      RCLCPP_INFO_STREAM(this->node_->get_logger(), "----------------------------------------");
      RCLCPP_INFO_STREAM(this->node_->get_logger(), "can port: " << i);
      RCLCPP_INFO_STREAM(
        this->node_->get_logger(), "bitrate: " << static_cast<int>(cfg_info.baud[i]));
      RCLCPP_INFO_STREAM(this->node_->get_logger(), "mode: " << static_cast<int>(cfg_info.mode[i]));
      RCLCPP_INFO_STREAM(
        this->node_->get_logger(), "filter type: " << static_cast<int>(cfg_info.flt_type[i]));
      RCLCPP_INFO_STREAM(this->node_->get_logger(), "filter id: " << cfg_info.flt_id[i]);
      RCLCPP_INFO_STREAM(this->node_->get_logger(), "filter mask: " << cfg_info.flt_mask[i]);
    }
    RCLCPP_INFO_STREAM(
      this->node_->get_logger(), "error set:" << static_cast<int>(cfg_info.err_set));
  }

  ret = EMUCInitCAN(this->dev_id_, EMUC_ACTIVE, EMUC_ACTIVE);
  if (ret != SUCCESS) {
    RCLCPP_ERROR(this->node_->get_logger(), "Error while ACTIVE CAN");
    return false;
  }

  this->is_started_ = true;
  return true;
}

bool InnodiskCanClient::send(
  const std::vector<can_msgs::msg::Frame> & frames, int32_t const frame_num)
{
  if (!this->is_started_) {
    RCLCPP_ERROR(this->node_->get_logger(), "CanClient is not started");
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

  for (auto i = 0; i < frame_num; ++i) {
    send_frame_.id = frames[i].id;
    send_frame_.dlc = frames[i].dlc;
    send_frame_.CAN_port = this->channel_id_;
    send_frame_.rtr = frames[i].is_rtr;
    send_frame_.id_type = frames[i].is_extended;
    for (auto j = 0; j < frames[i].dlc; ++j) {
      send_frame_.data[j] = frames[i].data[j];
    }
    auto ret = EMUCSend(this->dev_id_, &send_frame_);
    if (ret != SUCCESS) {
      RCLCPP_ERROR(this->node_->get_logger(), "Error while sending frames");
      return false;
    }
  }
  
  return true;
}

bool InnodiskCanClient::recv(std::vector<can_msgs::msg::Frame> & frames, int32_t const frame_num)
{
  if (!this->is_started_) {
    RCLCPP_ERROR(this->node_->get_logger(), "CanClient is not started");
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

  uint32_t recv_time = 10000;
  while (true) {
    if (--recv_time == 0) {
      RCLCPP_ERROR_STREAM(this->node_->get_logger(), "Error recv timeout");
      return false;
    }
    auto num = EMUCReceive(this->dev_id_, &recv_frame_);
    if (num != 1) {
      usleep(10);
      continue;
    }
    if (recv_frame_.msg_type != EMUC_DATA_TYPE) {
      continue;
    }
    can_msgs::msg::Frame frame;
    frame.header.stamp = this->node_->now();
    frame.id = recv_frame_.id;
    frame.dlc = recv_frame_.dlc;
    frame.is_rtr = recv_frame_.rtr;
    frame.is_extended = recv_frame_.id_type;
    for (auto i = 0; i < frame.dlc; ++i) {
      frame.data[i] = recv_frame_.data[i];
    }
    frames.push_back(frame);
    if (static_cast<int32_t>(frames.size()) == frame_num) {
      break;
    }
  }
  return true;
}

}  // namespace can_driver