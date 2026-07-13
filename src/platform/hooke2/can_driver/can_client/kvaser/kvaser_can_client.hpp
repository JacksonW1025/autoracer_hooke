#pragma once

#include "can_client.hpp"
#include "third_lib/kvaser_can/include/canlib.h"
#include "third_lib/kvaser_can/include/canstat.h"
#include "third_lib/kvaser_can/include/obsolete.h"

namespace can_driver {
class KvaserCanClient : public CanClient {
 public:
  KvaserCanClient(rclcpp::Node *node, uint32_t brand, uint32_t type,
                  uint32_t channel_id, uint32_t interface, uint32_t bitrate);
  ~KvaserCanClient();
  bool canSetBitrate(uint32_t baudrate);

  bool init() override;
  void stop() override;
  bool send(const std::vector<can_msgs::msg::Frame> &frames,
            int32_t const frame_num) override;
  bool recv(std::vector<can_msgs::msg::Frame> &frames,
            int32_t const frame_num) override;

 private:
  CanHandle kvaser_hnd_;
  char send_buf_[8];
  char recv_buf_[8];
};

KvaserCanClient::KvaserCanClient(rclcpp::Node *node, 
                                 uint32_t brand,
                                 uint32_t type, 
                                 uint32_t channel_id,
                                 uint32_t interface, 
                                 uint32_t bitrate)
    : CanClient(node, brand, type, channel_id, interface, bitrate),
      kvaser_hnd_(-1) {}

KvaserCanClient::~KvaserCanClient() { stop(); }

void KvaserCanClient::stop() {
  if (this->is_started_) {
    this->is_started_ = false;
    auto ret = canClose(this->kvaser_hnd_);
    if (ret != canOK) {
      RCLCPP_ERROR(this->node_->get_logger(),
                   "Failed to close kvaser can device");
    }
    RCLCPP_INFO(this->node_->get_logger(), "Kvaser can device closed");
  }
}

/**
 * determine the CAN bus parameters of kvaser leaf 2
 * Convert bitrate in number to canlib recongised parameters
 * */
inline canStatus calcBusParams(long &bitrate, unsigned int &tseg1,
                               unsigned int &tseg2, unsigned int &sjw) {
  if (bitrate > 0) {
    switch (bitrate) {
      case 1000000:
        bitrate = canBITRATE_1M;
        break;
      case 500000:
        bitrate = canBITRATE_500K;
        break;
      case 250000:
        bitrate = canBITRATE_250K;
        break;
      case 125000:
        bitrate = canBITRATE_125K;
        break;
      case 100000:
        bitrate = canBITRATE_100K;
        break;
      case 83333:
        bitrate = canBITRATE_83K;
        break;
      case 62500:
        bitrate = canBITRATE_62K;
        break;
      case 50000:
        bitrate = canBITRATE_50K;
        break;
      default:
        bitrate = -100;
    }
  }
  unsigned int nosamp;
  unsigned int syncmode;
  return canTranslateBaud(&bitrate, &tseg1, &tseg2, &sjw, &nosamp, &syncmode);
}

bool KvaserCanClient::canSetBitrate(uint32_t baudrate) {
  unsigned int tseg1;
  unsigned int tseg2;
  unsigned int sjw;
  long bitrate = baudrate;
  calcBusParams(bitrate, tseg1, tseg2, sjw);
  // canStatus status = canSetBusParams(this->kvaser_hnd_, canBITRATE_500K, 0, 0, 0, 0, 0);
  canStatus status = canSetBusParams(this->kvaser_hnd_, bitrate, tseg1, tseg2, sjw, 1, 0);
  if (status != canOK) {
    RCLCPP_ERROR(this->node_->get_logger(), "Failed to set bitrate");
    return false;
  }
  return true;
}

bool KvaserCanClient::init() {
  if (this->is_started_) {
    RCLCPP_WARN(this->node_->get_logger(), "KvaserCanClient already started");
    return true;
  }

  canInitializeLibrary();

  kvaser_hnd_ = canOpenChannel(this->channel_id_, canOPEN_EXCLUSIVE | canOPEN_REQUIRE_EXTENDED | canOPEN_ACCEPT_VIRTUAL);
  // kvaser_hnd_ = canOpenChannel(this->channel_id_, canOPEN_REQUIRE_EXTENDED);
  if (kvaser_hnd_ < 0) {
    RCLCPP_ERROR(this->node_->get_logger(), "Failed to open channel");
    return false;
  }

  if (!canSetBitrate(this->baudrate_)) {
    RCLCPP_ERROR(this->node_->get_logger(), "Failed to set bitrate");
    return false;
  }

  if (canBusOn(kvaser_hnd_) != canOK) {
    RCLCPP_ERROR(this->node_->get_logger(), "Failed to turn on bus");
    return false;
  }

  this->is_started_ = true;
  return true;
}

bool KvaserCanClient::send(const std::vector<can_msgs::msg::Frame> &frames,
                           int32_t const frame_num) {
  if (!this->is_started_) {
    RCLCPP_ERROR(this->node_->get_logger(), "KvaserCanClient not started");
    return false;
  }

  if (frame_num > MAX_CAN_SEND_FRAME_LEN || frame_num < 0) {
    RCLCPP_ERROR(this->node_->get_logger(), "Invalid frame number");
    return false;
  }
  for (int32_t i = 0; i < frame_num; i++) {
    long id = static_cast<long>(frames[i].id);
    std::memcpy(send_buf_, frames[i].data.data(), frames[i].dlc);
    auto status = canWrite(kvaser_hnd_, id, static_cast<void *>(send_buf_), frames[i].dlc,
                      canMSG_STD | canMSG_EXT);
    if (status != canOK) {
      RCLCPP_ERROR(this->node_->get_logger(), "Failed to send frame");
      return false;
    }
  }
  return true;
}

bool KvaserCanClient::recv(std::vector<can_msgs::msg::Frame> &frames,
                           int32_t const frame_num) {
  if (!this->is_started_) {
    RCLCPP_ERROR(this->node_->get_logger(), "KvaserCanClient not started");
    return false;
  }

  if (frame_num > MAX_CAN_RECV_FRAME_LEN || frame_num < 0) {
    RCLCPP_ERROR(this->node_->get_logger(), "Invalid frame number");
    return false;
  }

  long id;
  unsigned int len;
  unsigned long time = 1000;
  can_msgs::msg::Frame frame;
  for (int32_t i = 0; i < frame_num; i++) {
    memset(recv_buf_, 0, sizeof(recv_buf_));
    // auto status = canRead(kvaser_hnd_, &id, static_cast<void *>(recv_buf_), &len, nullptr, &time);
    auto status = canReadWait(kvaser_hnd_, &id, static_cast<void *>(recv_buf_),
                         &len, nullptr, nullptr, time);
    if (status != canOK) {
      RCLCPP_ERROR(this->node_->get_logger(), "Failed to receive frame");
      return false;
    }
    
    for (unsigned int i = 0; i < len; i++) {
      frame.data[i] = recv_buf_[i];
    }
    frame.id = id;
    frame.dlc = len;
    frame.header.stamp = this->node_->get_clock()->now();
    frames.push_back(frame);
  }

  return true;
}

}  // namespace can_driver