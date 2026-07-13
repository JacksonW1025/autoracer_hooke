#pragma once

#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"

#define CAN_ID_MASK 0x1FFFF800U  // can_filter mask
#define ESD_CAN_EXTENDED_FRAME 0
#define MAX_CAN_RECV_FRAME_LEN 10
#define MAX_CAN_SEND_FRAME_LEN 10

namespace can_driver {

enum CanCardBrand : uint32_t {
  SOCKET = 0,
  INNODISK = 1,
  ESD = 2,
  KVASER = 3
};

enum CanCardType : uint32_t {
  PCI_CARD = 0,
  USB_CARD = 1
};

enum CanChannelId : uint32_t {
  CHANNEL_ID_ZERO = 0,
  CHANNEL_ID_ONE = 1,
  CHANNEL_ID_TWO = 2,
  CHANNEL_ID_THREE = 3,
  CHANNEL_ID_FOUR = 4,
  CHANNEL_ID_FIVE = 5,
  CHANNEL_ID_SIX = 6,
  CHANNEL_ID_SEVEN = 7
};

enum CanBitrate : uint32_t {
  BITRATE_10K = 10000,
  BITRATE_20K = 20000,
  BITRATE_50K = 50000,
  BITRATE_100K = 100000,
  BITRATE_125K = 125000,
  BITRATE_250K = 250000,
  BITRATE_500K = 500000,
  BITRATE_800K = 800000,
  BITRATE_1M = 1000000
};

enum CanDevId : uint32_t {
  ttyACM0 = 24,
  ttyACM1 = 25,
  ttyACM2 = 26
};

enum CanInterface : uint32_t {
  NATIVE = 0,
  VIRTUAL = 1,
  SLCAN = 2
};

class CanClient
{
public:
  CanClient(rclcpp::Node* node,
            uint32_t brand, 
            uint32_t type, 
            uint32_t channel_id, 
            uint32_t interface = static_cast<uint32_t>(CanInterface::NATIVE), 
            uint32_t bitrate = static_cast<uint32_t>(CanBitrate::BITRATE_500K), 
            uint32_t dev_id = static_cast<uint32_t>(CanDevId::ttyACM0)) 
    : node_(node),
      brand_(static_cast<CanCardBrand>(brand)),
      type_(static_cast<CanCardType>(type)),
      channel_id_(static_cast<CanChannelId>(channel_id)),
      interface_(static_cast<CanInterface>(interface)),
      baudrate_(static_cast<CanBitrate>(bitrate)),
      dev_id_(static_cast<CanDevId>(dev_id)),
      is_started_(false){};
      
  virtual ~CanClient() = default;
  virtual bool init() = 0;
  virtual void stop() = 0;
  virtual bool send(const std::vector<can_msgs::msg::Frame>& frames, int32_t const frame_num) = 0;
  virtual bool recv(std::vector<can_msgs::msg::Frame>& frames, int32_t const frame_num) = 0;

protected:
  rclcpp::Node* node_;

  CanCardBrand brand_;
  CanCardType type_;
  CanChannelId channel_id_;
  CanInterface interface_;
  CanBitrate baudrate_;
  CanDevId dev_id_;

  bool is_started_;
};

}  // namespace can_driver