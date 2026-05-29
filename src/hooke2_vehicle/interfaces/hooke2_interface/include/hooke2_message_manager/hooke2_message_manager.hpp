#ifndef HOOKE2_MESSAGE_MANAGER__HOOKE2_MESSAGE_MANAGER_HPP_
#define HOOKE2_MESSAGE_MANAGER__HOOKE2_MESSAGE_MANAGER_HPP_

#pragma once

#include <hooke2_msgs/msg/throttle_cmd100.hpp>
#include <hooke2_msgs/msg/brake_cmd101.hpp>
#include <hooke2_msgs/msg/steering_cmd102.hpp>
#include <hooke2_msgs/msg/gear_cmd103.hpp>
#include <hooke2_msgs/msg/park_cmd104.hpp>
#include <hooke2_msgs/msg/vehicle_mode_cmd105.hpp>

#include "common/throttle_command_100.hpp"
#include "common/brake_command_101.hpp"
#include "common/steering_command_102.hpp"
#include "common/gear_command_103.hpp"
#include "common/park_command_104.hpp"
#include "common/vehicle_mode_command_105.hpp"

#include "common/message_manager.hpp"
#include <can_msgs/msg/frame.hpp>

namespace hooke2 {

using hooke2::common::MessageManager;

class Hooke2MessageManager : public MessageManager<can_msgs::msg::Frame> {
 public:
  Hooke2MessageManager();
  virtual ~Hooke2MessageManager();
};

}  // namespace hooke2

#endif  // HOOKE2_MESSAGE_MANAGER__HOOKE2_MESSAGE_MANAGER_HPP_