// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HOOKE2_CAN_PUBLISHER__HOOKE2_CAN_PUBLISHER_HPP_
#define HOOKE2_CAN_PUBLISHER__HOOKE2_CAN_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <hooke2_msgs/msg/global_rpt.hpp>
#include <hooke2_msgs/msg/steering_cmd.hpp>
#include <hooke2_msgs/msg/system_cmd_float.hpp>
#include <hooke2_msgs/msg/system_cmd_int.hpp>
#include <hooke2_msgs/msg/system_rpt_float.hpp>
#include <hooke2_msgs/msg/system_rpt_int.hpp>
#include <hooke2_msgs/msg/wheel_speed_rpt.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <hooke2_interface/hooke2_interface.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace hooke2::hooke2_can_publisher
{

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;

class Hooke2CanPublisher : public rclcpp::Node
{
  public:
    Hooke2CanPublisher(hooke2::Hooke2Interface* hooke2_interface);

  private:
    using Hooke2FeedbacksSyncPolicy = message_filters::sync_policies::ApproximateTime<
      hooke2_msgs::msg::SystemRptFloat, hooke2_msgs::msg::WheelSpeedRpt,
      hooke2_msgs::msg::SystemRptFloat, hooke2_msgs::msg::SystemRptFloat,
      hooke2_msgs::msg::SystemRptInt, hooke2_msgs::msg::SystemRptInt, hooke2_msgs::msg::GlobalRpt>;

    /* subscribers */

    // From Hooke2
    std::unique_ptr<message_filters::Subscriber<hooke2_msgs::msg::SystemRptFloat>>
      steer_wheel_rpt_sub_;
    std::unique_ptr<message_filters::Subscriber<hooke2_msgs::msg::WheelSpeedRpt>>
      wheel_speed_rpt_sub_;
    std::unique_ptr<message_filters::Subscriber<hooke2_msgs::msg::SystemRptFloat>> accel_rpt_sub_;
    std::unique_ptr<message_filters::Subscriber<hooke2_msgs::msg::SystemRptFloat>> brake_rpt_sub_;
    std::unique_ptr<message_filters::Subscriber<hooke2_msgs::msg::SystemRptInt>> shift_rpt_sub_;
    std::unique_ptr<message_filters::Subscriber<hooke2_msgs::msg::SystemRptInt>> turn_rpt_sub_;
    std::unique_ptr<message_filters::Subscriber<hooke2_msgs::msg::GlobalRpt>> global_rpt_sub_;
    std::unique_ptr<message_filters::Synchronizer<Hooke2FeedbacksSyncPolicy>> hooke2_feedbacks_sync_;




    rclcpp::TimerBase::SharedPtr can_pub_timer_;  // lwy20230811
    //  can topic pub
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;

    uint8_t driving_interface_;
    hooke2::Hooke2Interface* hooke2_interface_;
    void publishCan();
    
};

}   //  namespace hooke2_can_publisher

#endif  // HOOKE2_CAN_PUBLISHER__HOOKE2_CAN_PUBLISHER_HPP_
