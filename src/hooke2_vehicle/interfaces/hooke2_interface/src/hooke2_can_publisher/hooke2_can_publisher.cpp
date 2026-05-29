#include <hooke2_can_publisher/hooke2_can_publisher.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>

namespace hooke2::hooke2_can_publisher
{
Hooke2CanPublisher::Hooke2CanPublisher(hooke2::Hooke2Interface* hooke2_interface)
: Node("hooke2_can_publisher"),
  hooke2_interface_(hooke2_interface)
{
  //  向can_driver发送数据，lwy20230811
  can_pub_ = create_publisher<can_msgs::msg::Frame>("/can_rx_from_autoware", rclcpp::QoS{10});
  //  定时发布can报文topic，lwy20230811 execut every 20 ms
  can_pub_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(50).period(), std::bind(&Hooke2CanPublisher::publishCan, this));
}

void Hooke2CanPublisher::publishCan() {
  const bool disable_auto_mode = hooke2_interface_->getDiasbleAutoMode();
  const uint8_t driving_interface = hooke2_interface_->getVehicleDrivingInterface();

  hooke2_msgs::msg::ThrottleCmd100 throttle_msg = hooke2_interface_->getThrottleCmd();
  hooke2_msgs::msg::BrakeCmd101 brake_msg = hooke2_interface_->getBrakeCmd();
  hooke2_msgs::msg::SteeringCmd102 steering_msg = hooke2_interface_->getSteeringCmd();
  hooke2_msgs::msg::GearCmd103 gear_msg = hooke2_interface_->getGearCmd();
  hooke2_msgs::msg::ParkCmd104 park_msg = hooke2_interface_->getParkCmd();
  hooke2_msgs::msg::VehicleModeCmd105 vehicle_mode_msg = hooke2_interface_->getVehicleModeCmd();
  
  //  send 0x100 can msg, Throttle
  {
    can_msgs::msg::Frame can_msg100;
    can_msg100.id = hooke2::common::Throttlecommand100::ID;
    can_msg100.dlc = 8;
    can_msg100.header.stamp = this->now();
    //  update can data 
    //  check if is in disable auto mode
    hooke2::common::Throttlecommand100 throttle_command_can;
    // if (engage_cmd_) {
    if (!disable_auto_mode) {
      throttle_command_can.set_throttle_en_ctrl(throttle_msg.throttle_en_ctrl);
      // throttle_command_can.set_throttle_en_ctrl(ThrottleCmd100::THROTTLE_EN_CTRL_ENABLE);
      switch (driving_interface)
      {
        case hooke2::Hooke2Interface::DrivingInterface::THROTTLE_PEDAL_MODE: {
          throttle_command_can.set_throttle_pedal_target(throttle_msg.throttle_pedal_target);
          throttle_command_can.set_throttle_acc(0.0);
          break;
        }
        case hooke2::Hooke2Interface::DrivingInterface::ACCELERATION_MODE: {
          throttle_command_can.set_throttle_acc(throttle_msg.throttle_acc);
          throttle_command_can.set_throttle_pedal_target(0.0);
          break;
        }
        case hooke2::Hooke2Interface::DrivingInterface::SPEED_MODE: {
          throttle_command_can.set_vel_target(throttle_msg.vel_target);
          // throttle_command_can.set_vel_target(2.0);
          break;
        }
        default:
          break;
      }
    } else {
      //  reset can msg data
      throttle_command_can.Reset();
    }
    throttle_command_can.UpdateData(&can_msg100.data[0]);  

    // RCLCPP_WARN(get_logger(), "Can msd throttle enable: %d, driving_interface: %d, vel_target: %f",
    // throttle_msg.throttle_en_ctrl, vehicle_driving_interface_, throttle_msg.vel_target);

    can_pub_->publish(can_msg100);
  }

  //  send 0x101 can msg, Brake
  {
    can_msgs::msg::Frame can_msg101;
    can_msg101.id = hooke2::common::Brakecommand101::ID;
    can_msg101.dlc = 8;
    can_msg101.header.stamp = this->now();
    //  update can data 
    //  check if is in disable auto mode
    hooke2::common::Brakecommand101 brake_command_can;
    // if (engage_cmd_) {
    if (!disable_auto_mode) {
      brake_command_can.set_aeb_en_ctrl(brake_msg.aeb_en_ctrl);
      // brake_command_can.set_brake_dec(brake_msg.brake_dec); //  未测试，暂时不用
      brake_command_can.set_brake_dec(0.0);
      brake_command_can.set_brake_pedal_target(brake_msg.brake_pedal_target);
      
      brake_command_can.set_brake_en_ctrl(brake_msg.brake_en_ctrl);

      //  for debug
      {
        // brake_command_can.set_aeb_en_ctrl(BrakeCmd101::AEB_EN_CTRL_DISABLE_AEB);
        // brake_command_can.set_brake_dec(0.0);
        // brake_command_can.set_brake_pedal_target(0.0);
        // brake_command_can.set_brake_en_ctrl(BrakeCmd101::BRAKE_EN_CTRL_ENABLE);
      }
      
    } else {
      // reset can msg data
      brake_command_can.Reset();
    }
    brake_command_can.UpdateData(&can_msg101.data[0]);
    
    // RCLCPP_WARN(get_logger(), "Can msd brake enable: %d, AEB_en_ctrl: %d, brake_pedal_target: %f",
    //   brake_msg.brake_en_ctrl, brake_msg.aeb_en_ctrl, brake_msg.brake_pedal_target);

    can_pub_->publish(can_msg101);
  }
  //  send 0x102 can msg, Steering
  {
    can_msgs::msg::Frame can_msg102;
    can_msg102.id = hooke2::common::Steeringcommand102::ID;
    can_msg102.dlc = 8;
    can_msg102.header.stamp = this->now();
    //  update can data 
    //  check if is in disable auto mode
    hooke2::common::Steeringcommand102 steering_command_can;
    // if (engage_cmd_) {
    if (!disable_auto_mode) {
      steering_command_can.set_steer_en_ctrl(steering_msg.steer_en_ctrl);      
      steering_command_can.set_steer_angle_target(steering_msg.steer_angle_target);
      steering_command_can.set_steer_angle_spd(steering_msg.steer_angle_spd);
      
      //  for debug
      {
        // steering_command_can.set_steer_en_ctrl(SteeringCmd102::STEER_EN_CTRL_ENABLE);      
        // steering_command_can.set_steer_angle_target(0.0);
        // steering_command_can.set_steer_angle_spd(220);
      }
      
    } else {
      //  reset can msg data
      steering_command_can.Reset();
    }
    steering_command_can.UpdateData(&can_msg102.data[0]);

    // RCLCPP_WARN(get_logger(), "Can msd steering enable: %d, steer_angle: %d, steer_spd: %d",
    //   steering_msg.steer_en_ctrl, steering_msg.steer_angle_target, steering_msg.steer_angle_spd);

    can_pub_->publish(can_msg102);
  }
  //  send 0x103 can msg, Gear
  {
    can_msgs::msg::Frame can_msg103;
    can_msg103.id = hooke2::common::Gearcommand103::ID;
    can_msg103.dlc = 8;
    can_msg103.header.stamp = this->now();
    //  update can data 
    //  check if is in disable auto mode
    hooke2::common::Gearcommand103 gear_command_can;
    // if (engage_cmd_) {
    if (!disable_auto_mode) {
      gear_command_can.set_gear_en_ctrl(gear_msg.gear_en_ctrl);
      gear_command_can.set_gear_target(gear_msg.gear_target);

      //  for debug
      {
        // gear_command_can.set_gear_en_ctrl(GearCmd103::GEAR_EN_CTRL_ENABLE);
        // gear_command_can.set_gear_target(GearCmd103::GEAR_TARGET_DRIVE);
      }
      
    } else {
      //  reset can msg data
      gear_command_can.Reset();
    }
    gear_command_can.UpdateData(&can_msg103.data[0]);

    // RCLCPP_WARN(get_logger(), "Can msd gear enable: %d, gear_target: %d",
    //   gear_msg.gear_en_ctrl, gear_msg.gear_target);
    
    can_pub_->publish(can_msg103);
  }
  //  send 0x104 can msg, Park
  {
    can_msgs::msg::Frame can_msg104;
    can_msg104.id = hooke2::common::Parkcommand104::ID;
    can_msg104.dlc = 8;
    can_msg104.header.stamp = this->now();
    //  update can data 
    //  check if is in disable auto mode
    hooke2::common::Parkcommand104 park_command_can;
    // if (engage_cmd_) {
    if (!disable_auto_mode) {
      park_command_can.set_park_target(park_msg.park_target);
      park_command_can.set_park_en_ctrl(park_msg.park_en_ctrl);
      
      //  for debug
      {
        // park_command_can.set_park_target(ParkCmd104::PARK_TARGET_RELEASE);
        // park_command_can.set_park_en_ctrl(ParkCmd104::PARK_EN_CTRL_ENABLE);
      }
      
    } else {
      //  reset can msg data
      park_command_can.Reset();
    }
    park_command_can.UpdateData(&can_msg104.data[0]);
    
    // RCLCPP_WARN(get_logger(), "Can msd park enable: %d, park_target: %d",
    //   park_msg.park_en_ctrl, park_msg.park_target);

    can_pub_->publish(can_msg104);
  }
  //  send 0x105 can msg
  {
    can_msgs::msg::Frame can_msg105;
    can_msg105.id = hooke2::common::Vehiclemodecommand105::ID;
    can_msg105.dlc = 8;
    can_msg105.header.stamp = this->now();
    //  update can data 
    //  check if is in disable auto mode
    hooke2::common::Vehiclemodecommand105 vehicle_mode_command_can;
    // if (engage_cmd_) {
    if (!disable_auto_mode) {
      vehicle_mode_command_can.set_turn_light_ctrl(vehicle_mode_msg.turn_light_ctrl);
      vehicle_mode_command_can.set_vin_req(vehicle_mode_msg.vin_req);
      vehicle_mode_command_can.set_drive_mode_ctrl(vehicle_mode_msg.drive_mode_ctrl);  //  底盘纵向驱动模式选择
      vehicle_mode_command_can.set_steer_mode_ctrl(vehicle_mode_msg.steer_mode_ctrl);  //  底盘横向转向模式选择

      //  for debug
      {
        // vehicle_mode_command_can.set_turn_light_ctrl(VehicleModeCmd105::TURN_LIGHT_CTRL_LEFT_TURNLAMP_ON);
        // vehicle_mode_command_can.set_vin_req(VehicleModeCmd105::VIN_REQ_VIN_REQ_ENABLE);
        // vehicle_mode_command_can.set_drive_mode_ctrl(VehicleModeCmd105::DRIVE_MODE_CTRL_SPEED_DRIVE);  //  底盘纵向驱动模式选择
        // vehicle_mode_command_can.set_steer_mode_ctrl(VehicleModeCmd105::STEER_MODE_CTRL_STANDARD_STEER);  //  底盘横向转向模式选择
      }
      
    } else {
      //  reset can msg data
      vehicle_mode_command_can.Reset();
    }
    vehicle_mode_command_can.UpdateData(&can_msg105.data[0]);

    // RCLCPP_WARN(get_logger(), "Can msd turn_light: %d, vin_req: %d, drive_mode: %d, steer_mode: %d",
    //   vehicle_mode_msg.turn_light_ctrl, vehicle_mode_msg.vin_req, 
    //   vehicle_mode_msg.drive_mode_ctrl, vehicle_mode_msg.steer_mode_ctrl);
    
    can_pub_->publish(can_msg105);
  }
}

} //  namespace hooke2_can_publisher