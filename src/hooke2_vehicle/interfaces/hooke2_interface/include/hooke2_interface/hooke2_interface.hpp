// Copyright 2017-2019 Autoware Foundation
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

#ifndef HOOKE2_INTERFACE__HOOKE2_INTERFACE_HPP_
#define HOOKE2_INTERFACE__HOOKE2_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>
// #include <vehicle_info_util/vehicle_info_util.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/engage.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
// #include "autoware_vehicle_msgs/msg/control_mode_command.hpp"
#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>
#include <hooke2_msgs/msg/global_rpt.hpp>
#include <hooke2_msgs/msg/steering_cmd.hpp>
#include <hooke2_msgs/msg/system_cmd_float.hpp>
#include <hooke2_msgs/msg/system_cmd_int.hpp>
#include <hooke2_msgs/msg/system_rpt_float.hpp>
#include <hooke2_msgs/msg/system_rpt_int.hpp>
#include <hooke2_msgs/msg/wheel_speed_rpt.hpp>
#include <hooke2_msgs/msg/vin_report516.hpp>
#include <hooke2_msgs/msg/vin_report515.hpp>
#include <hooke2_msgs/msg/vin_report514.hpp>
#include <hooke2_msgs/msg/bms_report512.hpp>
#include <hooke2_msgs/msg/ultr_sensor511.hpp>
#include <hooke2_msgs/msg/ultr_sensor510.hpp>
#include <hooke2_msgs/msg/ultr_sensor509.hpp>
#include <hooke2_msgs/msg/ultr_sensor508.hpp>
#include <hooke2_msgs/msg/ultr_sensor507.hpp>
#include <hooke2_msgs/msg/wheel_speed_report506.hpp>
#include <hooke2_msgs/msg/vcu_report505.hpp>
#include <hooke2_msgs/msg/park_report504.hpp>
#include <hooke2_msgs/msg/gear_report503.hpp>
#include <hooke2_msgs/msg/steering_report502.hpp>
#include <hooke2_msgs/msg/brake_report501.hpp>
#include <hooke2_msgs/msg/throttle_report500.hpp>
#include <hooke2_msgs/msg/throttle_cmd100.hpp>
#include <hooke2_msgs/msg/brake_cmd101.hpp>
#include <hooke2_msgs/msg/steering_cmd102.hpp>
#include <hooke2_msgs/msg/gear_cmd103.hpp>
#include <hooke2_msgs/msg/park_cmd104.hpp>
#include <hooke2_msgs/msg/vehicle_mode_cmd105.hpp>
#include <tier4_api_msgs/msg/door_status.hpp>
#include <tier4_external_api_msgs/srv/set_door.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <tier4_vehicle_msgs/msg/actuation_status_stamped.hpp>
#include <tier4_vehicle_msgs/msg/steering_wheel_status_stamped.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>
#include <tier4_vehicle_msgs/msg/battery_status.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "common/message_manager.hpp"
#include "hooke2_message_manager/hooke2_message_manager.hpp"

#include "common/param_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <cstdint>
#include <thread>


namespace hooke2 {

class Hooke2Interface : public rclcpp::Node
{
  public:
    using ActuationCommandStamped = tier4_vehicle_msgs::msg::ActuationCommandStamped;
    using ActuationStatusStamped = tier4_vehicle_msgs::msg::ActuationStatusStamped;
    using SteeringWheelStatusStamped = tier4_vehicle_msgs::msg::SteeringWheelStatusStamped;
    using ControlModeCommand = autoware_vehicle_msgs::srv::ControlModeCommand;
    
    Hooke2Interface();

    ~Hooke2Interface() 
    {
      is_running_.exchange(false);
      // std::cout << "Multithread joinable: " << can_response_thread_.joinable() << std::endl;
      if (can_response_thread_.joinable()) {
        can_response_thread_.join();
      }
    }


    uint8_t getVehicleDrivingInterface() const;
    bool getDiasbleAutoMode() const;
    hooke2_msgs::msg::ThrottleCmd100 getThrottleCmd() const;
    hooke2_msgs::msg::BrakeCmd101 getBrakeCmd() const;
    hooke2_msgs::msg::SteeringCmd102 getSteeringCmd() const;
    hooke2_msgs::msg::GearCmd103 getGearCmd() const;
    hooke2_msgs::msg::ParkCmd104 getParkCmd() const;
    hooke2_msgs::msg::VehicleModeCmd105 getVehicleModeCmd() const;

    enum DrivingInterface : uint8_t
    {
      THROTTLE_PEDAL_MODE = 0,
      ACCELERATION_MODE = 1,
      SPEED_MODE = 2
    };

  private:
    typedef message_filters::sync_policies::ApproximateTime<
      hooke2_msgs::msg::SystemRptFloat, hooke2_msgs::msg::WheelSpeedRpt,
      hooke2_msgs::msg::SystemRptFloat, hooke2_msgs::msg::SystemRptFloat,
      hooke2_msgs::msg::SystemRptInt, hooke2_msgs::msg::SystemRptInt> Hooke2FeedbacksSyncPolicy;

    /* subscribers */
    // From Autoware
    rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr
      control_cmd_sub_;
    rclcpp::Subscription<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_sub_;
    rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr
      turn_indicators_cmd_sub_;
    rclcpp::Subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr
      hazard_lights_cmd_sub_;
    rclcpp::Subscription<ActuationCommandStamped>::SharedPtr actuation_cmd_sub_;
    rclcpp::Subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr emergency_sub_;

    // From Hooke2
    rclcpp::Subscription<hooke2_msgs::msg::SystemRptInt>::SharedPtr rear_door_rpt_sub_;

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
    
    //  can sub
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
    //  can pub
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;

    /* publishers */
    // From Hooke2(Rx)
    rclcpp::Publisher<hooke2_msgs::msg::VinReport516>::SharedPtr vin_rpt516_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::VinReport515>::SharedPtr vin_rpt515_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::VinReport514>::SharedPtr vin_rpt514_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::BmsReport512>::SharedPtr bms_rpt512_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::UltrSensor511>::SharedPtr ultr_sensor511_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::UltrSensor510>::SharedPtr ultr_sensor510_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::UltrSensor509>::SharedPtr ultr_sensor509_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::UltrSensor508>::SharedPtr ultr_sensor508_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::UltrSensor507>::SharedPtr ultr_sensor507_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::WheelSpeedReport506>::SharedPtr wheelspeed_rpt506_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::VcuReport505>::SharedPtr vcu_rpt505_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::ParkReport504>::SharedPtr park_rpt504_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::GearReport503>::SharedPtr gear_rpt503_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::SteeringReport502>::SharedPtr steering_rpt502_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::BrakeReport501>::SharedPtr brake_rpt501_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::ThrottleReport500>::SharedPtr throttle_rpt500_pub_;

    rclcpp::Publisher<hooke2_msgs::msg::SystemRptFloat>::SharedPtr steer_wheel_rpt_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::WheelSpeedRpt>::SharedPtr wheel_speed_rpt_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::SystemRptFloat>::SharedPtr accel_rpt_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::SystemRptFloat>::SharedPtr brake_rpt_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::SystemRptInt>::SharedPtr shift_rpt_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::SystemRptInt>::SharedPtr turn_rpt_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::GlobalRpt>::SharedPtr global_rpt_pub_;

    // To Hooke2(Tx)
    rclcpp::Publisher<hooke2_msgs::msg::SystemCmdFloat>::SharedPtr throttle_cmd_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::SystemCmdFloat>::SharedPtr acceleration_cmd_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::SystemCmdFloat>::SharedPtr speed_cmd_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::SystemCmdFloat>::SharedPtr brake_cmd_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::SteeringCmd>::SharedPtr steer_cmd_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::SystemCmdInt>::SharedPtr shift_cmd_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::SystemCmdInt>::SharedPtr parking_cmd_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::SystemCmdInt>::SharedPtr turn_cmd_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::SystemCmdInt>::SharedPtr door_cmd_pub_;
    rclcpp::Publisher<hooke2_msgs::msg::SteeringCmd>::SharedPtr
      raw_steer_cmd_pub_;  // only for debug

    // To Autoware
    rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr
      control_mode_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr vehicle_twist_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr
      steering_status_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr
      turn_indicators_status_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsReport>::SharedPtr
      hazard_lights_status_pub_;
    rclcpp::Publisher<ActuationStatusStamped>::SharedPtr actuation_status_pub_;
    rclcpp::Publisher<SteeringWheelStatusStamped>::SharedPtr steering_wheel_status_pub_;
    rclcpp::Publisher<tier4_api_msgs::msg::DoorStatus>::SharedPtr door_status_pub_;
    rclcpp::Publisher<tier4_vehicle_msgs::msg::BatteryStatus>::SharedPtr battery_status_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::TimerBase::SharedPtr can_pub_timer_;  // lwy20230811

    /* ros param */
    std::string base_frame_id_;
    int command_timeout_ms_;  // vehicle_cmd timeout [ms]
    bool is_hooke2_rpt_received_ = false;
    bool is_hooke2_auto_enabled_ = false;
    bool is_clear_override_needed_ = false;
    bool prev_override_ = false;
    double loop_rate_;           // [Hz]
    double tire_radius_;         // [m]
    double wheel_base_;          // [m]
    double steering_offset_;     // [rad] def: measured = truth + offset
    double vgr_coef_a_;          // variable gear ratio coeffs
    double vgr_coef_b_;          // variable gear ratio coeffs
    double vgr_coef_c_;          // variable gear ratio coeffs
    double accel_pedal_offset_;  // offset of accel pedal value
    double brake_pedal_offset_;  // offset of brake pedal value

    double emergency_brake_;              // brake command when emergency [m/s^2]
    double max_speed_;                    // max speed, unit: m/s
    double max_throttle_;                 // max throttle [0~1]
    double max_brake_;                    // max brake [0~1]
    double max_steering_wheel_;           // max steering wheel angle [rad]
    double max_steering_wheel_rate_;      // [rad/s]
    double min_steering_wheel_rate_;      // [rad/s]
    double steering_wheel_rate_low_vel_;  // [rad/s]
    double steering_wheel_rate_stopped_;  // [rad/s]
    double low_vel_thresh_;               // [m/s]

    bool enable_steering_rate_control_;   // use steering angle speed for command [rad/s]
    bool need_separate_engage_sequence_;  // when you use a newer version of firmware than 3.3, it
                                          // must be true
    uint8_t vehicle_driving_interface_;   //  vehicle driving interface, 0 - throttle, 1 - acceleration, 2 - speed
    uint8_t vehicle_steering_interface_;   //  vehicle steering interface, 0 - standard front wheel steering, 1 - four wheel steering

    double hazard_thresh_time_;
    int hazard_recover_count_ = 0;
    const int hazard_recover_cmd_num_ = 5;
    //@whaledynamic
    double current_speed = 0.0;
    int parking_actual_ = 1;

    autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

    // Service
    tier4_api_utils::Service<tier4_external_api_msgs::srv::SetDoor>::SharedPtr srv_;
    rclcpp::Service<ControlModeCommand>::SharedPtr control_mode_server_;

    /* input values */
    ActuationCommandStamped::ConstSharedPtr actuation_cmd_ptr_;
    autoware_control_msgs::msg::Control::ConstSharedPtr control_cmd_ptr_;
    autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr turn_indicators_cmd_ptr_;
    autoware_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr hazard_lights_cmd_ptr_;
    autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr gear_cmd_ptr_;

    
    hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr steer_wheel_rpt_ptr_;  // [rad]
    hooke2_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt_ptr_;   // [m/s]
    hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt_ptr_;
    hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt_ptr_;   // [m/s]
    hooke2_msgs::msg::SystemRptInt::ConstSharedPtr gear_rpt_ptr_;  // [m/s]
    hooke2_msgs::msg::BmsReport512::ConstSharedPtr bms_rpt512_ptr_;  // 电池状态
    hooke2_msgs::msg::GlobalRpt global_rpt_ptr_;      // [m/s]
    hooke2_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt_ptr_;
    hooke2_msgs::msg::SteeringCmd prev_steer_cmd_;
    rclcpp::Time prev_steer_cmd_time_;

    bool engage_cmd_{false};
    bool is_emergency_{false};
    rclcpp::Time control_command_received_time_;
    rclcpp::Time actuation_command_received_time_;
    rclcpp::Time last_shift_inout_matched_time_;

    /* callbacks */
    void callbackActuationCmd(const ActuationCommandStamped::ConstSharedPtr msg);
    void callbackControlCmd(
      const autoware_control_msgs::msg::Control::ConstSharedPtr msg);

    void callbackEmergencyCmd(
      const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg);

    void callbackGearCmd(const autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg);
    void callbackTurnIndicatorsCommand(
      const autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg);
    void callbackHazardLightsCommand(
      const autoware_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg);
    void callbackEngage(const autoware_vehicle_msgs::msg::Engage::ConstSharedPtr msg);
    void callbackRearDoor(const hooke2_msgs::msg::SystemRptInt::ConstSharedPtr rear_door_rpt);
    void callbackHooke2Rpt(
      const hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr steer_wheel_rpt,
      const hooke2_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt,
      const hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt,
      const hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt,
      const hooke2_msgs::msg::SystemRptInt::ConstSharedPtr gear_cmd_rpt,
      const hooke2_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt);

    /*  functions */
    void publishCommands();
    void publishCan();  // lwy20230811
    double calculateVehicleVelocity(
      const hooke2_msgs::msg::WheelSpeedRpt & wheel_speed_rpt,
      const hooke2_msgs::msg::SystemRptInt & shift_rpt);
    double calculateVariableGearRatio(const double vel, const double steer_wheel);
    double calcSteerWheelRateCmd(const double gear_ratio);
    uint16_t toHooke2ShiftCmd(const autoware_vehicle_msgs::msg::GearCommand & gear_cmd);
    uint16_t toHooke2TurnCmd(
      const autoware_vehicle_msgs::msg::TurnIndicatorsCommand & turn,
      const autoware_vehicle_msgs::msg::HazardLightsCommand & hazard);
    uint16_t toHooke2TurnCmdWithHazardRecover(
      const autoware_vehicle_msgs::msg::TurnIndicatorsCommand & turn,
      const autoware_vehicle_msgs::msg::HazardLightsCommand & hazard);

    std::optional<int32_t> toAutowareShiftReport(const hooke2_msgs::msg::SystemRptInt & shift);
    int32_t toAutowareTurnIndicatorsReport(const hooke2_msgs::msg::SystemRptInt & turn);
    int32_t toAutowareHazardLightsReport(const hooke2_msgs::msg::SystemRptInt & turn);
    double steerWheelRateLimiter(
      const double current_steer_cmd, const double prev_steer_cmd,
      const rclcpp::Time & current_steer_time, const rclcpp::Time & prev_steer_time,
      const double steer_rate, const double current_steer_output, const bool engage);
    hooke2_msgs::msg::SystemCmdInt createClearOverrideDoorCommand();
    hooke2_msgs::msg::SystemCmdInt createDoorCommand(const bool open);
    void setDoor(
      const tier4_external_api_msgs::srv::SetDoor::Request::SharedPtr request,
      const tier4_external_api_msgs::srv::SetDoor::Response::SharedPtr response);
    tier4_api_msgs::msg::DoorStatus toAutowareDoorStatusMsg(
      const hooke2_msgs::msg::SystemRptInt & msg_ptr);
      
    void onControlModeRequest(
      const ControlModeCommand::Request::SharedPtr request,
      const ControlModeCommand::Response::SharedPtr response);
    
    enum MessageID : uint16_t
    {
      //  Command CAN ID
      THROTTLE_COMMAND = 0x100,
      BRAKE_COMMAND = 0x101,
      STEERING_COMMAND = 0x102,
      GEAR_COMMAND = 0x103,
      PARK_COMMAND = 0x104,
      VEHICLE_MODE_COMMAND = 0x105,

      //  Report CAN ID
      THROTTLE_REPORT = 0x500,
      BRAKE_REPORT = 0x501,
      STEERING_REPORT = 0x502,
      GEAR_REPORT = 0x503,
      PARK_REPORT = 0x504,
      VCU_REPORT = 0x505,
      WHEELSPEED_REPORT = 0x506,
      ULTR_SENSOR_1 = 0x507,
      ULTR_SENSOR_2 = 0x508,
      ULTR_SENSOR_3 = 0x509,
      ULTR_SENSOR_4 = 0x510,
      ULTR_SENSOR_5 = 0x511,
      BMS_REPORT = 0x512,
      VIN_RESP1 = 0x514,
      VIN_RESP2 = 0x515,
      VIN_RESP3 = 0x516
    };

    bool isTargetId(uint32_t id)
    {
      // bms_report:          0x512
      // brake_command:       0x101
      // brake_report:        0x501
      // gear_command:        0x103
      // gear_report:         0x503
      // park_command:        0x104
      // park_report:         0x504
      // steering_command:    0x102
      // steering_report:     0x502
      // throttle_command:    0x100
      // throttle_report:     0x500
      // ultr_sensor_1:       0x507
      // ultr_sensor_2:       0x508
      // ultr_sensor_3:       0x509
      // ultr_sensor_4:       0x510
      // ultr_sensor_5:       0x511
      // vcu_report:          0x505
      // vehicle_mode_command:0x105
      // vin_resp1:           0x514
      // vin_resp2:           0x515
      // vin_resp3:           0x516
      // wheelspeed_report:   0x506

      return id == BMS_REPORT || id == 0x101 || id == BRAKE_REPORT || id == 0x103 || id == GEAR_REPORT || id == 0x104 ||
            id == PARK_REPORT || id == 0x102 || id == STEERING_REPORT || id == 0x100 || id == THROTTLE_REPORT || id == ULTR_SENSOR_1 ||
            id == ULTR_SENSOR_2 || id == ULTR_SENSOR_3 || id == ULTR_SENSOR_4 || id == ULTR_SENSOR_5 || id == VCU_REPORT || id == 0x105 ||
            id == VIN_RESP1 || id == VIN_RESP2 || id == VIN_RESP3 || id == WHEELSPEED_REPORT;
    }
    void canTxCallback(const can_msgs::msg::Frame::ConstSharedPtr msg);
    void onVinReport516(const std::uint8_t* msg);
    void onVinReport515(const std::uint8_t* msg);
    void onVinReport514(const std::uint8_t* msg);
    void onBmsReport512(const std::uint8_t* msg);
    void onUltrSensor511(const std::uint8_t* msg);
    void onUltrSensor510(const std::uint8_t* msg);
    void onUltrSensor509(const std::uint8_t* msg);
    void onUltrSensor508(const std::uint8_t* msg);
    void onUltrSensor507(const std::uint8_t* msg);
    void onWheelSpeedReport506(const std::uint8_t* msg);
    void onVcuReport505(const std::uint8_t* msg);
    void onParkReport504(const std::uint8_t* msg);
    void onGearReport503(const std::uint8_t* msg);
    void onSteeringReport502(const std::uint8_t* msg);
    void onBrakeReport501(const std::uint8_t* msg);
    void onThrottleReport500(const std::uint8_t* msg);
    bool control_mode_cmd_{false};

    std::thread can_response_thread_;
    std::atomic<bool> is_running_;
    static void enableAutoMode(Hooke2Interface* hooke2_interface);
    static void disableAutoMode(Hooke2Interface* hooke2_interface);
    bool disable_auto_mode_ = false;
    static bool checkResponse(Hooke2Interface* hooke2_interface, bool need_wait);

    uint8_t can_pub_count_ = 0;

    hooke2_msgs::msg::VinReport514 vin_report514_;
    hooke2_msgs::msg::VinReport515 vin_report515_;
    hooke2_msgs::msg::VinReport516 vin_report516_;
    std::string vehicle_id_ = ""; 
    void processGlobalRpts(
      const bool autodrive_mode_cmd, const std::string vin,
      const hooke2_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt_ptr,
      const hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr wheel_speed_rpt_ptr,
      const hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt_ptr,
      const hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt_ptr);
    void verifyVehicleId();
    bool checkVin();
    void requestVin();
    void disableVinReq();

    hooke2_msgs::msg::ThrottleCmd100 msg_throttle_cmd_;
    hooke2_msgs::msg::BrakeCmd101 msg_brake_cmd_;
    hooke2_msgs::msg::SteeringCmd102 msg_steering_cmd_;
    hooke2_msgs::msg::GearCmd103 msg_gear_cmd_;
    hooke2_msgs::msg::ParkCmd104 msg_park_cmd_;
    hooke2_msgs::msg::VehicleModeCmd105 msg_vehicle_mode_cmd_;
    double desired_speed_ = 0.0;
    double desired_acceleration_ = 0.0;

    std::unique_ptr<hooke2::common::MessageManager<can_msgs::msg::Frame>> message_manager_;
    /**
     * @brief create hooke2 message manager
     * @returns a unique_ptr that points to the created message manager
     */
    std::unique_ptr<hooke2::common::MessageManager<can_msgs::msg::Frame>> CreateMessageManager();

    //@dangshaobo
    // heading rate calculation
    double front_left_wheel_speed = 0.0;
    double front_right_wheel_speed = 0.0;

    //  handle gear shift
    bool vehicle_shifting = false;
    uint8_t vehicle_shift_counter_ = 50;
};

} //  namespace hooke2

#endif  // HOOKE2_INTERFACE__HOOKE2_INTERFACE_HPP_
