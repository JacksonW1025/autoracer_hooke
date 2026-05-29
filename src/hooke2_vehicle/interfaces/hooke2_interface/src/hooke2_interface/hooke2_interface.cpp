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

#include <hooke2_interface/hooke2_interface.hpp>


#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include <wd_byte/byte.hpp>

using wd_byte::Byte;
using hooke2_msgs::msg::ThrottleCmd100;
using hooke2_msgs::msg::BrakeCmd101;
using hooke2_msgs::msg::SteeringCmd102;
using hooke2_msgs::msg::GearCmd103;
using hooke2_msgs::msg::ParkCmd104;
using hooke2_msgs::msg::VehicleModeCmd105;

namespace hooke2
{

Hooke2Interface::Hooke2Interface()
: Node("hooke2_interface"),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo())
{
  /* setup parameters */
  base_frame_id_ = declare_parameter("base_frame_id", "base_link");
  command_timeout_ms_ = declare_parameter("command_timeout_ms", 1000);
  loop_rate_ = declare_parameter("loop_rate", 30.0);

  /* parameters for vehicle specifications */
  // tire_radius_ = vehicle_info_.wheel_radius_m;  //  车轮半径
  // wheel_base_ = vehicle_info_.wheel_base_m;     //  前后轴距
    tire_radius_ = 0.39;  //  车轮半径
  wheel_base_ = 2.0;     //  前后轴距

  steering_offset_ = declare_parameter("steering_offset", 0.0); //  单位 degree？？
  enable_steering_rate_control_ = declare_parameter("enable_steering_rate_control", false);

  /* parameters for emergency stop */
  emergency_brake_ = declare_parameter("emergency_brake", 0.7); //  [0.0 ,1.0]？？

  /* vehicle parameters */
  vgr_coef_a_ = declare_parameter("vgr_coef_a", 15.713);
  vgr_coef_b_ = declare_parameter("vgr_coef_b", 0.053);
  vgr_coef_c_ = declare_parameter("vgr_coef_c", 0.042);
  accel_pedal_offset_ = declare_parameter("accel_pedal_offset", 0.0); //  [0.0 ,1.0]？？
  brake_pedal_offset_ = declare_parameter("brake_pedal_offset", 0.0); //  [0.0 ,1.0]？？

  /* parameters for limitter */
  max_speed_ = declare_parameter("max_speed", 1.5);       //  最大速度
  max_throttle_ = declare_parameter("max_throttle", 0.2); //  最大油门量
  max_brake_ = declare_parameter("max_brake", 0.8);       //  最大制动量
  max_steering_wheel_ = declare_parameter("max_steering_wheel", 4.88);    //  最大转向角度限制, 单位rad
  max_steering_wheel_rate_ = declare_parameter("max_steering_wheel_rate", 4.746); //  最大转向速度限制，单位rad
  min_steering_wheel_rate_ = declare_parameter("min_steering_wheel_rate", 0.5); //  最小转向速度限制，单位rad
  steering_wheel_rate_low_vel_ = declare_parameter("steering_wheel_rate_low_vel", 5.0); //  方向盘转速设置，在车速较低时,单位rad
  steering_wheel_rate_stopped_ = declare_parameter("steering_wheel_rate_stopped", 5.0); //  方向盘转速设置，在车辆停止时,单位rad
  low_vel_thresh_ = declare_parameter("low_vel_thresh", 1.389);  // 5.0kmh，车辆处于低速的速度阈值设置

  /* parameters for turn signal recovery */
  hazard_thresh_time_ = declare_parameter("hazard_thresh_time", 0.20);  // s

  /* parameter for engage sequence */
  need_separate_engage_sequence_ = declare_parameter("need_separate_engage_sequence", false);

  /* parameters for switch vehicle driving interface */
  // 0 - throttle, 1 - acceleration, 2 - speed
  vehicle_driving_interface_ = declare_parameter("vehicle_driving_interface", 2);
  // vehicle_driving_interface_ = 2;

  /* parameter for switch vehicle steering interface */
  // 0 - standar steering(front ackman), 1 - four wheel steering
  vehicle_steering_interface_ = declare_parameter("vehicle_steering_interface", 0);

  /* initialize */
  prev_steer_cmd_time_ = this->now();
  prev_steer_cmd_.header.stamp = prev_steer_cmd_time_;
  prev_steer_cmd_.command = 0.0;
  last_shift_inout_matched_time_ = prev_steer_cmd_time_;

  /* subscribers */
  using std::placeholders::_1;
  using std::placeholders::_2;

  //  From autoware - Control
  //  接收来自autoware Control模块的的控制指令
  control_cmd_sub_ = create_subscription<autoware_control_msgs::msg::Control>(
    "/control/command/control_cmd", 1, std::bind(&Hooke2Interface::callbackControlCmd, this, _1));
  //  接收换挡指令
  gear_cmd_sub_ = create_subscription<autoware_vehicle_msgs::msg::GearCommand>(
    "/control/command/gear_cmd", 1, std::bind(&Hooke2Interface::callbackGearCmd, this, _1));
  //  接收转向灯指令
  turn_indicators_cmd_sub_ =
    create_subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
      "/control/command/turn_indicators_cmd", rclcpp::QoS{1},
      std::bind(&Hooke2Interface::callbackTurnIndicatorsCommand, this, _1));
  //  接收双闪灯指令
  hazard_lights_cmd_sub_ =
    create_subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>(
      "/control/command/hazard_lights_cmd", rclcpp::QoS{1},
      std::bind(&Hooke2Interface::callbackHazardLightsCommand, this, _1));
  //  接收车辆功能性指令（驻车、拉手刹、车身稳定等）？
  actuation_cmd_sub_ = create_subscription<ActuationCommandStamped>(
    "/control/command/actuation_cmd", 1,
    std::bind(&Hooke2Interface::callbackActuationCmd, this, _1));
  //  接收紧急指令(急停AEB？？)
  emergency_sub_ = create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
    "/control/command/emergency_cmd", 1,
    std::bind(&Hooke2Interface::callbackEmergencyCmd, this, _1));
  //  从control订阅控制模式服务(only for receiving AUTO or MANUAL driving command)
  control_mode_server_ = create_service<ControlModeCommand>(
    "/control/control_mode_request", std::bind(&Hooke2Interface::onControlModeRequest, this, _1, _2));
  
  //  并行线程循环执行enableAutoMode()
  can_response_thread_ = std::thread(&Hooke2Interface::enableAutoMode, this);
  
  //  receive data from can_driver
  can_sub_ = create_subscription<can_msgs::msg::Frame>(
    "/can_tx_to_autoware", rclcpp::QoS{100}, std::bind(&Hooke2Interface::canTxCallback, this, _1));
  //  send data to can_driver
  can_pub_ = create_publisher<can_msgs::msg::Frame>("/can_rx_from_autoware", rclcpp::QoS{10});
    
  //  From hooke2，订阅来自hooke2车辆底盘状态
  //  接收后排车门状态
  rear_door_rpt_sub_ = create_subscription<hooke2_msgs::msg::SystemRptInt>(
    "/hooke2/rear_pass_door_rpt", 1, std::bind(&Hooke2Interface::callbackRearDoor, this, _1));
  //  接收方向盘转角状态
  steer_wheel_rpt_sub_ =
    std::make_unique<message_filters::Subscriber<hooke2_msgs::msg::SystemRptFloat>>(
      this, "/hooke2/steering_rpt");
  //  接收车辆轮速状态
  wheel_speed_rpt_sub_ =
    std::make_unique<message_filters::Subscriber<hooke2_msgs::msg::WheelSpeedRpt>>(
      this, "/hooke2/wheel_speed_rpt");
  //  接收车辆加速度状态
  accel_rpt_sub_ = std::make_unique<message_filters::Subscriber<hooke2_msgs::msg::SystemRptFloat>>(
    this, "/hooke2/accel_rpt");
  //  接收车辆制动状态
  brake_rpt_sub_ = std::make_unique<message_filters::Subscriber<hooke2_msgs::msg::SystemRptFloat>>(
    this, "/hooke2/brake_rpt");
  //  接收档位状态
  shift_rpt_sub_ = std::make_unique<message_filters::Subscriber<hooke2_msgs::msg::SystemRptInt>>(
    this, "/hooke2/shift_rpt");
  //  接收转向灯状态
  turn_rpt_sub_ = std::make_unique<message_filters::Subscriber<hooke2_msgs::msg::SystemRptInt>>(
    this, "/hooke2/turn_rpt");
  //  接收车辆全局消息(如接管、通讯超时、接管等信息)
  global_rpt_sub_ = std::make_unique<message_filters::Subscriber<hooke2_msgs::msg::GlobalRpt>>(
    this, "/hooke2/global_rpt");

  //  初始化hooke2的消息同步器
  hooke2_feedbacks_sync_ =
    std::make_unique<message_filters::Synchronizer<Hooke2FeedbacksSyncPolicy>>(
      Hooke2FeedbacksSyncPolicy(300), *steer_wheel_rpt_sub_, *wheel_speed_rpt_sub_, *accel_rpt_sub_,
      *brake_rpt_sub_, *shift_rpt_sub_, *turn_rpt_sub_);
  // 将回调函数注册到同步器上，用于处理同步的消息
  hooke2_feedbacks_sync_->registerCallback(std::bind(
    &Hooke2Interface::callbackHooke2Rpt, this, std::placeholders::_1, std::placeholders::_2,
    std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));

  /* publisher 发送节点 */
  // From Hooke2(Rx)，从hooke2底盘收集数据
  vin_rpt516_pub_ = create_publisher<hooke2_msgs::msg::VinReport516>("/hooke2/vin_rpt516", rclcpp::QoS{5});
  vin_rpt515_pub_ = create_publisher<hooke2_msgs::msg::VinReport515>("/hooke2/vin_rpt515", rclcpp::QoS{5});
  vin_rpt514_pub_ = create_publisher<hooke2_msgs::msg::VinReport514>("/hooke2/vin_rpt514", rclcpp::QoS{5});
  bms_rpt512_pub_ = create_publisher<hooke2_msgs::msg::BmsReport512>("/hooke2/bms_rpt512", rclcpp::QoS{5});
  ultr_sensor511_pub_ = create_publisher<hooke2_msgs::msg::UltrSensor511>("/hooke2/ultr_sensor511", rclcpp::QoS{5});
  ultr_sensor510_pub_ = create_publisher<hooke2_msgs::msg::UltrSensor510>("/hooke2/ultr_sensor510", rclcpp::QoS{5});
  ultr_sensor509_pub_ = create_publisher<hooke2_msgs::msg::UltrSensor509>("/hooke2/ultr_sensor509", rclcpp::QoS{5});
  ultr_sensor508_pub_ = create_publisher<hooke2_msgs::msg::UltrSensor508>("/hooke2/ultr_sensor508", rclcpp::QoS{5});
  ultr_sensor507_pub_ = create_publisher<hooke2_msgs::msg::UltrSensor507>("/hooke2/ultr_sensor507", rclcpp::QoS{5});
  wheelspeed_rpt506_pub_ = create_publisher<hooke2_msgs::msg::WheelSpeedReport506>("/hooke2/wheelspeed_rpt506", rclcpp::QoS{5});
  vcu_rpt505_pub_ = create_publisher<hooke2_msgs::msg::VcuReport505>("/hooke2/vcu_rpt505", rclcpp::QoS{5});
  park_rpt504_pub_ = create_publisher<hooke2_msgs::msg::ParkReport504>("/hooke2/park_rpt504", rclcpp::QoS{5});
  gear_rpt503_pub_ = create_publisher<hooke2_msgs::msg::GearReport503>("/hooke2/gear_rpt503", rclcpp::QoS{5});
  steering_rpt502_pub_ = create_publisher<hooke2_msgs::msg::SteeringReport502>("/hooke2/steering_rpt502", rclcpp::QoS{5});
  brake_rpt501_pub_ = create_publisher<hooke2_msgs::msg::BrakeReport501>("/hooke2/brake_rpt501", rclcpp::QoS{5});
  throttle_rpt500_pub_ = create_publisher<hooke2_msgs::msg::ThrottleReport500>("/hooke2/throttle_rpt500", rclcpp::QoS{5});
  //  车辆方向盘状态发布
  steer_wheel_rpt_pub_ = create_publisher<hooke2_msgs::msg::SystemRptFloat>("/hooke2/steering_rpt", rclcpp::QoS{5});
  //  车辆车轮状态发布
  wheel_speed_rpt_pub_ = create_publisher<hooke2_msgs::msg::WheelSpeedRpt>("/hooke2/wheel_speed_rpt", rclcpp::QoS{5});
  //  车辆加速度状态发布
  accel_rpt_pub_ = create_publisher<hooke2_msgs::msg::SystemRptFloat>("/hooke2/accel_rpt", rclcpp::QoS{5});
  //  车辆制动状态发布
  brake_rpt_pub_ = create_publisher<hooke2_msgs::msg::SystemRptFloat>("/hooke2/brake_rpt", rclcpp::QoS{5});
  //  档位状态发布
  shift_rpt_pub_ = create_publisher<hooke2_msgs::msg::SystemRptInt>("/hooke2/shift_rpt", rclcpp::QoS{5});
  //  转向灯状态发布
  turn_rpt_pub_ = create_publisher<hooke2_msgs::msg::SystemRptInt>("/hooke2/turn_rpt", rclcpp::QoS{5});
  //  车辆状态全局消息发布(如接管、通讯超时、接管、vin等信息)
  global_rpt_pub_ = create_publisher<hooke2_msgs::msg::GlobalRpt>("/hooke2/global_rpt", rclcpp::QoS{5});

  // message_manager_ = CreateMessageManager();  //  lwy20230815，创建hooke2消息管理器

  // To hooke2（Tx），发送给hooke2车辆底盘
  // 油门指令发送节点
  throttle_cmd_pub_ =
    create_publisher<hooke2_msgs::msg::SystemCmdFloat>("/hooke2/throttle_cmd", rclcpp::QoS{1});
  // 加速度指令发送节点
  acceleration_cmd_pub_ =
    create_publisher<hooke2_msgs::msg::SystemCmdFloat>("/hooke2/acceleration_cmd", rclcpp::QoS{1});
  // 速度指令发送节点
  speed_cmd_pub_ =
    create_publisher<hooke2_msgs::msg::SystemCmdFloat>("/hooke2/speed_cmd", rclcpp::QoS{1});
  // 制动指令发送节点
  brake_cmd_pub_ =
    create_publisher<hooke2_msgs::msg::SystemCmdFloat>("/hooke2/brake_cmd", rclcpp::QoS{1});
  // 转向指令发送节点
  steer_cmd_pub_ =
    create_publisher<hooke2_msgs::msg::SteeringCmd>("/hooke2/steering_cmd", rclcpp::QoS{1});
  // 换挡指令发送节点
  shift_cmd_pub_ =
    create_publisher<hooke2_msgs::msg::SystemCmdInt>("/hooke2/shift_cmd", rclcpp::QoS{1});
  // 换挡指令发送节点
  parking_cmd_pub_ =
    create_publisher<hooke2_msgs::msg::SystemCmdInt>("/hooke2/parking_cmd", rclcpp::QoS{1});
  // 转向灯指令发送节点
  turn_cmd_pub_ =
    create_publisher<hooke2_msgs::msg::SystemCmdInt>("/hooke2/turn_cmd", rclcpp::QoS{1});
  // 后门开关控制指令发送节点
  door_cmd_pub_ =
    create_publisher<hooke2_msgs::msg::SystemCmdInt>("/hooke2/rear_pass_door_cmd", rclcpp::QoS{1});
  // 原始转向指令发送节点，用于debug
  raw_steer_cmd_pub_ = create_publisher<hooke2_msgs::msg::SteeringCmd>(
    "/hooke2/raw_steer_cmd", rclcpp::QoS{1});  // only for debug

  // To Autoware 将当前hooke2状态反馈给autoware
  // 车辆驾驶模式(手动or自动)状态发送节点
  control_mode_pub_ = create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>(
    "/vehicle/status/control_mode", rclcpp::QoS{1});
  // 车辆速度状态发送节点
  vehicle_twist_pub_ = create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", rclcpp::QoS{1});
  // 车辆转向状态发送节点
  steering_status_pub_ = create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", rclcpp::QoS{1});
  // 车辆档位状态发送节点
  gear_status_pub_ = create_publisher<autoware_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", rclcpp::QoS{1});
  // 车辆控制模式发送节点
  turn_indicators_status_pub_ =
    create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>(
      "/vehicle/status/turn_indicators_status", rclcpp::QoS{1});
  // 车辆双闪状态发送节点
  hazard_lights_status_pub_ = create_publisher<autoware_vehicle_msgs::msg::HazardLightsReport>(
    "/vehicle/status/hazard_lights_status", rclcpp::QoS{1});
  // 车辆功能性指令状态发送节点
  actuation_status_pub_ =
    create_publisher<ActuationStatusStamped>("/vehicle/status/actuation_status", 1);
  // 车辆方向盘状态发送节点
  steering_wheel_status_pub_ =
    create_publisher<SteeringWheelStatusStamped>("/vehicle/status/steering_wheel_status", 1);
  // 车门状态发送节点
  door_status_pub_ =
    create_publisher<tier4_api_msgs::msg::DoorStatus>("/vehicle/status/door_status", 1);
  // 电池充电状态发送节点
  battery_status_pub_ =
    create_publisher<tier4_vehicle_msgs::msg::BatteryStatus>("/vehicle/status/battery_charge", 1);

  /* service */
  //  From autoware，Tier4网络通讯控制门的开关
  tier4_api_utils::ServiceProxyNodeInterface proxy(this);
  srv_ = proxy.create_service<tier4_external_api_msgs::srv::SetDoor>(
    "/api/vehicle/set/door",
    std::bind(&Hooke2Interface::setDoor, this, std::placeholders::_1, std::placeholders::_2));

  //  定时发布车辆底盘控制指令topic
  const auto period_ns = rclcpp::Rate(loop_rate_).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&Hooke2Interface::publishCommands, this));
  //  定时发布can报文topic
  can_pub_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(50).period(), std::bind(&Hooke2Interface::publishCan, this)); //  lwy20230811，execut every 20 ms

  ParamMonitor::getInstance().startAutoRefresh();

}

//  从发布节点的消息中获取最新的Actuation_cmd
void Hooke2Interface::callbackActuationCmd(const ActuationCommandStamped::ConstSharedPtr msg)
{
  actuation_command_received_time_ = this->now();
  actuation_cmd_ptr_ = msg;
}

//  从发布节点的消息中获取最新的Emergency_cmd
void Hooke2Interface::callbackEmergencyCmd(
  const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg)
{
  is_emergency_ = msg->emergency;
}

//  从发布节点的消息中获取最新的Control_cmd
void Hooke2Interface::callbackControlCmd(
  const autoware_control_msgs::msg::Control::ConstSharedPtr msg)
{
  control_command_received_time_ = this->now();
  control_cmd_ptr_ = msg;
}

//  从发布节点的消息中获取最新的Gear_cmd
void Hooke2Interface::callbackGearCmd(
  const autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
{
  gear_cmd_ptr_ = msg;
}

//  从发布节点的消息中获取最新的Turn_Indicator_cmd
void Hooke2Interface::callbackTurnIndicatorsCommand(
  const autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg)
{
  turn_indicators_cmd_ptr_ = msg;
}

//  从发布节点的消息中获取最新的Hazard_Light_cmd
void Hooke2Interface::callbackHazardLightsCommand(
  const autoware_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg)
{
  hazard_lights_cmd_ptr_ = msg;
}

//  get自动驾驶模式请求（AUTO or MANUAL）
void Hooke2Interface::onControlModeRequest(
  const ControlModeCommand::Request::SharedPtr request,
  const ControlModeCommand::Response::SharedPtr response)
{
  // RCLCPP_ERROR(get_logger(), "request control_mode: %d", request->mode );  //  none-0, AUTO-(1~3), MANUAL-4
  if (request->mode == ControlModeCommand::Request::AUTONOMOUS ||
    request->mode == ControlModeCommand::Request::AUTONOMOUS_STEER_ONLY ||
    request->mode == ControlModeCommand::Request::AUTONOMOUS_VELOCITY_ONLY) {
    ParamMonitor::getInstance().updateCommand("vehicle_mode", "AUTO", fmt::color::yellow);
    engage_cmd_ = true;
    is_clear_override_needed_ = true;
    response->success = true;
    control_mode_cmd_ = true;
    return;
  }

  if (request->mode == ControlModeCommand::Request::MANUAL ||
    request->mode == ControlModeCommand::Request::NO_COMMAND) {
    //  若control模块请求人工驾驶，则退出/禁止进入自动驾驶
    ParamMonitor::getInstance().updateCommand("vehicle_mode", "MANUAL", fmt::color::yellow);
    engage_cmd_ = false;
    is_clear_override_needed_ = true;
    response->success = true;
    control_mode_cmd_ = false;
    return;
  }

  RCLCPP_ERROR(get_logger(), "unsupported control_mode!!");
  response->success = false;
  return;
}

//  从hooke2车辆状态发布节点的消息中获取最新的rear_door反馈状态
void Hooke2Interface::callbackRearDoor(
  const hooke2_msgs::msg::SystemRptInt::ConstSharedPtr rear_door_rpt)
{
  /* publish current door status */
  door_status_pub_->publish(toAutowareDoorStatusMsg(*rear_door_rpt));
}

//  从hooke2车辆状态发布节点的消息中获取最新的消息
void Hooke2Interface::callbackHooke2Rpt(
  const hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr steer_wheel_rpt,
  const hooke2_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt,
  const hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt,
  const hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt,
  const hooke2_msgs::msg::SystemRptInt::ConstSharedPtr shift_rpt,
  const hooke2_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt)
{
  is_hooke2_rpt_received_ = true;
  steer_wheel_rpt_ptr_ = steer_wheel_rpt;
  wheel_speed_rpt_ptr_ = wheel_speed_rpt;
  accel_rpt_ptr_ = accel_rpt;
  brake_rpt_ptr_ = brake_rpt;
  gear_rpt_ptr_ = shift_rpt;
  turn_rpt_ptr_ = turn_rpt;

  //  0、Get Vehicle ID获取VIN码
  verifyVehicleId();

  //  1、更新global_rpt_pub_数据
  processGlobalRpts(control_mode_cmd_, vehicle_id_, turn_rpt_ptr_, 
                  steer_wheel_rpt_ptr_, accel_rpt_ptr_, brake_rpt_ptr_);

  //  2、检查车辆转向、油门、刹车3个使能反馈信息（√）若3个线控均被使能，则is_hooke2_auto_enabled_为true进入自动驾驶状态
  is_hooke2_auto_enabled_ =
    steer_wheel_rpt_ptr_->enabled && accel_rpt_ptr_->enabled && brake_rpt_ptr_->enabled;
  //  调试记录所有车辆线控使能信息
  RCLCPP_DEBUG(
    get_logger(),
    "enabled: is_hooke2_auto_enabled_ %d, steer %d, accel %d, brake %d, shift %d, ",
    is_hooke2_auto_enabled_, steer_wheel_rpt_ptr_->enabled, accel_rpt_ptr_->enabled,
    brake_rpt_ptr_->enabled, gear_rpt_ptr_->enabled);
  
  //  3、更新反馈车速（√）
  const double current_velocity = calculateVehicleVelocity(*wheel_speed_rpt_ptr_, *gear_rpt_ptr_);  // current vehicle speed > 0 [m/s]
  //  4、更新方向盘反馈角度（√）
  const double current_steer_wheel = steer_wheel_rpt_ptr_->output;     // current vehicle steering wheel angle [rad]
  //  5、更新自适应变速轮比率（可能为转向标定表）
  const double adaptive_gear_ratio = calculateVariableGearRatio(current_velocity, current_steer_wheel);
  //  6、更新当前车辆前轮实际转向反馈角度
  // const double current_steer = current_steer_wheel / adaptive_gear_ratio + steering_offset_;
  const double current_steer = current_steer_wheel / 16.6 + steering_offset_;

  std::string current_velocity_string = fmt::format("{:.2f}",current_velocity);
  ParamMonitor::getInstance().updateCurrent("velocity", current_velocity_string, fmt::color::green);
  std::string current_steer_wheel_string = fmt::format("{:.2f}",current_steer_wheel);
  ParamMonitor::getInstance().updateCurrent("steer_wheel", current_steer_wheel_string, fmt::color::green);

  std_msgs::msg::Header header;
  header.frame_id = base_frame_id_;
  header.stamp = get_clock()->now();

  /* publish steering wheel status */
  //  从车底盘解析数据中获取信息后，对autoware平台发布，相当于apollo的chassisDetails和Chassis的关系
  //  7、更新方向盘角度反馈信息（√）
  {
    SteeringWheelStatusStamped steering_wheel_status_msg;
    steering_wheel_status_msg.stamp = header.stamp;
    steering_wheel_status_msg.data = current_steer_wheel;
    steering_wheel_status_pub_->publish(steering_wheel_status_msg);
  }

  /* publish vehicle status control_mode */
  //  8、更新车辆底盘自动驾驶控制反馈状态（自动驾驶 or 手动）（√）
  {
    autoware_vehicle_msgs::msg::ControlModeReport control_mode_msg;
    control_mode_msg.stamp = header.stamp;
    //  检查车辆底盘线控状态
    if (is_hooke2_auto_enabled_) {
      control_mode_msg.mode = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
    } else {
      control_mode_msg.mode = autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;
    }

    control_mode_pub_->publish(control_mode_msg);
  }

  /* publish current shift */
  //  9、更新车辆档位反馈信息（√）
  std::string gear_report_msg_string;
  {
    autoware_vehicle_msgs::msg::GearReport gear_report_msg;
    gear_report_msg.stamp = header.stamp;
    const auto opt_gear_report = toAutowareShiftReport(*gear_rpt_ptr_);

    if (opt_gear_report) {
      gear_report_msg.report = *opt_gear_report;
      gear_status_pub_->publish(gear_report_msg);
    
    switch (gear_report_msg.report)
    {
    case 1:
      gear_report_msg_string = "N";
      break;
    case 2:
      gear_report_msg_string = "D";
      break;
    case 20:
      gear_report_msg_string = "R";
      break;
    case 22:
      gear_report_msg_string = "P";
      break;
    default:
      gear_report_msg_string = "NONE";
      break;
    }
    ParamMonitor::getInstance().updateCurrent("gear", gear_report_msg_string, fmt::color::green);
    }
  }

  /* publish vehicle status twist */
  //  10、更新车辆速度、转角速度反馈信息（√）
  {
    autoware_vehicle_msgs::msg::VelocityReport twist;
    twist.header = header;
    twist.longitudinal_velocity = current_velocity;                                 // [m/s]
    twist.lateral_velocity = current_velocity * std::tan(current_steer);
    // twist.heading_rate = current_velocity * std::tan(current_steer) / wheel_base_;  // [rad/s]
    twist.heading_rate = 0.0;
    //@dangshaobo

    double fr_wheel_speed = front_right_wheel_speed * 3.6;  //  m/s
    double fl_wheel_speed = front_left_wheel_speed * 3.6;   //  m/s
    if (gear_report_msg_string == "R") {
      // vehicle Gear at R
      fr_wheel_speed = -fr_wheel_speed;
      fl_wheel_speed = -fl_wheel_speed;
    }
    // twist.heading_rate = (fr_wheel_speed - fl_wheel_speed) / 1.5 * std::cos(current_steer); // [rad/s]
    vehicle_twist_pub_->publish(twist);
  }

  /* publish current status */
  //  11、更新车辆前轮实际转向反馈信息（√）
  {
    autoware_vehicle_msgs::msg::SteeringReport steer_msg;
    steer_msg.stamp = header.stamp;
    steer_msg.steering_tire_angle = current_steer;
    steering_status_pub_->publish(steer_msg);
  }

  /* publish control status */
  //  12、更新油门、刹车、转向反馈信息（√）
  {
    ActuationStatusStamped actuation_status;
    actuation_status.header = header;
    actuation_status.status.accel_status = accel_rpt_ptr_->output;
    actuation_status.status.brake_status = brake_rpt_ptr_->output;
    actuation_status.status.steer_status = current_steer;
    actuation_status_pub_->publish(actuation_status);

    std::string accel_status_string = fmt::format("{:.2f}",actuation_status.status.brake_status);
    ParamMonitor::getInstance().updateCurrent("accel", accel_status_string, fmt::color::green);
    std::string brake_status_string = fmt::format("{:.2f}",actuation_status.status.brake_status);
    ParamMonitor::getInstance().updateCurrent("brake", brake_status_string, fmt::color::green);

  }

  /* publish current turn signal */
  //  13、更新车辆转向、双闪灯反馈信息（√）
  {
    autoware_vehicle_msgs::msg::TurnIndicatorsReport turn_msg;
    turn_msg.stamp = header.stamp;
    turn_msg.report = toAutowareTurnIndicatorsReport(*turn_rpt_ptr_);  //  从车辆底盘获取转向信息
    turn_indicators_status_pub_->publish(turn_msg);

    autoware_vehicle_msgs::msg::HazardLightsReport hazard_msg;
    hazard_msg.stamp = header.stamp;
    hazard_msg.report = toAutowareHazardLightsReport(*turn_rpt_ptr_);  //  从车辆底盘获取双闪信息
    hazard_lights_status_pub_->publish(hazard_msg);
  }

  // 14、发布电池充电状态到/vehicle/status/battery_charge (如果有电池数据)
  if (bms_rpt512_ptr_) {
    tier4_vehicle_msgs::msg::BatteryStatus battery_status_msg;
    battery_status_msg.stamp = header.stamp;
    battery_status_msg.energy_level = static_cast<float>(bms_rpt512_ptr_->battery_soc); // 电池电量百分比 (0-100)
    battery_status_pub_->publish(battery_status_msg);
  }

}

//  发布车辆控制指令topic
void Hooke2Interface::publishCommands()
{
  /* guard */ 
  //  安全守卫，检查来自于Control模块的指令(actuation_cmd_ptr_、control_cmd_ptr_、gear_cmd_ptr_)以及车辆底盘(is_hooke2_rpt_received_)是否均在线
  if ( !control_cmd_ptr_ || !is_hooke2_rpt_received_ || !gear_cmd_ptr_) {
    // RCLCPP_INFO_THROTTLE(
    //   get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(),
    //   "Msg Guard, wd_hooke2_rpt_received = %d, vehicle_actuation_cmd = %d, control_cmd = %d, gear_cmd = %d. Please check your Vehicle feedback msg or your AUTO-Dringving Start Bottom!", 
    //   is_hooke2_rpt_received_, actuation_cmd_ptr_ != nullptr,
    //   control_cmd_ptr_ != nullptr,  gear_cmd_ptr_ p!= nullptr);
    return;
  }

  const rclcpp::Time current_time = this->now();

  //  更新油门刹车指令值
  double desired_throttle = accel_pedal_offset_;
  double desired_brake = brake_pedal_offset_;
  // desired_speed_ = control_cmd_ptr_->longitudinal.speed;
  desired_speed_ = control_cmd_ptr_->longitudinal.velocity;
  desired_acceleration_ = control_cmd_ptr_->longitudinal.acceleration;
  if (this->parking_actual_ == 1 || this->vehicle_shift_counter_>0) {
    desired_throttle = 0.0;
    desired_speed_ = 0.0;
    desired_acceleration_ = 0.0;
    msg_throttle_cmd_.throttle_pedal_target = 0.0;
    msg_throttle_cmd_.throttle_acc = 0.0;
    msg_throttle_cmd_.vel_target =0.0;

    this->vehicle_shift_counter_ -= 1;
  }
  ParamMonitor::getInstance().updateCommand("velocity", std::to_string(desired_speed_), fmt::color::yellow);
  //@whaledynamic
  std::string accel_status_string = fmt::format("{:.2f}",desired_acceleration_);
  ParamMonitor::getInstance().updateCommand("accel", accel_status_string, fmt::color::yellow);
  ParamMonitor::getInstance().updateCurrent("shifting", std::to_string(this->vehicle_shifting),fmt::color::green);
  // RCLCPP_ERROR(get_logger(), "shifting= %f, desired_speed_ = %f, desired_acceleration_ = %f", this->vehicle_shifting, desired_speed_, desired_acceleration_);

  if (!actuation_cmd_ptr_) {
    if (desired_acceleration_ >= 0) {
      //  acceleration
      desired_throttle = (desired_speed_ / 5.0);
      desired_brake = 0.0;
    } else {
      //  deceleration
      desired_throttle = 0.0;
      desired_brake = desired_acceleration_ / (-5.0);
    }
  } else {
    desired_throttle = actuation_cmd_ptr_->actuation.accel_cmd + accel_pedal_offset_;
    desired_brake = actuation_cmd_ptr_->actuation.brake_cmd + brake_pedal_offset_;
  }

  //@whaledynamic
  std::string brake_status_string = fmt::format("{:.2f}",desired_brake);
  ParamMonitor::getInstance().updateCommand("brake", brake_status_string, fmt::color::yellow);

  // if (actuation_cmd_ptr_->actuation.brake_cmd <= std::numeric_limits<double>::epsilon()) {
  //   desired_brake = 0.0;
  // }

  //  内部处理超时 或 紧急状态逻辑处理
  /* check emergency and timeout */
  const double control_cmd_delta_time_ms =
    (current_time - control_command_received_time_).seconds() * 1000.0;
  // const double actuation_cmd_delta_time_ms =
  //   (current_time - actuation_command_received_time_).seconds() * 1000.0;
  bool timeouted = false;
  const int t_out = command_timeout_ms_;
  // if (t_out >= 0 && (control_cmd_delta_time_ms > t_out || actuation_cmd_delta_time_ms > t_out)) {
  if (t_out >= 0 && (control_cmd_delta_time_ms > t_out > t_out)) {
    RCLCPP_ERROR(
      get_logger(), "Control command timeout, control_cmd_delta_time_ms= %f,  actuation_cmd_delta_time_ms= %f", control_cmd_delta_time_ms, actuation_command_received_time_);
    timeouted = true;
  }

  if (is_emergency_ || timeouted) {
    // RCLCPP_ERROR(
    //   get_logger(), "Emergency Stopping, emergency = %d, control cmd timeouted = %d", is_emergency_, timeouted);
    desired_throttle = 0.0;
    desired_brake = emergency_brake_;

    msg_brake_cmd_.aeb_en_ctrl = BrakeCmd101::AEB_EN_CTRL_ENABLE_AEB;
  } else {
    msg_brake_cmd_.aeb_en_ctrl = BrakeCmd101::AEB_EN_CTRL_DISABLE_AEB;
  }

  const double current_velocity =
    calculateVehicleVelocity(*wheel_speed_rpt_ptr_, *gear_rpt_ptr_);
  const double current_steer_wheel = steer_wheel_rpt_ptr_->output;

  /* calculate desired steering wheel */
  //  根据当前速度计算合适的方向盘转角desired_steer_wheel
  double adaptive_gear_ratio = calculateVariableGearRatio(current_velocity, current_steer_wheel); //  自适应计算出来的传动比
  // double desired_steer_wheel =
  //   (control_cmd_ptr_->lateral.steering_tire_angle - steering_offset_) * adaptive_gear_ratio;   //  rad
  double desired_steer_wheel =
    (control_cmd_ptr_->lateral.steering_tire_angle - steering_offset_) * 16.0;   //  rad
    // control_cmd_ptr_->lateral.steering_tire_angle - steering_offset_;   //  rad
  desired_steer_wheel =
    std::min(std::max(-desired_steer_wheel, -max_steering_wheel_), max_steering_wheel_);

  /* check clear flag */
  //  检查接管信号？？逻辑还不清楚
  bool clear_override = false;
  if (is_hooke2_auto_enabled_ == true) {
    is_clear_override_needed_ = false;
  } else if (is_clear_override_needed_ == true) {
    clear_override = true;
  }

  /* make engage cmd false when a driver overrides vehicle control */
  //  人工接管，则退出/禁止进入自动驾驶
  if (!prev_override_ && global_rpt_ptr_.override_active) {
    engage_cmd_ = false;
  }
  prev_override_ = global_rpt_ptr_.override_active;
  if (disable_auto_mode_) {
    // RCLCPP_WARN_THROTTLE(
    //   get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(),
    //   "Hooke2 is overridden, exit & forbidden AUTO mode untill restart this modules.");
  }
  

  /* make engage cmd false when vehicle report is timed out, e.g. E-stop is depressed */
  //  车辆心跳监控，若车辆通讯中断超过1s，则退出/禁止进入自动驾驶
  const auto global_report_stamp =
    rclcpp::Time(global_rpt_ptr_.header.stamp, get_clock()->get_clock_type());
  const bool report_timed_out = ((current_time - global_report_stamp).seconds() > 1.0);
  if (report_timed_out) {
    // RCLCPP_WARN_THROTTLE(
    //   get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
    //   "Hooke2 report is timed out, enable flag is back to false");
    engage_cmd_ = false;
  }

  /* make engage cmd false when vehicle fault is active */
  //  车辆底盘出现故障，退出/禁止进入自动驾驶，需要根据实际测试情况进行考虑
  if (global_rpt_ptr_.hooke2_sys_fault_active) {
    // RCLCPP_WARN_THROTTLE(
    //   get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
    //   "Hooke2 fault is active, enable flag is back to false");
    engage_cmd_ = false;
  }
  // RCLCPP_INFO_THROTTLE(
  //   get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
  //   "is_hooke2_auto_enabled_ = %d, is_clear_override_needed_ = %d, clear_override = %d",
  //   is_hooke2_auto_enabled_, is_clear_override_needed_, clear_override);


  /* check shift change */
  //  档位切换
  const double brake_for_shift_trans = 0.7; //  单位m/s^2或者[0.0 | 1.0]]？？
  uint16_t desired_shift = gear_rpt_ptr_->output;
  if (std::fabs(current_velocity) < 0.1) {  // velocity is low -> the shift can be changed
    
    if (toHooke2ShiftCmd(*gear_cmd_ptr_) != gear_rpt_ptr_->output) {  // need shift
                                                                          // change.
      desired_throttle = 0.0;
      desired_brake = brake_for_shift_trans;  // set brake to change the shift
      desired_shift = toHooke2ShiftCmd(*gear_cmd_ptr_);
      // RCLCPP_WARN_THROTTLE(
      //   get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      //   "Doing shift change. hooke2 current Gear = %d, desired autoware Gear = %d. set brake_cmd to %f",
      //   gear_rpt_ptr_->output, toHooke2ShiftCmd(*gear_cmd_ptr_), desired_brake);
      this->vehicle_shift_counter_ = 10;
    }

    // using autoware_vehicle_msgs::msg::GearCommand;
    // std::string gear_status_string = "N";
    // if (gear_cmd_ptr_->command == GearCommand::PARK) {
    //   gear_status_string = "P";
    // }
    // if (gear_cmd_ptr_->command == GearCommand::REVERSE) {
    //   msg_park_cmd_.park_target = ParkCmd104::PARK_TARGET_RELEASE;
    //   gear_status_string = "R";
    // }
    // if (gear_cmd_ptr_->command == GearCommand::DRIVE) {
    //   msg_park_cmd_.park_target = ParkCmd104::PARK_TARGET_RELEASE;
    //   gear_status_string = "D";
    // }
    // if (gear_cmd_ptr_->command == GearCommand::NEUTRAL) {
    //   msg_park_cmd_.park_target = ParkCmd104::PARK_TARGET_RELEASE;
    //   gear_status_string = "N";
    // }
    // ParamMonitor::getInstance().updateCommand("parking_actual", std::to_string(msg_park_cmd_.park_target), fmt::color::yellow);
    // ParamMonitor::getInstance().updateCommand("gear", gear_status_string, fmt::color::yellow);
  }

  //  是否分别激活油门、制动和转向功能
  const auto & sep_engage = need_separate_engage_sequence_;
  
  /* publish throttle cmd */
  //  发布油门指令
  {
    hooke2_msgs::msg::SystemCmdFloat throttle_cmd;
    throttle_cmd.header.frame_id = base_frame_id_;
    throttle_cmd.header.stamp = current_time;
 
    throttle_cmd.enable = clear_override && sep_engage ? false : engage_cmd_;  
    throttle_cmd.ignore_overrides = false;
    throttle_cmd.clear_override = clear_override;
    throttle_cmd.command = std::max(0.0, std::min(desired_throttle, max_throttle_));

    msg_throttle_cmd_.throttle_pedal_target = throttle_cmd.command;

    throttle_cmd_pub_->publish(throttle_cmd);
  }

  /* publish acceleration cmd */
  //  发布加速度指令
  {
    hooke2_msgs::msg::SystemCmdFloat acc_cmd;
    acc_cmd.header.frame_id = base_frame_id_;
    acc_cmd.header.stamp = current_time;
 
    acc_cmd.enable = clear_override && sep_engage ? false : engage_cmd_;  
    acc_cmd.ignore_overrides = false;
    acc_cmd.clear_override = clear_override;
    acc_cmd.command = (desired_acceleration_ > 0.0) ? desired_acceleration_ : 0.0;

    msg_throttle_cmd_.throttle_acc = acc_cmd.command;

    acceleration_cmd_pub_->publish(acc_cmd);
  }


  /* publish spped cmd */
  //  发布速度指令
  {
    hooke2_msgs::msg::SystemCmdFloat speed_cmd;
    speed_cmd.header.frame_id = base_frame_id_;
    speed_cmd.header.stamp = current_time;
 
    speed_cmd.enable = clear_override && sep_engage ? false : engage_cmd_;  
    speed_cmd.ignore_overrides = false;
    speed_cmd.clear_override = clear_override;
    if (desired_speed_ >= 0.0){
      speed_cmd.command = std::max(0.0, std::min(desired_speed_, max_speed_));
    } else{
      speed_cmd.command = std::max(0.20, std::min(-desired_speed_, max_speed_));
    }
  
    msg_throttle_cmd_.vel_target = speed_cmd.command;

    speed_cmd_pub_->publish(speed_cmd);
  }

  /* publish brake cmd */
  //  发布制动指令
  {
    hooke2_msgs::msg::SystemCmdFloat brake_cmd;
    brake_cmd.header.frame_id = base_frame_id_;
    brake_cmd.header.stamp = current_time;
    brake_cmd.enable = clear_override && sep_engage ? false : engage_cmd_;
    brake_cmd.ignore_overrides = false;
    brake_cmd.clear_override = clear_override;
    brake_cmd.command = std::max(0.0, std::min(desired_brake, max_brake_));
    msg_brake_cmd_.brake_pedal_target = brake_cmd.command;

    msg_brake_cmd_.brake_dec = (desired_acceleration_ < 0.0)? desired_acceleration_ : 0.0;

    brake_cmd_pub_->publish(brake_cmd);
  }

  /* publish steering cmd */
  //  发布转向指令
  {
    hooke2_msgs::msg::SteeringCmd steer_cmd;
    steer_cmd.header.frame_id = base_frame_id_;
    steer_cmd.header.stamp = current_time;
    steer_cmd.time = current_time;
    steer_cmd.enable = clear_override && sep_engage ? false : engage_cmd_;
    steer_cmd.ignore_overrides = false;
    steer_cmd.clear_override = clear_override;
    //@whaledynamic
    // steer_cmd.rotation_rate = calcSteerWheelRateCmd(adaptive_gear_ratio); //  转速计算rad/s
    steer_cmd.rotation_rate = this->max_steering_wheel_rate_; //  转速计算rad/s
    steer_cmd.command = steerWheelRateLimiter(
      desired_steer_wheel, prev_steer_cmd_.command, current_time, prev_steer_cmd_time_,
      steer_cmd.rotation_rate, current_steer_wheel, engage_cmd_);   //  rad
    steer_cmd.wheel_tier_angle = steer_cmd.command / 16.0;  //  vehicle wheel tier angle, rad

    msg_steering_cmd_.steer_angle_target = steer_cmd.command * static_cast<int32_t>(180.0 / M_PI);  //  can_steering cmd, unit:degree

    steer_cmd_pub_->publish(steer_cmd);
    prev_steer_cmd_ = steer_cmd;
    prev_steer_cmd_time_ = current_time;
  }

  /* publish raw steering cmd for debug */
  //  发布转向指令用于调试
  {
    hooke2_msgs::msg::SteeringCmd raw_steer_cmd;
    raw_steer_cmd.header.frame_id = base_frame_id_;
    raw_steer_cmd.header.stamp = current_time;
    raw_steer_cmd.enable = clear_override && sep_engage ? false : engage_cmd_;
    raw_steer_cmd.ignore_overrides = false;
    raw_steer_cmd.clear_override = clear_override;
    raw_steer_cmd.command = desired_steer_wheel;    //  rad
    raw_steer_cmd.rotation_rate =
      control_cmd_ptr_->lateral.steering_tire_rotation_rate * adaptive_gear_ratio;

    // msg_steering_cmd_.steer_angle_spd =  raw_steer_cmd.rotation_rate; //  未测试，暂时不器用；转速限制在哪里配置？

    raw_steer_cmd_pub_->publish(raw_steer_cmd);
  }

  /* publish shift cmd */
  //  发布换挡指令
  {
    hooke2_msgs::msg::SystemCmdInt shift_cmd;
    shift_cmd.header.frame_id = base_frame_id_;
    shift_cmd.header.stamp = current_time;
    shift_cmd.enable = clear_override && sep_engage ? false : engage_cmd_;
    shift_cmd.ignore_overrides = false;
    shift_cmd.clear_override = clear_override;
    shift_cmd.command = desired_shift;

    msg_gear_cmd_.gear_target = shift_cmd.command;

    shift_cmd_pub_->publish(shift_cmd);
  }

  {
    hooke2_msgs::msg::SystemCmdInt parking_cmd;
    parking_cmd.header.frame_id = base_frame_id_;
    parking_cmd.header.stamp = current_time;
    parking_cmd.enable = clear_override && sep_engage ? false : engage_cmd_;
    parking_cmd.ignore_overrides = false;
    parking_cmd.clear_override = clear_override;
    parking_cmd.command = msg_park_cmd_.park_target == ParkCmd104::PARK_TARGET_RELEASE? 0 : 1;

    parking_cmd_pub_->publish(parking_cmd);
  }

  if (turn_indicators_cmd_ptr_ && hazard_lights_cmd_ptr_) {
    /* publish turn cmd */
    //  发布转向灯、双闪指令
    hooke2_msgs::msg::SystemCmdInt turn_cmd;
    turn_cmd.header.frame_id = base_frame_id_;
    turn_cmd.header.stamp = current_time;
    turn_cmd.enable = clear_override && sep_engage ? false : engage_cmd_;
    turn_cmd.ignore_overrides = false;
    turn_cmd.clear_override = clear_override;
    turn_cmd.command =
      toHooke2TurnCmdWithHazardRecover(*turn_indicators_cmd_ptr_, *hazard_lights_cmd_ptr_); //  转向、双闪指令逻辑处理
    turn_cmd_pub_->publish(turn_cmd);
  }

  RCLCPP_DEBUG(get_logger(),
    "engage_cmd_: %d, disable_auto_mode_: %d", engage_cmd_, disable_auto_mode_);
  
  if (engage_cmd_ == false && is_hooke2_auto_enabled_ == true) {
    //  during AUTO mode
    //  由于接管、通讯超时、底盘故障禁止自动驾驶模式 / 禁止进入自动驾驶模式，需要重新拉起模块才能清除标志位
    
  } else if (engage_cmd_ == true && global_rpt_ptr_.override_active == false) {
    disable_auto_mode_ = false;
  } else if (global_rpt_ptr_.override_active == true) {
    disable_auto_mode_ = true;
  }
}

void Hooke2Interface::publishCan() {
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
    if (!disable_auto_mode_) {
      throttle_command_can.set_throttle_en_ctrl(msg_throttle_cmd_.throttle_en_ctrl);
      // throttle_command_can.set_throttle_en_ctrl(ThrottleCmd100::THROTTLE_EN_CTRL_ENABLE);
      switch (vehicle_driving_interface_)
      {
        case DrivingInterface::THROTTLE_PEDAL_MODE: {
          throttle_command_can.set_throttle_pedal_target(msg_throttle_cmd_.throttle_pedal_target);
          throttle_command_can.set_throttle_acc(0.0);
          break;
        }
        case DrivingInterface::ACCELERATION_MODE: {
          throttle_command_can.set_throttle_acc(msg_throttle_cmd_.throttle_acc);
          throttle_command_can.set_throttle_pedal_target(0.0);
          break;
        }
        case DrivingInterface::SPEED_MODE: {
          throttle_command_can.set_vel_target(msg_throttle_cmd_.vel_target);
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
    // msg_throttle_cmd_.throttle_en_ctrl, vehicle_driving_interface_, msg_throttle_cmd_.vel_target);

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
    if (!disable_auto_mode_) {
      brake_command_can.set_aeb_en_ctrl(msg_brake_cmd_.aeb_en_ctrl);
      // brake_command_can.set_brake_dec(msg_brake_cmd_.brake_dec); //  未测试，暂时不用
      brake_command_can.set_brake_dec(0.0);
      brake_command_can.set_brake_pedal_target(msg_brake_cmd_.brake_pedal_target);
      
      brake_command_can.set_brake_en_ctrl(msg_brake_cmd_.brake_en_ctrl);

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
    //   msg_brake_cmd_.brake_en_ctrl, msg_brake_cmd_.aeb_en_ctrl, msg_brake_cmd_.brake_pedal_target);

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
    double steer_cmd = 0.0;
    // if (engage_cmd_) {
    if (!disable_auto_mode_) {
      steering_command_can.set_steer_en_ctrl(msg_steering_cmd_.steer_en_ctrl);     
      //@whaledynamic 
      steering_command_can.set_steer_angle_target(msg_steering_cmd_.steer_angle_target);
      steering_command_can.set_steer_angle_spd(msg_steering_cmd_.steer_angle_spd);
      
      steer_cmd = msg_steering_cmd_.steer_angle_target;
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

    std::string steer_command_string = fmt::format("{:.2f}",steer_cmd);
    ParamMonitor::getInstance().updateCommand("steer_wheel", steer_command_string, fmt::color::yellow);
    // RCLCPP_WARN(get_logger(), "Can msd steering enable: %d, steer_angle: %d, steer_spd: %d",
    //   msg_steering_cmd_.steer_en_ctrl, msg_steering_cmd_.steer_angle_target, msg_steering_cmd_.steer_angle_spd);

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
    using autoware_vehicle_msgs::msg::GearCommand;
    std::string gear_status_string = "N";
    // if (engage_cmd_) {
    if (!disable_auto_mode_) {
      gear_command_can.set_gear_en_ctrl(msg_gear_cmd_.gear_en_ctrl);
      gear_command_can.set_gear_target(msg_gear_cmd_.gear_target);

      if (msg_gear_cmd_.gear_target == GearCmd103::GEAR_TARGET_PARK) {
        gear_status_string = "P";
      }
      if (msg_gear_cmd_.gear_target == GearCmd103::GEAR_TARGET_REVERSE) {
        gear_status_string = "R";
      }
      if (msg_gear_cmd_.gear_target == GearCmd103::GEAR_TARGET_NEUTRAL) {
        gear_status_string = "N";
      }
      if (msg_gear_cmd_.gear_target == GearCmd103::GEAR_TARGET_DRIVE) {
        gear_status_string = "D";
      }
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

    ParamMonitor::getInstance().updateCommand("gear", gear_status_string, fmt::color::yellow);
    // RCLCPP_WARN(get_logger(), "Can msd gear enable: %d, gear_target: %d",
    //   msg_gear_cmd_.gear_en_ctrl, msg_gear_cmd_.gear_target);
    
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
    std::string park_enable_string = "0";
    std::string park_target_string = "1";
    // if (engage_cmd_) {
    if (!disable_auto_mode_) {
      park_command_can.set_park_target(msg_park_cmd_.park_target);
      park_command_can.set_park_en_ctrl(msg_park_cmd_.park_en_ctrl);

      park_enable_string = std::to_string(msg_park_cmd_.park_en_ctrl);
      park_target_string = std::to_string(msg_park_cmd_.park_target);
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
    
    ParamMonitor::getInstance().updateCommand("park_enable", park_enable_string, fmt::color::yellow);
    ParamMonitor::getInstance().updateCommand("parking_actual", park_target_string, fmt::color::yellow);
    // RCLCPP_WARN(get_logger(), "Can msd park enable: %d, park_target: %d",
    //   msg_park_cmd_.park_en_ctrl, msg_park_cmd_.park_target);

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
    if (!disable_auto_mode_) {
      vehicle_mode_command_can.set_turn_light_ctrl(msg_vehicle_mode_cmd_.turn_light_ctrl);
      vehicle_mode_command_can.set_vin_req(msg_vehicle_mode_cmd_.vin_req);
      vehicle_mode_command_can.set_drive_mode_ctrl(msg_vehicle_mode_cmd_.drive_mode_ctrl);  //  底盘纵向驱动模式选择
      vehicle_mode_command_can.set_steer_mode_ctrl(msg_vehicle_mode_cmd_.steer_mode_ctrl);  //  底盘横向转向模式选择

      // for debug
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

    RCLCPP_DEBUG(get_logger(), "Can msd turn_light: %d, vin_req: %d, drive_mode: %d, steer_mode: %d",
      msg_vehicle_mode_cmd_.turn_light_ctrl, msg_vehicle_mode_cmd_.vin_req, 
      msg_vehicle_mode_cmd_.drive_mode_ctrl, msg_vehicle_mode_cmd_.steer_mode_ctrl);
    
    can_pub_->publish(can_msg105);
  }
}

void Hooke2Interface::enableAutoMode(Hooke2Interface* hooke2_interface) {
  const std::chrono::milliseconds default_period(20); // 20毫秒
  hooke2_interface->is_running_.exchange(true);

  while (rclcpp::ok() && hooke2_interface->is_running_.load()) {
    // RCLCPP_ERROR(hooke2_interface->get_logger(),"driving_interface: %d", hooke2_interface->vehicle_driving_interface_);
    if (hooke2_interface->engage_cmd_) {
      //  received engage AUTO cmd

      if (hooke2_interface->is_hooke2_auto_enabled_ == true) {
        RCLCPP_DEBUG(hooke2_interface->get_logger(), "Vehicle is already in AUTO_DRIVE mode.");
        return;
      }

      //  根据用户配置，车辆底盘使用油门/速度/加速度接口模式
      switch (hooke2_interface->vehicle_driving_interface_) {
        case DrivingInterface::THROTTLE_PEDAL_MODE: {
          // enable throttle pedal control
          hooke2_interface->msg_vehicle_mode_cmd_.drive_mode_ctrl = VehicleModeCmd105::DRIVE_MODE_CTRL_THROTTLE_PADDLE_DRIVE;
          break;
        }
        case DrivingInterface::ACCELERATION_MODE: {
          // enable acceleration control
          hooke2_interface->msg_vehicle_mode_cmd_.drive_mode_ctrl = VehicleModeCmd105::DRIVE_MODE_CTRL_THROTTLE_PADDLE_DRIVE;
          break;
        }
        case DrivingInterface::SPEED_MODE: {
          // enable speed control
          hooke2_interface->msg_vehicle_mode_cmd_.drive_mode_ctrl = VehicleModeCmd105::DRIVE_MODE_CTRL_SPEED_DRIVE;
          break;
        }
        default: {
          // enable throttle pedal control
          hooke2_interface->msg_vehicle_mode_cmd_.drive_mode_ctrl = VehicleModeCmd105::DRIVE_MODE_CTRL_THROTTLE_PADDLE_DRIVE;
          break;
        }
      }

      // enable standard steer control
      switch (hooke2_interface->vehicle_steering_interface_)
      {
        case 0:
          hooke2_interface->msg_vehicle_mode_cmd_.steer_mode_ctrl = VehicleModeCmd105::STEER_MODE_CTRL_STANDARD_STEER;
          break;
        case 1:
          hooke2_interface->msg_vehicle_mode_cmd_.steer_mode_ctrl = VehicleModeCmd105::STEER_MODE_CTRL_NON_DIRECTION_STEER;
          break;
        default:
          break;

      }
      // hooke2_interface->msg_vehicle_mode_cmd_.steer_mode_ctrl = VehicleModeCmd105::STEER_MODE_CTRL_STANDARD_STEER;
      // hooke2_interface->msg_vehicle_mode_cmd_.steer_mode_ctrl = VehicleModeCmd105::STEER_MODE_CTRL_NON_DIRECTION_STEER;

      //  enable Park Wire, release Park
      hooke2_interface->msg_park_cmd_.park_en_ctrl = ParkCmd104::PARK_EN_CTRL_ENABLE;
      hooke2_interface->msg_park_cmd_.park_target = ParkCmd104::PARK_TARGET_RELEASE;

      //  enable Gear Wire, switch to D
      hooke2_interface->msg_gear_cmd_.gear_en_ctrl = GearCmd103::GEAR_EN_CTRL_ENABLE;
      hooke2_interface->msg_gear_cmd_.gear_target = GearCmd103::GEAR_TARGET_DRIVE;

      //  enable Steer Wire, set SteeringSpeed upto 250 degree/s, set target angle to current angle
      hooke2_interface->msg_steering_cmd_.steer_en_ctrl = SteeringCmd102::STEER_EN_CTRL_ENABLE;
      hooke2_interface->msg_steering_cmd_.steer_angle_spd = 220;
      hooke2_interface->msg_steering_cmd_.steer_angle_target = hooke2_interface->steer_wheel_rpt_ptr_->output;

      // enable Brake Wire
      hooke2_interface->msg_brake_cmd_.brake_en_ctrl = BrakeCmd101::BRAKE_EN_CTRL_ENABLE;

      // enable Throttle Wire, set initial speed to current speed
      hooke2_interface->msg_throttle_cmd_.throttle_en_ctrl = ThrottleCmd100::THROTTLE_EN_CTRL_ENABLE;
      hooke2_interface->msg_throttle_cmd_.vel_target = hooke2_interface->current_speed;
      
      if (!hooke2_interface->checkResponse(hooke2_interface, true)) {
        // RCLCPP_WARN_THROTTLE(
        //   hooke2_interface->get_logger(), *hooke2_interface->get_clock(), std::chrono::milliseconds(100).count(),
        //   "Failed to switch to AUTO_DRIVE mode. Please check the emergency button or chassis.");
        // disableAutoMode(hooke2_interface);
        return;
      }

      // RCLCPP_INFO(hooke2_interface->get_logger(), "Enable Steer/Park/Gear/Brake/Throttle to AUTO_DRIVE mode. Update cmd to CAN publisher!");
    } else {
      // do notthing
    }

    std::this_thread::sleep_for(default_period);
  }
}

void Hooke2Interface::disableAutoMode(Hooke2Interface* hooke2_interface) {
  //  当前设计没有自动清除机制
  hooke2_interface->disable_auto_mode_ = true;
}

bool Hooke2Interface::checkResponse(Hooke2Interface* hooke2_interface, bool need_wait) {
  int32_t retry_num = 50;
  bool is_steer_online = false;
  bool is_throttle_online = false;
  bool is_brake_online = false;

  do {
    if (!hooke2_interface->is_hooke2_rpt_received_) {
      RCLCPP_ERROR_THROTTLE(
      hooke2_interface->get_logger(), *hooke2_interface->get_clock(),
      std::chrono::milliseconds(1000).count(),"Get vehicle chassis detail failed!");
      return false;
    }
    bool check_ok = true;

    // is_steer_online = hooke2_interface->steer_wheel_rpt_ptr_->enabled;
    // check_ok = check_ok && is_steer_online;
    
    // is_throttle_online = hooke2_interface->accel_rpt_ptr_->enabled;
    // is_brake_online = hooke2_interface->brake_rpt_ptr_->enabled;

    // check_ok = check_ok && is_throttle_online && is_brake_online;

    if (check_ok) {
      return true;
    } else {
      RCLCPP_WARN(hooke2_interface->get_logger(), "Start AUTO-Driving. Checking vehicle chassis response again. Try.%d", 50 - retry_num);
    }

    if (need_wait) {
      --retry_num;
      std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(20));
    }
  } while (need_wait && retry_num);

  // RCLCPP_ERROR_THROTTLE(
  //     hooke2_interface->get_logger(), *hooke2_interface->get_clock(), std::chrono::milliseconds(1000).count(),
  //     "Check_response failed, is_steer_online: %d, is_throttle_online: %d, is_brake_online: %d",
  //     is_steer_online, is_throttle_online, is_brake_online);
 return false;
}

void Hooke2Interface::processGlobalRpts(
      const bool autodrive_mode_cmd, const std::string vin,
      const hooke2_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt_ptr,
      const hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr wheel_speed_rpt_ptr,
      const hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt_ptr,
      const hooke2_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt_ptr){
  using hooke2_msgs::msg::SystemRptInt;
  
  global_rpt_ptr_.header.stamp = this->now();
  global_rpt_ptr_.header.frame_id = "global_rpt_msg";

  //  Update Control mode(Manual or Auto)检查Control是否下发进入自动驾驶
  if (autodrive_mode_cmd == true) {
    global_rpt_ptr_.enabled = true;
  } else {
    global_rpt_ptr_.enabled = false;
  }
  //  Update override signal检查接管信号（针对Hooke2，目前仅有遥控器AUTO开关 以及 急停按钮开关进行接管）
  switch (turn_rpt_ptr->vehicle_mode_state )
  {
    case SystemRptInt::VEHICLE_MODE_STATE_MANUAL_REMOTE_MODE:   // 0
      global_rpt_ptr_.override_active = true;
      break;
    case SystemRptInt::VEHICLE_MODE_STATE_AUTO_MODE:            //  1
      global_rpt_ptr_.override_active = false;
      break;
    case SystemRptInt::VEHICLE_MODE_STATE_EMERGENCY_MODE:       //  2
      global_rpt_ptr_.override_active = true;
      break;
    default:
      global_rpt_ptr_.override_active = false;
      break;
  }
  //  Update vehicle fault检查车辆故障
  if ((wheel_speed_rpt_ptr->vehicle_communication_fault == true) |
      (accel_rpt_ptr->vehicle_communication_fault == true) |
      (brake_rpt_ptr->vehicle_communication_fault == true))
  {
    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(), 
    //   "Chassis error! steer fault: %d, throttle fault: %d, brake fault: %d",
    //   wheel_speed_rpt_ptr->vehicle_communication_fault,
    //   accel_rpt_ptr->vehicle_communication_fault,
    //   brake_rpt_ptr->vehicle_communication_fault);

    global_rpt_ptr_.hooke2_sys_fault_active = true;
  }
  else
  {
    global_rpt_ptr_.hooke2_sys_fault_active = false;
  }
  //  更新车辆VIN码
  global_rpt_ptr_.vehicle_vin = vin;

  global_rpt_pub_->publish(global_rpt_ptr_); //  发布global_rpt_pub_信息
}

void Hooke2Interface::verifyVehicleId() {
  if (!checkVin()) {
    requestVin();
  } else {
    disableVinReq();
  }
}

bool Hooke2Interface::checkVin() {
  // Vin set 17 bits, such as "LSBN1234567890123" is prased as
  // vin17(L),vin16(S),vin15(B),......,vin03(1),vin02(2),vin01(3)
  vehicle_id_ += vin_report514_.vin00;
  vehicle_id_ += vin_report514_.vin01;
  vehicle_id_ += vin_report514_.vin02;
  vehicle_id_ += vin_report514_.vin03;
  vehicle_id_ += vin_report514_.vin04;
  vehicle_id_ += vin_report514_.vin05;
  vehicle_id_ += vin_report514_.vin06;
  vehicle_id_ += vin_report514_.vin07;
  vehicle_id_ += vin_report515_.vin08;
  vehicle_id_ += vin_report515_.vin09;
  vehicle_id_ += vin_report515_.vin10;
  vehicle_id_ += vin_report515_.vin11;
  vehicle_id_ += vin_report515_.vin12;
  vehicle_id_ += vin_report515_.vin13;
  vehicle_id_ += vin_report515_.vin15;
  vehicle_id_ += vin_report515_.vin15;
  vehicle_id_ += vin_report516_.vin16;
  std::reverse(vehicle_id_.begin(), vehicle_id_.end());


  // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), std::chrono::milliseconds(100).count(), 
  // "Vehicle ID: %d, %s, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d",
  // vin_report514_.vin00, vin_report514_.vin01, vin_report514_.vin02, vin_report514_.vin03,
  // vin_report514_.vin04, vin_report514_.vin05, vin_report514_.vin06, vin_report514_.vin07,
  // vin_report515_.vin08, vin_report515_.vin09, vin_report515_.vin10, vin_report515_.vin11,
  // vin_report515_.vin12, vin_report515_.vin13, vin_report515_.vin14, vin_report515_.vin15,
  // vin_report516_.vin16);

  if (vehicle_id_.size() == 17) {
    return true;
  }

  //@whaledynamic
  // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), std::chrono::milliseconds(30000).count(), "Vehicle ID is not correct!");
  // return false;
  return true;
}

void Hooke2Interface::requestVin() {
  msg_vehicle_mode_cmd_.vin_req = VehicleModeCmd105::VIN_REQ_VIN_REQ_ENABLE;
}

void Hooke2Interface::disableVinReq() {
  msg_vehicle_mode_cmd_.vin_req = VehicleModeCmd105::VIN_REQ_VIN_REQ_DISABLE;
}

//  计算方向盘转向速度指令
double Hooke2Interface::calcSteerWheelRateCmd(const double gear_ratio)
{
  const auto current_vel =
    std::fabs(calculateVehicleVelocity(*wheel_speed_rpt_ptr_, *gear_rpt_ptr_));

  // send low steer rate at low speed
  if (current_vel < std::numeric_limits<double>::epsilon()) {
    return steering_wheel_rate_stopped_;
  } else if (current_vel < low_vel_thresh_) {
    return steering_wheel_rate_low_vel_;
  }

  if (!enable_steering_rate_control_) {
    return max_steering_wheel_rate_;
  }

  constexpr double margin = 1.5;
  const double rate = margin * control_cmd_ptr_->lateral.steering_tire_rotation_rate * gear_ratio;
  return std::min(std::max(std::fabs(rate), min_steering_wheel_rate_), max_steering_wheel_rate_);
}

//  计算车速
double Hooke2Interface::calculateVehicleVelocity(
  const hooke2_msgs::msg::WheelSpeedRpt & wheel_speed_rpt,
  const hooke2_msgs::msg::SystemRptInt & shift_rpt)
{
  //  档位判断
  // const double sign = (shift_rpt.output == hooke2_msgs::msg::SystemRptInt::SHIFT_REVERSE) ? -1 : 1;
  //  车速计算
  // const double vel =
  //   (wheel_speed_rpt.rear_left_wheel_speed + wheel_speed_rpt.rear_right_wheel_speed) * 0.5 *
  //   tire_radius_;
  // const double vel =
  //   (wheel_speed_rpt.rear_left_wheel_speed + wheel_speed_rpt.rear_right_wheel_speed) * 0.5;
  //@whaledynamic
  const double vel = this->current_speed;
  return vel;
}

//  计算方向盘可变转速速率
double Hooke2Interface::calculateVariableGearRatio(const double vel, const double steer_wheel)
{
  return std::max(
    1e-5, vgr_coef_a_ + vgr_coef_b_ * vel * vel - vgr_coef_c_ * std::fabs(steer_wheel));
}

//  根据autoware逻辑切换档位指令
uint16_t Hooke2Interface::toHooke2ShiftCmd(
  const autoware_vehicle_msgs::msg::GearCommand & gear_cmd)
{
  using hooke2_msgs::msg::SystemCmdInt;            // hooke2GEAR: NONE-0, P-1, R-2, N-3, D-4, LOW-5
  using autoware_vehicle_msgs::msg::GearCommand; // autowareGEAG: NONE-0, N-1, D-(2~19), R-(20~21), P-22, LOW-(23~24)
  if (gear_cmd.command == GearCommand::PARK) {
    msg_park_cmd_.park_target = ParkCmd104::PARK_TARGET_PARKING_TRIGGER;
    return SystemCmdInt::SHIFT_PARK;  //  P-1
  }
  else if (gear_cmd.command == GearCommand::REVERSE) {
    msg_park_cmd_.park_target = ParkCmd104::PARK_TARGET_RELEASE;
    return SystemCmdInt::SHIFT_REVERSE; //  R-2
  }
  else if (gear_cmd.command == GearCommand::DRIVE) {
    msg_park_cmd_.park_target = ParkCmd104::PARK_TARGET_RELEASE;
    return SystemCmdInt::SHIFT_FORWARD; //  D-4
  }
  else if (gear_cmd.command == GearCommand::LOW) {
    msg_park_cmd_.park_target = ParkCmd104::PARK_TARGET_RELEASE;
    return SystemCmdInt::SHIFT_FORWARD; //  D-4
  }
  else if (gear_cmd.command == GearCommand::NEUTRAL) {
    msg_park_cmd_.park_target = ParkCmd104::PARK_TARGET_RELEASE;
    return SystemCmdInt::SHIFT_NEUTRAL ; // N-3
  }

  // return SystemCmdInt::SHIFT_NONE;    //  N-0
  msg_park_cmd_.park_target = ParkCmd104::PARK_TARGET_RELEASE;
  return SystemCmdInt::SHIFT_FORWARD; //  D-4
}

//  从底盘获取数据更新autoware车辆档位信息
std::optional<int32_t> Hooke2Interface::toAutowareShiftReport(
  const hooke2_msgs::msg::SystemRptInt & shift)
{
  using autoware_vehicle_msgs::msg::GearReport;    // autowareGEAG: NONE-0, N-1, D-(2~19), R-(20~21), P-22, LOW-(23~24)
  using hooke2_msgs::msg::SystemRptInt;              // hooke2GEAR: NONE-0, P-1, R-2, N-3, D-4, LOW-5

  if (shift.output == SystemRptInt::SHIFT_PARK) {
    return GearReport::PARK;
  }
  if (shift.output == SystemRptInt::SHIFT_REVERSE) {
    return GearReport::REVERSE;
  }
  if (shift.output == SystemRptInt::SHIFT_FORWARD) {
    return GearReport::DRIVE;
  }
  if (shift.output == SystemRptInt::SHIFT_LOW) {
    return GearReport::DRIVE;
  }
  if (shift.output == SystemRptInt::SHIFT_NEUTRAL) {
    return GearReport::NEUTRAL;
  }
  return GearReport::NONE;
}

//  切换双闪、转向指令
uint16_t Hooke2Interface::toHooke2TurnCmd(
  const autoware_vehicle_msgs::msg::TurnIndicatorsCommand & turn,
  const autoware_vehicle_msgs::msg::HazardLightsCommand & hazard)
{
  using autoware_vehicle_msgs::msg::HazardLightsCommand;
  using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
  using hooke2_msgs::msg::SystemCmdInt;

  // NOTE: hazard lights command has a highest priority here.
  if (hazard.command == HazardLightsCommand::ENABLE) {
    msg_vehicle_mode_cmd_.turn_light_ctrl = VehicleModeCmd105::TURN_LIGHT_CTRL_HAZARD_WARNING_LAMPSTS_ON;
    return SystemCmdInt::TURN_HAZARDS;
  }
  if (turn.command == TurnIndicatorsCommand::ENABLE_LEFT) {
    msg_vehicle_mode_cmd_.turn_light_ctrl = VehicleModeCmd105::TURN_LIGHT_CTRL_LEFT_TURNLAMP_ON;
    return SystemCmdInt::TURN_LEFT;
  }
  if (turn.command == TurnIndicatorsCommand::ENABLE_RIGHT) {
    msg_vehicle_mode_cmd_.turn_light_ctrl = VehicleModeCmd105::TURN_LIGHT_CTRL_RIGHT_TURNLAMP_ON;
    return SystemCmdInt::TURN_RIGHT;
  }
  msg_vehicle_mode_cmd_.turn_light_ctrl = VehicleModeCmd105::TURN_LIGHT_CTRL_TURNLAMP_OFF;
  return SystemCmdInt::TURN_NONE;
}

//  双闪灯恢复逻辑
uint16_t Hooke2Interface::toHooke2TurnCmdWithHazardRecover(
  const autoware_vehicle_msgs::msg::TurnIndicatorsCommand & turn,
  const autoware_vehicle_msgs::msg::HazardLightsCommand & hazard)
{
  using hooke2_msgs::msg::SystemRptInt;

  if (!engage_cmd_ || turn_rpt_ptr_->command == turn_rpt_ptr_->output) {

    last_shift_inout_matched_time_ = get_clock()->now();
    return toHooke2TurnCmd(turn, hazard);
  }

  if ((get_clock()->now() - last_shift_inout_matched_time_).seconds() < hazard_thresh_time_) {
    return toHooke2TurnCmd(turn, hazard);
  }

  // hazard recover mode
  if (hazard_recover_count_ > hazard_recover_cmd_num_) {
    last_shift_inout_matched_time_ = get_clock()->now();
    hazard_recover_count_ = 0;
  }
  hazard_recover_count_++;

  if (
    turn_rpt_ptr_->command != SystemRptInt::TURN_HAZARDS &&
    turn_rpt_ptr_->output == SystemRptInt::TURN_HAZARDS) {
    // publish hazard commands for turning off the hazard lights
    //  若当前未打开双闪，则根据指令下发双闪开启命令
    return SystemRptInt::TURN_HAZARDS;
  } else if (  
    // NOLINT
    turn_rpt_ptr_->command == SystemRptInt::TURN_HAZARDS &&
    turn_rpt_ptr_->output != SystemRptInt::TURN_HAZARDS) {
    // publish none commands for turning on the hazard lights？？？
    //  当前已打开双闪，则根据指令关闭双闪灯
    return SystemRptInt::TURN_NONE;
  } else {
    // something wrong
    // RCLCPP_ERROR_STREAM(
    //   get_logger(), "turn signal command and output do not match. "
    //                   << "COMMAND: " << turn_rpt_ptr_->command
    //                   << "; OUTPUT: " << turn_rpt_ptr_->output);
    return toHooke2TurnCmd(turn, hazard);
  }
}

//  从车辆获取信息更新转向灯指示信息
int32_t Hooke2Interface::toAutowareTurnIndicatorsReport(
  const hooke2_msgs::msg::SystemRptInt & turn)
{
  using autoware_vehicle_msgs::msg::TurnIndicatorsReport;
  using hooke2_msgs::msg::SystemRptInt;

  if (turn.output == SystemRptInt::TURN_RIGHT) {
    return TurnIndicatorsReport::ENABLE_RIGHT;
  } else if (turn.output == SystemRptInt::TURN_LEFT) {    
    return TurnIndicatorsReport::ENABLE_LEFT;
  } else if (turn.output == SystemRptInt::TURN_NONE) {
    return TurnIndicatorsReport::DISABLE;
  }
  return TurnIndicatorsReport::DISABLE;
}

//  从车辆获取信息更新双闪灯指示信息
int32_t Hooke2Interface::toAutowareHazardLightsReport(
  const hooke2_msgs::msg::SystemRptInt & hazard)
{
  using autoware_vehicle_msgs::msg::HazardLightsReport;
  using hooke2_msgs::msg::SystemRptInt;

  if (hazard.output == SystemRptInt::TURN_HAZARDS) {
    return HazardLightsReport::ENABLE;
  }

  return HazardLightsReport::DISABLE;
}

//  施加方向盘转速限制
double Hooke2Interface::steerWheelRateLimiter(
  const double current_steer_cmd, const double prev_steer_cmd,
  const rclcpp::Time & current_steer_time, const rclcpp::Time & prev_steer_time,
  const double steer_rate, const double current_steer_output, const bool engage)
{
  if (!engage) {
    // return current steer as steer command ( do not apply steer rate filter )
    return current_steer_output;
  }

  const double dsteer = current_steer_cmd - prev_steer_cmd;
  const double dt = std::max(0.0, (current_steer_time - prev_steer_time).seconds());
  const double max_dsteer = std::fabs(steer_rate) * dt;
  // const double limited_steer_cmd =
  //   prev_steer_cmd + std::min(std::max(-max_dsteer, dsteer), max_dsteer);
  // return limited_steer_cmd; //  限制过后的方向盘转向指令 TODO
  return current_steer_cmd;
}

//  ？？？
hooke2_msgs::msg::SystemCmdInt Hooke2Interface::createClearOverrideDoorCommand()
{
  hooke2_msgs::msg::SystemCmdInt door_cmd;
  door_cmd.header.frame_id = "base_link";
  door_cmd.header.stamp = this->now();
  door_cmd.clear_override = true;
  return door_cmd;
}

//  开门指令
hooke2_msgs::msg::SystemCmdInt Hooke2Interface::createDoorCommand(const bool open)
{
  hooke2_msgs::msg::SystemCmdInt door_cmd;
  door_cmd.header.frame_id = "base_link";
  door_cmd.header.stamp = this->now();
  door_cmd.enable = true;

  if (open) {
    door_cmd.command = hooke2_msgs::msg::SystemCmdInt::DOOR_OPEN;
  } else {
    door_cmd.command = hooke2_msgs::msg::SystemCmdInt::DOOR_CLOSE;
  }
  return door_cmd;
}

//  控制后座控制门开关，更新门开关信息
void Hooke2Interface::setDoor(
  const tier4_external_api_msgs::srv::SetDoor::Request::SharedPtr request,
  const tier4_external_api_msgs::srv::SetDoor::Response::SharedPtr response)
{
  if (!engage_cmd_) {
    // when the vehicle mode is manual, ignore the request.
    response->status.code = tier4_external_api_msgs::msg::ResponseStatus::IGNORED;
    response->status.message = "Current vehicle mode is manual. The request is ignored.";
    return;
  }

  // open/close the door
  door_cmd_pub_->publish(createClearOverrideDoorCommand());
  rclcpp::Rate(10.0).sleep();  // avoid message loss
  door_cmd_pub_->publish(createDoorCommand(request->open));
  response->status.code = tier4_external_api_msgs::msg::ResponseStatus::SUCCESS;

  if (request->open) {
    // open the door
    response->status.message = "Success to open the door.";
  } else {
    // close the door
    response->status.message = "Success to close the door.";
  }
}

//  将车辆后座控制门开关状态发布到autoware
tier4_api_msgs::msg::DoorStatus Hooke2Interface::toAutowareDoorStatusMsg(
  const hooke2_msgs::msg::SystemRptInt & msg_ptr)
{
  using hooke2_msgs::msg::SystemRptInt;
  using tier4_api_msgs::msg::DoorStatus;
  DoorStatus door_status;

  door_status.status = DoorStatus::UNKNOWN;

  if (msg_ptr.command == SystemRptInt::DOOR_CLOSE && msg_ptr.output == SystemRptInt::DOOR_OPEN) {
    // do not used (command & output are always the same value)
    door_status.status = DoorStatus::DOOR_CLOSING;
  } else if (  // NOLINT
    msg_ptr.command == SystemRptInt::DOOR_OPEN && msg_ptr.output == SystemRptInt::DOOR_CLOSE) {
    // do not used (command & output are always the same value)
    door_status.status = DoorStatus::DOOR_OPENING;
  } else if (msg_ptr.output == SystemRptInt::DOOR_CLOSE) {
    door_status.status = DoorStatus::DOOR_CLOSED;
  } else if (msg_ptr.output == SystemRptInt::DOOR_OPEN) {
    door_status.status = DoorStatus::DOOR_OPENED;
  }

  return door_status;
}

// CAN receiver, data parser
void Hooke2Interface::canTxCallback(const can_msgs::msg::Frame::ConstSharedPtr msg)
{
  if (msg->is_rtr || msg->is_error) {
    RCLCPP_ERROR(get_logger(), "Receive error can msg.");
    return;
  }
  RCLCPP_DEBUG (get_logger(), "Receive can msg. Id:%u", msg->id);

  if (isTargetId(msg->id)) {
    switch (msg->id) {
      case THROTTLE_REPORT:
        onThrottleReport500(&msg->data[0]);
        break;
      case BRAKE_REPORT:
        onBrakeReport501(&msg->data[0]);
        break;
      case STEERING_REPORT:
        onSteeringReport502(&msg->data[0]);
        break;
      case GEAR_REPORT:
        onGearReport503(&msg->data[0]);
        break;
      case PARK_REPORT:
        onParkReport504(&msg->data[0]);
        break;
      case VCU_REPORT:
        onVcuReport505(&msg->data[0]);
        break;
      case WHEELSPEED_REPORT:
        onWheelSpeedReport506(&msg->data[0]);
        break;
      case ULTR_SENSOR_1:
        onUltrSensor507(&msg->data[0]);
        break;
      case ULTR_SENSOR_2:
        onUltrSensor508(&msg->data[0]);
        break;
      case ULTR_SENSOR_3:
        onUltrSensor509(&msg->data[0]);
        break;
      case ULTR_SENSOR_4:
        onUltrSensor510(&msg->data[0]);
        break;
      case ULTR_SENSOR_5:
        onUltrSensor511(&msg->data[0]);
        break;
      case BMS_REPORT:
        onBmsReport512(&msg->data[0]);
        break;
      case VIN_RESP1:
        onVinReport514(&msg->data[0]);
        break;
      case VIN_RESP2:
        onVinReport515(&msg->data[0]);
        break;
      case VIN_RESP3:
        onVinReport516(&msg->data[0]);
        break;
      default:
        break;
    }
  }
}

void Hooke2Interface::onVinReport516(const std::uint8_t* msg)
{
  // vin16， 0 - 255
  Byte t0(msg + 0);
  int32_t x = t0.get_byte(0, 8);

  int vin16 = x;

  vin_report516_.header.frame_id = "base_link";
  vin_report516_.header.stamp = this->now();
  vin_report516_.vin16 = vin16;
  vin_rpt516_pub_->publish(vin_report516_);
}

void Hooke2Interface::onVinReport515(const std::uint8_t* msg)
{
  // vin15， 0 - 255
  Byte t0(msg + 7);
  int32_t x = t0.get_byte(0, 8);

  int vin15 = x;
  // vin14， 0 - 255
  Byte t1(msg + 6);
  x = t1.get_byte(0, 8);

  int vin14 = x;
  // vin13， 0 - 255
  Byte t2(msg + 5);
  x = t2.get_byte(0, 8);

  int vin13 = x;
  // vin12， 0 - 255
  Byte t3(msg + 4);
  x = t3.get_byte(0, 8);

  int vin12 = x;
  // vin11， 0 - 255
  Byte t4(msg + 3);
  x = t4.get_byte(0, 8);

  int vin11 = x;
  // vin10， 0 - 255
  Byte t5(msg + 2);
  x = t5.get_byte(0, 8);

  int vin10 = x;
  // vin09， 0 - 255
  Byte t6(msg + 1);
  x = t6.get_byte(0, 8);

  int vin09 = x;
  // vin08， 0 - 255
  Byte t7(msg + 0);
  x = t7.get_byte(0, 8);

  int vin08 = x;

  vin_report515_.header.frame_id = "base_link";
  vin_report515_.header.stamp = this->now();
  vin_report515_.vin15 = vin15;
  vin_report515_.vin14 = vin14;
  vin_report515_.vin13 = vin13;
  vin_report515_.vin12 = vin12;
  vin_report515_.vin11 = vin11;
  vin_report515_.vin10 = vin10;
  vin_report515_.vin09 = vin09;
  vin_report515_.vin08 = vin08;
  vin_rpt515_pub_->publish(vin_report515_);
}

void Hooke2Interface::onVinReport514(const std::uint8_t* msg)
{
  // vin07， 0 - 255
  Byte t0(msg + 7);
  int32_t x = t0.get_byte(0, 8);

  int vin07 = x;
  // vin06， 0 - 255
  Byte t1(msg + 6);
  x = t1.get_byte(0, 8);

  int vin06 = x;
  // vin05， 0 - 255
  Byte t2(msg + 5);
  x = t2.get_byte(0, 8);

  int vin05 = x;
  // vin04， 0 - 255
  Byte t3(msg + 4);
  x = t3.get_byte(0, 8);

  int vin04 = x;
  // vin03， 0 - 255
  Byte t4(msg + 3);
  x = t4.get_byte(0, 8);

  int vin03 = x;
  // vin02， 0 - 255
  Byte t5(msg + 2);
  x = t5.get_byte(0, 8);

  int vin02 = x;
  // vin01， 0 - 255
  Byte t6(msg + 1);
  x = t6.get_byte(0, 8);

  int vin01 = x;
  // vin00， 0 - 255
  Byte t7(msg + 0);
  x = t7.get_byte(0, 8);

  int vin00 = x;

  vin_report514_.header.frame_id = "base_link";
  vin_report514_.header.stamp = this->now();
  vin_report514_.vin07 = vin07;
  vin_report514_.vin06 = vin06;
  vin_report514_.vin05 = vin05;
  vin_report514_.vin04 = vin04;
  vin_report514_.vin03 = vin03;
  vin_report514_.vin02 = vin02;
  vin_report514_.vin01 = vin01;
  vin_report514_.vin00 = vin00;
  vin_rpt514_pub_->publish(vin_report514_);
}

void Hooke2Interface::onBmsReport512(const std::uint8_t* msg)
{
  // battery_current， -3200(Charging) - 3353.5 A
  Byte t0(msg + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(msg + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double battery_current = x * 0.100000 + -3200.000000;

  // battery_voltage，0 - 300 V
  Byte t2(msg + 0);
  x = t2.get_byte(0, 8);

  Byte t3(msg + 1);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double battery_voltage = x * 0.010000;

  // battery_soc， 0 - 100%
  Byte t4(msg + 4);
  x = t4.get_byte(0, 8);

  int battery_soc = x;

  hooke2_msgs::msg::BmsReport512 bms_report;
  bms_report.header.frame_id = "base_link";
  bms_report.header.stamp = this->now();
  bms_report.battery_current = battery_current; //  if battery_current < 0, vehicle is charging
  bms_report.battery_voltage = battery_voltage;
  bms_report.battery_soc = battery_soc;
  bms_rpt512_pub_->publish(bms_report);

  // 保存BMS数据指针用于后续发布
  bms_rpt512_ptr_ = std::make_shared<hooke2_msgs::msg::BmsReport512>(bms_report);
}

void Hooke2Interface::onUltrSensor511(const std::uint8_t* msg)
{
  // uiuss7_tof_direct, 0-65535 cm (ultrasonic distance)
  Byte t0(msg + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(msg + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double uiuss7_tof_direct = x * 0.017240;

  // uiuss6_tof_direct, 0-65535 cm (ultrasonic distance)
  Byte t2(msg + 4);
  x = t2.get_byte(0, 8);

  Byte t3(msg + 5);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double uiuss6_tof_direct = x * 0.017240;

  // uiuss1_tof_direct, 0-65535 cm (ultrasonic distance)
  Byte t4(msg + 2);
  x = t4.get_byte(0, 8);

  Byte t5(msg + 3);
  t = t5.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double uiuss1_tof_direct = x * 0.017240;

  // uiuss0_tof_direct, 0-65535 cm (ultrasonic distance)
  Byte t6(msg + 0);
  x = t6.get_byte(0, 8);

  Byte t7(msg + 1);
  t = t7.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double uiuss0_tof_direct = x * 0.017240;

  hooke2_msgs::msg::UltrSensor511 ultr_sensor_report;
  ultr_sensor_report.header.frame_id = "base_link";
  ultr_sensor_report.header.stamp = this->now();
  ultr_sensor_report.uiuss7_tof_direct = uiuss7_tof_direct;
  ultr_sensor_report.uiuss6_tof_direct = uiuss6_tof_direct;
  ultr_sensor_report.uiuss1_tof_direct = uiuss1_tof_direct;
  ultr_sensor_report.uiuss0_tof_direct = uiuss0_tof_direct;
  ultr_sensor511_pub_->publish(ultr_sensor_report);
}

void Hooke2Interface::onUltrSensor510(const std::uint8_t* msg)
{
  // uiuss5_tof_indirect, 0-65535 cm (ultrasonic distance)
  Byte t0(msg + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(msg + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double uiuss5_tof_indirect = x * 0.017240;

  // uiuss4_tof_indirect, 0-65535 cm (ultrasonic distance)
  Byte t2(msg + 4);
  x = t2.get_byte(0, 8);

  Byte t3(msg + 5);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double uiuss4_tof_indirect = x * 0.017240;

  // uiuss3_tof_indirect, 0-65535 cm (ultrasonic distance)
  Byte t4(msg + 2);
  x = t4.get_byte(0, 8);

  Byte t5(msg + 3);
  t = t5.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double uiuss3_tof_indirect = x * 0.017240;

  // uiuss2_tof_indirect, 0-65535 cm (ultrasonic distance)
  Byte t6(msg + 0);
  x = t6.get_byte(0, 8);

  Byte t7(msg + 1);
  t = t7.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double uiuss2_tof_indirect = x * 0.017240;

  hooke2_msgs::msg::UltrSensor510 ultr_sensor_report;
  ultr_sensor_report.header.frame_id = "base_link";
  ultr_sensor_report.header.stamp = this->now();
  ultr_sensor_report.uiuss5_tof_indirect = uiuss5_tof_indirect;
  ultr_sensor_report.uiuss4_tof_indirect = uiuss4_tof_indirect;
  ultr_sensor_report.uiuss3_tof_indirect = uiuss3_tof_indirect;
  ultr_sensor_report.uiuss2_tof_indirect = uiuss2_tof_indirect;
  ultr_sensor510_pub_->publish(ultr_sensor_report);
}

void Hooke2Interface::onUltrSensor509(const std::uint8_t* msg)
{
  // uiuss5_tof_direct, 0-65535 cm (ultrasonic distance)
  Byte t0(msg + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(msg + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double uiuss5_tof_direct = x * 0.017240;

  // uiuss4_tof_direct, 0-65535 cm (ultrasonic distance)
  Byte t2(msg + 4);
  x = t2.get_byte(0, 8);

  Byte t3(msg + 5);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double uiuss4_tof_direct = x * 0.017240;

  // uiuss3_tof_direct, 0-65535 cm (ultrasonic distance)
  Byte t4(msg + 2);
  x = t4.get_byte(0, 8);

  Byte t5(msg + 3);
  t = t5.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double uiuss3_tof_direct = x * 0.017240;

  // uiuss2_tof_direct, 0-65535 cm (ultrasonic distance)
  Byte t6(msg + 0);
  x = t6.get_byte(0, 8);

  Byte t7(msg + 1);
  t = t7.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double uiuss2_tof_direct = x * 0.017240;

  hooke2_msgs::msg::UltrSensor509 ultr_sensor_report;
  ultr_sensor_report.header.frame_id = "base_link";
  ultr_sensor_report.header.stamp = this->now();
  ultr_sensor_report.uiuss5_tof_direct = uiuss5_tof_direct;
  ultr_sensor_report.uiuss4_tof_direct = uiuss4_tof_direct;
  ultr_sensor_report.uiuss3_tof_direct = uiuss3_tof_direct;
  ultr_sensor_report.uiuss2_tof_direct = uiuss2_tof_direct;
  ultr_sensor509_pub_->publish(ultr_sensor_report);
}

void Hooke2Interface::onUltrSensor508(const std::uint8_t* msg)
{
  // uiuss9_tof_indirect, 0-65535 cm (ultrasonic distance)
  Byte t0(msg + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(msg + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double uiuss9_tof_indirect = x * 0.017240;

  // uiuss8_tof_indirect, 0-65535 cm (ultrasonic distance)
  Byte t2(msg + 0);
  x = t2.get_byte(0, 8);

  Byte t3(msg + 1);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double uiuss8_tof_indirect = x * 0.017240;

  //  uiuss11_tof_indirect, 0-65535 cm (ultrasonic distance)
  Byte t4(msg + 6);
  x = t4.get_byte(0, 8);

  Byte t5(msg + 7);
  t = t5.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double uiuss11_tof_indirect = x * 0.017240;

  // uiuss10_tof_indirect, 0-65535 cm (ultrasonic distance)
  Byte t6(msg + 4);
  x = t6.get_byte(0, 8);

  Byte t7(msg + 5);
  t = t7.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double uiuss10_tof_indirect = x * 0.017240;

  hooke2_msgs::msg::UltrSensor508 ultr_sensor_report;
  ultr_sensor_report.header.frame_id = "base_link";
  ultr_sensor_report.header.stamp = this->now();
  ultr_sensor_report.uiuss9_tof_indirect = uiuss9_tof_indirect;
  ultr_sensor_report.uiuss8_tof_indirect = uiuss8_tof_indirect;
  ultr_sensor_report.uiuss11_tof_indirect = uiuss11_tof_indirect;
  ultr_sensor_report.uiuss10_tof_indirect = uiuss10_tof_indirect;
  ultr_sensor508_pub_->publish(ultr_sensor_report);
}

void Hooke2Interface::onUltrSensor507(const std::uint8_t* msg)
{
  // uiuss9_tof_direct, 0-65535 cm (ultrasonic distance)
  Byte t0(msg + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(msg + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double uiuss9_tof_direct = x * 0.017240;

  // uiuss8_tof_direct, 0-65535 cm (ultrasonic distance)
  Byte t2(msg + 0);
  x = t2.get_byte(0, 8);

  Byte t3(msg + 1);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double uiuss8_tof_direct = x * 0.017240;

  //  uiuss11_tof_direct, 0-65535 cm (ultrasonic distance)
  Byte t4(msg + 6);
  x = t4.get_byte(0, 8);

  Byte t5(msg + 7);
  t = t5.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double uiuss11_tof_direct = x * 0.017240;

  // uiuss10_tof_direct, 0-65535 cm (ultrasonic distance)
  Byte t6(msg + 4);
  x = t6.get_byte(0, 8);

  Byte t7(msg + 5);
  t = t7.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double uiuss10_tof_direct = x * 0.017240;

  hooke2_msgs::msg::UltrSensor507 ultr_sensor_report;
  ultr_sensor_report.header.frame_id = "base_link";
  ultr_sensor_report.header.stamp = this->now();
  ultr_sensor_report.uiuss9_tof_direct = uiuss9_tof_direct;
  ultr_sensor_report.uiuss8_tof_direct = uiuss8_tof_direct;
  ultr_sensor_report.uiuss11_tof_direct = uiuss11_tof_direct;
  ultr_sensor_report.uiuss10_tof_direct = uiuss10_tof_direct;
  ultr_sensor507_pub_->publish(ultr_sensor_report);
}

void Hooke2Interface::onWheelSpeedReport506(const std::uint8_t* msg)
{
  // rear_right_wheel_speed，0 - 60 km/h
  Byte t0(msg + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(msg + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double rear_right_wheel_speed = x * 0.0001000;

  // rear_left_wheel_speed，0 - 60 km/h
  Byte t2(msg + 4);
  x = t2.get_byte(0, 8);

  Byte t3(msg + 5);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double rear_left_wheel_speed = x * 0.0001000;

  // front_right_wheel_speed，0 - 60 km/h
  Byte t4(msg + 2);
  x = t4.get_byte(0, 8);

  Byte t5(msg + 3);
  t = t5.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double front_right_wheel_speed = x * 0.0001000;

  // front_left_wheel_speed，0 - 60 km/h
  Byte t6(msg + 0);
  x = t6.get_byte(0, 8);

  Byte t7(msg + 1);
  t = t7.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double front_left_wheel_speed = x * 0.0001000;

  hooke2_msgs::msg::WheelSpeedReport506 wheelspeed_report;
  wheelspeed_report.header.frame_id = "base_link";
  wheelspeed_report.header.stamp = this->now();
  wheelspeed_report.rear_right = rear_right_wheel_speed;
  wheelspeed_report.rear_left = rear_left_wheel_speed;
  wheelspeed_report.front_right = front_right_wheel_speed;
  wheelspeed_report.front_left = front_left_wheel_speed;
  wheelspeed_rpt506_pub_->publish(wheelspeed_report);

  hooke2_msgs::msg::WheelSpeedRpt wheel_speed_rpt;
  wheel_speed_rpt.header.frame_id = "hooke2_wheel_speed";
  wheel_speed_rpt.header.stamp = this->now();
  wheel_speed_rpt.rear_left_wheel_speed = rear_left_wheel_speed;
  wheel_speed_rpt.rear_right_wheel_speed = rear_right_wheel_speed;
  wheel_speed_rpt.front_left_wheel_speed = front_left_wheel_speed;
  wheel_speed_rpt.front_right_wheel_speed = front_right_wheel_speed;
  wheel_speed_rpt_pub_->publish(wheel_speed_rpt);
}

void Hooke2Interface::onVcuReport505(const std::uint8_t* msg)
{
  // brake_light_actual, {0: 'BRAKE_LIGHT_ACTUAL_BRAKELIGHT_OFF', 1: 'BRAKE_LIGHT_ACTUAL_BRAKELIGHT_ON'}
  Byte t0(msg + 1);
  int32_t x = t0.get_byte(3, 1);

  uint8_t brake_light_actual = static_cast<uint8_t>(x);

  // turn_light_actual, {0: 'TURN_LIGHT_ACTUAL_TURNLAMPSTS_OFF', 1: 'TURN_LIGHT_ACTUAL_LEFT_TURNLAMPSTS_ON', 2: 'TURN_LIGHT_ACTUAL_RIGHT_TURNLAMPSTS_ON', 3: 'TURN_LIGHT_ACTUAL_HAZARD_WARNING_LAMPSTS_ON'}
  Byte t1(msg + 7);
  x = t1.get_byte(0, 2);

  uint8_t turn_light_actual = static_cast<uint8_t>(x);

  // chassis_errcode
  Byte t2(msg + 5);
  x = t2.get_byte(0, 8);

  uint8_t chassis_errcode = static_cast<uint8_t>(x);

  // Drive_mode_stsType, {0: 'DRIVE_MODE_STS_THROTTLE_PADDLE_DRIVE_MODE', 1: 'DRIVE_MODE_STS_SPEED_DRIVE_MODE'}
  Byte t3(msg + 4);
  x = t3.get_byte(5, 3);

  uint8_t drive_mode_sts = static_cast<uint8_t>(x);

  // steer_mode_sts, {0: 'STEER_MODE_STS_STANDARD_STEER_MODE', 1: 'STEER_MODE_STS_NON_DIRECTION_STEER_MODE', 2: 'STEER_MODE_STS_SYNC_DIRECTION_STEER_MODE'}
  Byte t4(msg + 1);
  x = t4.get_byte(0, 3);

  uint8_t steer_mode_sts = static_cast<uint8_t>(x);

  // vehicle_mode_state, {0: 'VEHICLE_MODE_STATE_MANUAL_REMOTE_MODE', 1: 'VEHICLE_MODE_STATE_AUTO_MODE', 2: 'VEHICLE_MODE_STATE_EMERGENCY_MODE', 3: 'VEHICLE_MODE_STATE_STANDBY_MODE'}
  Byte t5(msg + 4);
  x = t5.get_byte(3, 2);

  uint8_t vehicle_mode_state = static_cast<uint8_t>(x);

  // frontcrash_state, {0: 'FRONTCRASH_STATE_NO_EVENT', 1: 'FRONTCRASH_STATE_CRASH_EVENT'}
  Byte t6(msg + 4);
  x = t6.get_byte(1, 1);

  uint8_t frontcrash_state = static_cast<uint8_t>(x);

  // backcrash_state, {0: 'BACKCRASH_STATE_NO_EVENT', 1: 'BACKCRASH_STATE_CRASH_EVENT'}
  Byte t7(msg + 4);
  x = t7.get_byte(2, 1);

  uint8_t backcrash_state = static_cast<uint8_t>(x);

  // aeb_state, {0: 'AEB_STATE_INACTIVE', 1: 'AEB_STATE_ACTIVE'}
  Byte t8(msg + 4);
  x = t8.get_byte(0, 1);

  uint8_t aeb_state = static_cast<uint8_t>(x);

  // acc,  -10 - 10 m/s^2
  Byte t9(msg + 0);
  x = t9.get_byte(0, 8);

  Byte t10(msg + 1);
  int32_t t = t10.get_byte(4, 4);

  x <<= 4;
  x |= t;

  x <<= 20;
  x >>= 20;
  double vehicle_acc = x * 0.010000;

  // speed，-32.768 - 32.767 m/s
  Byte t11(msg + 2);
  x = t11.get_byte(0, 8);

  Byte t12(msg + 3);
  t = t12.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double vehicle_speed = x * 0.001000;

  hooke2_msgs::msg::VcuReport505 vcu_report;
  vcu_report.header.frame_id = "base_link";
  vcu_report.header.stamp = this->now();
  vcu_report.brake_light_actual = brake_light_actual; //  刹车灯
  vcu_report.turn_light_actual = turn_light_actual;   //  转向、双闪灯
  vcu_report.chassis_errcode = chassis_errcode;       //  底盘故障码
  vcu_report.drive_mode_sts = drive_mode_sts;         //  行车驱动模式
  vcu_report.steer_mode_sts = steer_mode_sts;         //  转向模式
  vcu_report.vehicle_mode_state = vehicle_mode_state; //  车辆遥控模式状态
  vcu_report.frontcrash_state = frontcrash_state;     //  前防撞梁状态
  vcu_report.backcrash_state = backcrash_state;       //  后防撞梁状态
  vcu_report.aeb_state = aeb_state;                   //  AEB制动状态
  vcu_report.acc = vehicle_acc;                       //  车辆加速度
  vcu_report.speed = vehicle_speed;                   //  车辆速度, m/s
  //@whaledynamic

  std::string mode_string;
  switch (int(vcu_report.vehicle_mode_state)) {
      case 0:
          mode_string = "MANUAL";
          is_hooke2_auto_enabled_ = false;
          break;
      case 1:
          mode_string = "AUTO";
          break;
      case 2:
          mode_string = "EMERGENCY";
          break;
      case 3:
          mode_string = "STANDBY";
          break;
      default:
          mode_string = "MANUAL";
          break;
  }
  ParamMonitor::getInstance().updateCurrent("vehicle_mode", mode_string, fmt::color::green);

  this->current_speed = vehicle_speed;
  vcu_rpt505_pub_->publish(vcu_report);

  using hooke2_msgs::msg::SystemRptInt;
  SystemRptInt turn_rpt;
  turn_rpt.header.stamp = this->now();
  turn_rpt.header.frame_id = "hooke2_turn";
  switch (turn_light_actual) {
    case 0:
      turn_rpt.output = SystemRptInt::TURN_NONE;
      break;
    case 1:
      turn_rpt.output = SystemRptInt::TURN_LEFT;
      break;
    case 2:
      turn_rpt.output = SystemRptInt::TURN_RIGHT;
      break;
    case 3:
      turn_rpt.output = SystemRptInt::TURN_HAZARDS;
      break;
    default:
      turn_rpt.output = SystemRptInt::TURN_NONE;
      break;
  }
  turn_rpt.vehicle_mode_state = vehicle_mode_state;
  turn_rpt_pub_->publish(turn_rpt);
}

void Hooke2Interface::onParkReport504(const std::uint8_t* msg)
{
  // Parking_actualType, {0: 'PARKING_ACTUAL_RELEASE', 1: 'PARKING_ACTUAL_PARKING_TRIGGER'}
  Byte t0(msg + 0);
  int32_t x = t0.get_byte(0, 1);

  uint8_t parking_actual = static_cast<uint8_t>(x);

  // Park_fltType, {0: 'PARK_FLT_NO_FAULT', 1: 'PARK_FLT_FAULT'}
  Byte t1(msg + 1);
  x = t1.get_byte(0, 8);

  uint8_t parking_flt = static_cast<uint8_t>(x);

  ParamMonitor::getInstance().updateCurrent("parking_actual", std::to_string(parking_actual), fmt::color::green);

  hooke2_msgs::msg::ParkReport504 park_report;
  park_report.header.frame_id = "base_link";
  park_report.header.stamp = this->now();
  park_report.park_flt = parking_flt;
  park_report.parking_actual = parking_actual;
  this->parking_actual_ = parking_actual;
  park_rpt504_pub_->publish(park_report);
}

void Hooke2Interface::onGearReport503(const std::uint8_t* msg)
{
  // Gear_fltType, {0: 'GEAR_FLT_NO_FAULT', 1: 'GEAR_FLT_FAULT'}
  Byte t0(msg + 1);
  int32_t x = t0.get_byte(0, 8);

  uint8_t gear_flt_type = static_cast<uint8_t>(x);
  
  // Gear_actualType, {0: 'GEAR_ACTUAL_INVALID', 1: 'GEAR_ACTUAL_PARK', 2: 'GEAR_ACTUAL_REVERSE', 3: 'GEAR_ACTUAL_NEUTRAL', 4: 'GEAR_ACTUAL_DRIVE'}
  Byte t1(msg + 0);
  x = t1.get_byte(0, 3);

  uint8_t gear_actual = static_cast<uint8_t>(x);

  hooke2_msgs::msg::GearReport503 gear_report;
  gear_report.header.frame_id = "base_link";
  gear_report.header.stamp = this->now();
  gear_report.gear_flt = gear_flt_type;
  gear_report.gear_actual = gear_actual;
  gear_rpt503_pub_->publish(gear_report);

  using hooke2_msgs::msg::SystemRptInt;
  SystemRptInt shift_rpt;
  shift_rpt.header.frame_id = "hooke2_gear";
  shift_rpt.header.stamp = this->now();
  shift_rpt.hooke2_fault = (gear_flt_type == 1)? true : false;
  switch (gear_actual)
  {
    case hooke2_msgs::msg::GearReport503::GEAR_ACTUAL_INVALID :
      //  INVALID
      shift_rpt.output = SystemRptInt::SHIFT_NONE;
      break;
    case hooke2_msgs::msg::GearReport503::GEAR_ACTUAL_PARK :
      //  PARK
      shift_rpt.output = SystemRptInt::SHIFT_PARK;
      break;
    case hooke2_msgs::msg::GearReport503::GEAR_ACTUAL_REVERSE :
      //  REVERSE
      shift_rpt.output = SystemRptInt::SHIFT_REVERSE;
      break;
    case hooke2_msgs::msg::GearReport503::GEAR_ACTUAL_NEUTRAL :
      //  NEUTRAL
      shift_rpt.output = SystemRptInt::SHIFT_NEUTRAL;
      break;
    case hooke2_msgs::msg::GearReport503::GEAR_ACTUAL_DRIVE :
      //  DRIVE
      shift_rpt.output = SystemRptInt::SHIFT_FORWARD;
      break;
    default:
      shift_rpt.output = SystemRptInt::SHIFT_NONE;
  }
  shift_rpt_pub_->publish(shift_rpt);
}

void Hooke2Interface::onSteeringReport502(const std::uint8_t* msg)
{
  // steer_angle_spd_actual, 0-250 deg/s
  Byte t0(msg + 7);
  int32_t x = t0.get_byte(0, 8);
  int steer_angle_spd_actual = x;

  // Steer_flt2Type, {0: 'STEER_FLT2_NO_FAULT', 1: 'STEER_FLT2_STEER_SYSTEM_COMUNICATION_FAULT'}
  Byte t1(msg + 2);
  x = t1.get_byte(0, 8);
  uint8_t steer_flt2_type = static_cast<uint8_t>(x);

  // Steer_flt1Type, {0: 'STEER_FLT1_NO_FAULT', 1: 'STEER_FLT1_STEER_SYSTEM_HARDWARE_FAULT'}
  Byte t2(msg + 1);
  x = t2.get_byte(0, 8);
  uint8_t steer_flt1_type = static_cast<uint8_t>(x);

  // Steer_en_stateType, {0: 'STEER_EN_STATE_MANUAL', 1: 'STEER_EN_STATE_AUTO', 2: 'STEER_EN_STATE_TAKEOVER', 3: 'STEER_EN_STATE_STANDBY'}
  Byte t3(msg + 0);
  x = t3.get_byte(0, 2);
  uint8_t steer_en_state_type = static_cast<uint8_t>(x);

  // steer_angle_actual, (right)-465 - 465(left) degree
  Byte t4(msg + 3);
  x = t4.get_byte(0, 8);
  Byte t5(msg + 4);
  int32_t t = t5.get_byte(0, 8);
  x <<= 8;
  x |= t;
  int steer_angle_actual = x + -500.000000;

  hooke2_msgs::msg::SteeringReport502 steering_report;
  steering_report.header.frame_id = "base_link";
  steering_report.header.stamp = this->now();
  steering_report.steer_flt1 = steer_flt1_type;
  steering_report.steer_flt2 = steer_flt2_type;
  steering_report.steer_en_state_type = steer_en_state_type;
  steering_report.steer_angle_actual = steer_angle_actual;  //  unit: degree;
  steering_report.steer_angle_spd_actual = steer_angle_spd_actual;  //  degree / s
  steering_rpt502_pub_->publish(steering_report);

  hooke2_msgs::msg::SystemRptFloat steer_wheel_rpt;
  steer_wheel_rpt.header.frame_id = "hooke2_steer";
  steer_wheel_rpt.header.stamp = this->now();
  steer_wheel_rpt.time = this->now();
  steer_wheel_rpt.enabled = (steer_en_state_type == 1) ? true : false;
  steer_wheel_rpt.output = steer_angle_actual / (180.0 / M_PI);  //  unit: rad
  steer_wheel_rpt.steer_tier_angle = (steer_angle_actual / (180.0 / M_PI)) / 16.0; // vehicle wheel tire angle, unit: rad
  steer_wheel_rpt.vehicle_communication_fault = (steer_flt2_type == 1)? true : false;
  steer_wheel_rpt.vehicle_hardware_fault = (steer_flt1_type == 1)? true : false;
  steer_wheel_rpt_pub_->publish(steer_wheel_rpt);
}

void Hooke2Interface::onBrakeReport501(const std::uint8_t* msg){
  // brake_pedal_actual, 0 - 100%
  Byte t0(msg + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(msg + 4);
  int32_t t = t1.get_byte(0, 8);

  x <<= 8;
  x |= t;

  double brake_pedal_actual = x * 0.100000;

  // brake_flt2type, {0: 'BRAKE_FLT2_NO_FAULT', 1: 'BRAKE_FLT2_BRAKE_SYSTEM_COMUNICATION_FAULT'}
  Byte t2(msg + 2);
  x = t2.get_byte(0, 8);
  uint8_t brake_flt2_type = static_cast<uint8_t>(x);

  // brake_flt1type, {0: 'BRAKE_FLT1_NO_FAULT', 1: 'BRAKE_FLT1_BRAKE_SYSTEM_HARDWARE_FAULT'}
  Byte t3(msg + 1);
  x = t3.get_byte(0, 8);
  uint8_t brake_flt1_type = static_cast<uint8_t>(x);

  // brake_en_statetype, {0: 'BRAKE_EN_STATE_MANUAL', 1: 'BRAKE_EN_STATE_AUTO', 2: 'BRAKE_EN_STATE_TAKEOVER', 3: 'BRAKE_EN_STATE_STANDBY'}
  Byte t4(msg + 0);
  x = t4.get_byte(0, 2);
  uint8_t brake_en_state_type = static_cast<uint8_t>(x);

  hooke2_msgs::msg::BrakeReport501 brake_report;
  brake_report.header.frame_id = "base_link";
  brake_report.header.stamp = this->now();
  brake_report.brake_flt1 = brake_flt1_type;
  brake_report.brake_flt2 = brake_flt2_type;
  brake_report.brake_en_state = brake_en_state_type;
  brake_report.brake_pedal_actual = brake_pedal_actual;
  brake_rpt501_pub_->publish(brake_report);

  hooke2_msgs::msg::SystemRptFloat brake_rpt;
  brake_rpt.header.frame_id = "hooke2_brake";
  brake_rpt.header.stamp = this->now();
  brake_rpt.enabled = (brake_en_state_type == 1)? true : false;
  brake_rpt.vehicle_communication_fault = (brake_flt2_type == 1)? true : false;
  brake_rpt.vehicle_hardware_fault = (brake_flt1_type == 1)? true : false;
  brake_rpt.output = brake_pedal_actual / 100.0;  // [0.0 | 1.0]
  brake_rpt_pub_->publish(brake_rpt);
}

void Hooke2Interface::onThrottleReport500(const std::uint8_t* msg){
  // throttle_pedal_actual, 0 - 100 %
  Byte t0(msg + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(msg + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double throttle_pedal_actual = x * 0.100000;

  // Throttle_flt2Type, {0: 'THROTTLE_FLT2_NO_FAULT', 1: 'THROTTLE_FLT2_DRIVE_SYSTEM_COMUNICATION_FAULT'}
  Byte t2(msg + 2);
  x = t2.get_byte(0, 8);

  int16_t throttle_flt2_type = static_cast<int16_t>(x);

  // Throttle_flt1Type, {0: 'THROTTLE_FLT1_NO_FAULT', 1: 'THROTTLE_FLT1_DRIVE_SYSTEM_HARDWARE_FAULT'}
  Byte t3(msg + 1);
  x = t3.get_byte(0, 8);

  int16_t throttle_flt1_type = static_cast<int16_t>(x);

  // Throttle_en_stateType, {0: 'THROTTLE_EN_STATE_MANUAL', 1: 'THROTTLE_EN_STATE_AUTO', 2: 'THROTTLE_EN_STATE_TAKEOVER', 3: 'THROTTLE_EN_STATE_STANDBY'}
  Byte t4(msg + 0);
  x = t4.get_byte(0, 2);

  int16_t throttle_en_state = static_cast<int16_t>(x);

  hooke2_msgs::msg::ThrottleReport500 throttle_report;
  throttle_report.header.frame_id = "base_link";
  throttle_report.header.stamp = this->now();
  throttle_report.throttle_flt1 = throttle_flt1_type;
  throttle_report.throttle_flt2 = throttle_flt2_type;
  throttle_report.throttle_en_state = throttle_en_state;
  throttle_report.throttle_pedal_actual = throttle_pedal_actual;
  throttle_rpt500_pub_->publish(throttle_report);

  hooke2_msgs::msg::SystemRptFloat accel_rpt;
  accel_rpt.header.frame_id = "hooke2_throttle";
  accel_rpt.header.stamp = this->now();
  accel_rpt.enabled = (throttle_en_state == 1)? true : false;
  accel_rpt.output = throttle_pedal_actual / 100.0;   //  [0.0 | 1.0]
  accel_rpt.vehicle_hardware_fault = (throttle_flt1_type == 1)? true : false; //  hooke2 throttle fault is always 1
  accel_rpt.vehicle_communication_fault = (throttle_flt2_type == 1)? true : false;
  accel_rpt_pub_->publish(accel_rpt);
}

std::unique_ptr<hooke2::common::MessageManager<can_msgs::msg::Frame>>
Hooke2Interface::CreateMessageManager() {
  return std::unique_ptr<hooke2::common::MessageManager<can_msgs::msg::Frame>>(new hooke2::Hooke2MessageManager());
}

uint8_t Hooke2Interface::getVehicleDrivingInterface() const {
  return vehicle_driving_interface_;
}

bool Hooke2Interface::getDiasbleAutoMode() const {
  return disable_auto_mode_;
}

hooke2_msgs::msg::ThrottleCmd100 Hooke2Interface::getThrottleCmd() const{
  return msg_throttle_cmd_;
}

hooke2_msgs::msg::BrakeCmd101 Hooke2Interface::getBrakeCmd() const{
  return msg_brake_cmd_;
}

hooke2_msgs::msg::SteeringCmd102 Hooke2Interface::getSteeringCmd() const{
  return msg_steering_cmd_;
}

hooke2_msgs::msg::GearCmd103 Hooke2Interface::getGearCmd() const{
  return msg_gear_cmd_;
}

hooke2_msgs::msg::ParkCmd104 Hooke2Interface::getParkCmd() const{
  return msg_park_cmd_;
}

hooke2_msgs::msg::VehicleModeCmd105 Hooke2Interface::getVehicleModeCmd() const{
  return msg_vehicle_mode_cmd_;
}

}  // namespace
