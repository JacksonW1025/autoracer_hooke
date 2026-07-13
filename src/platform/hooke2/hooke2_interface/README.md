# hooke2_interface

`hooke2_interface` is the package to connect Autoware with hooke2.

## Input / Output

### Input topics

- From Autoware

  | Name                                   | Type                                                     | Description                                           |
  | -------------------------------------- | -------------------------------------------------------- | ----------------------------------------------------- |
  | `/control/command/control_cmd`         | autoware_auto_control_msgs::msg::AckermannControlCommand | lateral and longitudinal control command              |
  | `/control/command/gear_cmd`            | autoware_auto_vehicle_msgs::msg::GearCommand             | gear command                                          |
  | `/control/command/turn_indicators_cmd` | autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand   | turn indicators command                               |
  | `/control/command/hazard_lights_cmd`   | autoware_auto_vehicle_msgs::msg::HazardLightsCommand     | hazard lights command                                 |
  | `/vehicle/engage`                      | autoware_auto_vehicle_msgs::msg::Engage                  | engage command                                        |
  | `/vehicle/command/actuation_cmd`       | tier4_vehicle_msgs::msg::ActuationCommandStamped         | actuation (accel/brake pedal, steering wheel) command |
  | `/control/command/emergency_cmd`       | tier4_vehicle_msgs::msg::VehicleEmergencyStamped         | emergency command                                     |
Apollo planning&control massages summary


- From hooke2

  | Name                      | Type                              | Description                                                             |
  | ------------------------- | --------------------------------- | ----------------------------------------------------------------------- |
  | `/hooke2/steering_rpt`    | hooke2_msgs::msg::SystemRptFloat | current steering wheel angle                                            |
  | `/hooke2/wheel_speed_rpt` | hooke2_msgs::msg::WheelSpeedRpt  | current wheel speed                                                     |
  | `/hooke2/accel_rpt`       | hooke2_msgs::msg::SystemRptFloat | current accel pedal                                                     |
  | `/hooke2/brake_rpt`       | hooke2_msgs::msg::SystemRptFloat | current brake pedal                                                     |
  | `/hooke2/shift_rpt`       | hooke2_msgs::msg::SystemRptInt   | current gear status                                                     |
  | `/hooke2/turn_rpt`        | hooke2_msgs::msg::SystemRptInt   | current turn indicators status                                          |
  | `/hooke2/global_rpt`      | hooke2_msgs::msg::GlobalRpt      | current status of other parameters (e.g. override_active, can_time_out) |
Apollo vehicle protocol parser, CAN receiver


### Output topics

- To hooke2

  | Name                   | Type                              | Description                                           |
  | ---------------------- | --------------------------------- | ----------------------------------------------------- |
  | `hooke2/accel_cmd`     | hooke2_msgs::msg::SystemCmdFloat | accel pedal command                                   |
  | `hooke2/brake_cmd`     | hooke2_msgs::msg::SystemCmdFloat | brake pedal command                                   |
  | `hooke2/steering_cmd`  | hooke2_msgs::msg::SystemCmdFloat | steering wheel angle and angular velocity command     |
  | `hooke2/shift_cmd`     | hooke2_msgs::msg::SystemCmdInt   | gear command                                          |
  | `hooke2/turn_cmd`      | hooke2_msgs::msg::SystemCmdInt   | turn indicators command                               |
  | `hooke2/raw_steer_cmd` | hooke2_msgs::msg::SteerSystemCmd | raw steering wheel angle and angular velocity command |
Apollo vehicle CAN sender

- To Autoware

  | Name                                     | Type                                                    | Description                                          |
  | ---------------------------------------- | ------------------------------------------------------- | ---------------------------------------------------- |
  | `/vehicle/status/control_mode`           | autoware_auto_vehicle_msgs::msg::ControlModeReport      | control mode                                         |
  | `/vehicle/status/velocity_status`        | autoware_auto_vehicle_msgs::msg::VelocityReport         | velocity                                             |
  | `/vehicle/status/steering_status`        | autoware_auto_vehicle_msgs::msg::SteeringReport         | steering wheel angle                                 |
  | `/vehicle/status/gear_status`            | autoware_auto_vehicle_msgs::msg::GearReport             | gear status                                          |
  | `/vehicle/status/turn_indicators_status` | autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport   | turn indicators status                               |
  | `/vehicle/status/hazard_lights_status`   | autoware_auto_vehicle_msgs::msg::HazardLightsReport     | hazard lights status                                 |
  | `/vehicle/status/actuation_status`       | autoware_auto_vehicle_msgs::msg::ActuationStatusStamped | actuation (accel/brake pedal, steering wheel) status |

Apollo chassis

## ROS Parameters
like Apollo conf parameters

| Name                              | Type   | Description                                                                               |
| --------------------------------- | ------ | ----------------------------------------------------------------------------------------- |
| `base_frame_id`                   | string | frame id (assigned in hooke2 command, but it does not make sense)                         |
| `command_timeout_ms`              | double | timeout [ms]                                                                              |
| `loop_rate`                       | double | loop rate to publish commands                                                             |
| `steering_offset`                 | double | steering wheel angle offset                                                               |
| `enable_steering_rate_control`    | bool   | when enabled, max steering wheel rate is used for steering wheel angular velocity command |
| `emergency_brake`                 | double | brake pedal for emergency                                                                 |
| `vgr_coef_a`                      | double | coefficient to calculate steering wheel angle                                             |
| `vgr_coef_b`                      | double | coefficient to calculate steering wheel angle                                             |
| `vgr_coef_c`                      | double | coefficient to calculate steering wheel angle                                             |
| `accel_pedal_offset`              | double | accel pedal offset                                                                        |
| `brake_pedal_offset`              | double | brake pedal offset                                                                        |
| `max_throttle`                    | double | max accel pedal                                                                           |
| `max_brake`                       | double | max brake pedal                                                                           |
| `max_steering_wheel`              | double | max steering wheel angle                                                                  |
| `max_steering_wheel_rate`         | double | max steering wheel angular velocity                                                       |
| `min_steering_wheel_rate`         | double | min steering wheel angular velocity                                                       |
| `steering_wheel_rate_low_vel`     | double | min steering wheel angular velocity when velocity is low                                  |
| `steering_wheel_rate_low_stopped` | double | min steering wheel angular velocity when velocity is almost 0                             |
| `low_vel_thresh`                  | double | threshold velocity to decide the velocity is low for `steering_wheel_rate_low_vel`        |
| `hazard_thresh_time`              | double | threshold time to keep hazard lights                                                      |