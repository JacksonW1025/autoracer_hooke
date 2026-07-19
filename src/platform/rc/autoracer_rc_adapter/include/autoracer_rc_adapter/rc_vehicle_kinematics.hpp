#ifndef AUTORACER_RC_ADAPTER__RC_VEHICLE_KINEMATICS_HPP_
#define AUTORACER_RC_ADAPTER__RC_VEHICLE_KINEMATICS_HPP_

namespace autoracer_rc_adapter
{

struct VehicleCommandLimits
{
  double maximum_command_speed_mps{3.0};
  double minimum_command_speed_mps{0.3};
  double max_steering_tire_angle_rad{0.349};
};

struct ChassisMotionSetpoint
{
  double speed_mps{0.0};
  double steering_tire_angle_rad{0.0};
  bool speed_saturated{false};
  bool steering_saturated{false};
  bool speed_below_minimum{false};
};

ChassisMotionSetpoint
control_to_chassis_motion(
  double longitudinal_velocity_mps,
  double steering_tire_angle_rad,
  const VehicleCommandLimits & limits);

} // namespace autoracer_rc_adapter

#endif // AUTORACER_RC_ADAPTER__RC_VEHICLE_KINEMATICS_HPP_
