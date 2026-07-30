#include "autoracer_rc_adapter/rc_vehicle_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace autoracer_rc_adapter
{

ChassisMotionSetpoint
control_to_chassis_motion(
  const double longitudinal_velocity_mps,
  const double steering_tire_angle_rad,
  const VehicleCommandLimits & limits)
{
  if (!std::isfinite(longitudinal_velocity_mps) ||
    !std::isfinite(steering_tire_angle_rad))
  {
    throw std::invalid_argument(
            "control velocity and steering angle must be finite");
  }
  if (!std::isfinite(limits.maximum_command_speed_mps) ||
    limits.maximum_command_speed_mps <= 0.0 ||
    !std::isfinite(limits.minimum_command_speed_mps) ||
    limits.minimum_command_speed_mps < 0.0 ||
    limits.minimum_command_speed_mps > limits.maximum_command_speed_mps ||
    !std::isfinite(limits.max_steering_tire_angle_rad) ||
    limits.max_steering_tire_angle_rad <= 0.0)
  {
    throw std::invalid_argument("vehicle command limits are invalid");
  }

  ChassisMotionSetpoint setpoint;
  setpoint.speed_mps =
    std::clamp(
    longitudinal_velocity_mps, -limits.maximum_command_speed_mps,
    limits.maximum_command_speed_mps);
  setpoint.steering_tire_angle_rad =
    std::clamp(
    steering_tire_angle_rad, -limits.max_steering_tire_angle_rad,
    limits.max_steering_tire_angle_rad);
  setpoint.speed_saturated = setpoint.speed_mps != longitudinal_velocity_mps;
  setpoint.steering_saturated =
    setpoint.steering_tire_angle_rad != steering_tire_angle_rad;

  if (setpoint.speed_mps != 0.0 &&
    std::abs(setpoint.speed_mps) < limits.minimum_command_speed_mps)
  {
    setpoint.speed_mps = 0.0;
    setpoint.speed_below_minimum = true;
  }

  return setpoint;
}

} // namespace autoracer_rc_adapter
