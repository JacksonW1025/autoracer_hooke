#include "autoracer_rc_adapter/rc_vehicle_state.hpp"

#include <stdexcept>

namespace autoracer_rc_adapter
{

bool has_latched_chassis_fault(
  const ChassisFeedback & feedback) noexcept
{
  return (feedback.status_bits & kRcStatusFaultLatched) != 0U;
}

bool has_live_chassis_fault(
  const ChassisFeedback & feedback) noexcept
{
  constexpr std::uint32_t kLiveFaultMask = kRcStatusRcInputFault;
  return (feedback.status_bits & kLiveFaultMask) != 0U;
}

bool fault_clear_is_permitted(
  const ChassisFeedback & feedback) noexcept
{
  constexpr std::uint32_t kClearBlocked =
    kRcStatusCommandTimeout | kRcStatusRcOverrideActive |
    kRcStatusStopOverrideActive;
  // HALL_FAULT and FRAME_ERROR_SEEN are deliberately not live sources here.
  // The firmware latches those historical diagnostics until this exact clear
  // handshake is received.
  return has_latched_chassis_fault(feedback) &&
         !has_live_chassis_fault(feedback) &&
         (feedback.status_bits & kRcStatusAutoEnabled) != 0U &&
         (feedback.status_bits & kClearBlocked) == 0U;
}

ReportedControlMode reported_control_mode(
  const ChassisFeedback & feedback) noexcept
{
  if ((feedback.status_bits & kRcStatusRcOverrideActive) != 0U) {
    return ReportedControlMode::kManual;
  }

  constexpr std::uint32_t kAutomaticCommandBlocked =
    kRcStatusCommandTimeout | kRcStatusStopOverrideActive |
    kRcStatusFaultLatched | kRcStatusHallFault |
    kRcStatusRcInputFault | kRcStatusFrameErrorSeen;
  if ((feedback.status_bits & kRcStatusAutoEnabled) != 0U &&
    (feedback.status_bits & kAutomaticCommandBlocked) == 0U)
  {
    return ReportedControlMode::kAutonomous;
  }

  return ReportedControlMode::kNoCommand;
}

bool steering_estimate_is_valid(
  const ChassisFeedback & feedback) noexcept
{
  return (feedback.status_bits & kRcStatusSteeringEstimateValid) != 0U;
}

bool velocity_report_is_publishable(
  const ChassisFeedback & feedback,
  const HallFeedbackDecision hall_decision) noexcept
{
  switch (velocity_feedback_state(feedback)) {
    case VelocityFeedbackState::kMeasuredMotion:
      return true;
    case VelocityFeedbackState::kConfirmedStandstill:
      return hall_decision == HallFeedbackDecision::kNotRequired;
    case VelocityFeedbackState::kUnavailable:
    case VelocityFeedbackState::kInconsistent:
      return false;
  }
  return false;
}

VelocityFeedbackState velocity_feedback_state(
  const ChassisFeedback & feedback) noexcept
{
  const bool measured_motion =
    (feedback.status_bits & kRcStatusHallFeedbackValid) != 0U;
  const bool confirmed_standstill =
    (feedback.status_bits & kRcStatusHallStandstillConfirmed) != 0U;
  const bool wire_speed_is_zero =
    feedback.hall_speed_command_signed_mps == 0.0;

  if (measured_motion && !confirmed_standstill && !wire_speed_is_zero) {
    return VelocityFeedbackState::kMeasuredMotion;
  }
  if (!measured_motion && confirmed_standstill && wire_speed_is_zero) {
    return VelocityFeedbackState::kConfirmedStandstill;
  }
  if (!measured_motion && !confirmed_standstill && wire_speed_is_zero) {
    return VelocityFeedbackState::kUnavailable;
  }
  return VelocityFeedbackState::kInconsistent;
}

bool logical_gear_allows_speed(
  const LogicalGear gear, const double speed_mps) noexcept
{
  switch (gear) {
    case LogicalGear::kDrive:
      return speed_mps >= 0.0;
    case LogicalGear::kReverse:
      return speed_mps <= 0.0;
    case LogicalGear::kPark:
    case LogicalGear::kNeutral:
      return speed_mps == 0.0;
  }
  return false;
}

bool manual_stop_is_required(
  const bool autonomous_requested, const bool has_transmitted_command,
  const bool stop_already_sent) noexcept
{
  return !stop_already_sent &&
         (autonomous_requested || has_transmitted_command);
}

EmergencyStatusMonitor::EmergencyStatusMonitor(const std::int64_t timeout_ms)
: timeout_ms_(timeout_ms)
{
  if (timeout_ms_ <= 0) {
    throw std::invalid_argument("emergency status timeout must be positive");
  }
}

void EmergencyStatusMonitor::observe(
  const bool active, const std::int64_t monotonic_ms) noexcept
{
  last_observation_ms_ = monotonic_ms;
  received_ = true;
  active_ = active;
}

bool EmergencyStatusMonitor::fresh_and_clear(
  const std::int64_t monotonic_ms) const noexcept
{
  if (!received_ || active_ || monotonic_ms < last_observation_ms_) {
    return false;
  }
  return monotonic_ms - last_observation_ms_ <= timeout_ms_;
}

bool EmergencyStatusMonitor::active() const noexcept
{
  return active_;
}

HallFeedbackMonitor::HallFeedbackMonitor(
  const std::int64_t acquisition_timeout_ms,
  const std::int64_t loss_timeout_ms)
: acquisition_timeout_ms_(acquisition_timeout_ms),
  loss_timeout_ms_(loss_timeout_ms)
{
  if (acquisition_timeout_ms_ <= 0 || loss_timeout_ms_ <= 0) {
    throw std::invalid_argument(
            "Hall feedback acquisition and loss timeouts must be positive");
  }
}

void HallFeedbackMonitor::note_effective_command(
  const double speed_mps, const std::int64_t monotonic_ms) noexcept
{
  const int direction = speed_mps > 0.0 ? 1 : (speed_mps < 0.0 ? -1 : 0);
  if (direction == 0) {
    reset();
    return;
  }
  if (!motion_expected_ || direction != command_direction_) {
    motion_started_ms_ = monotonic_ms;
    command_direction_ = direction;
    motion_expected_ = true;
    feedback_acquired_ = false;
    loss_active_ = false;
  }
}

HallFeedbackDecision HallFeedbackMonitor::observe(
  const bool feedback_valid, const std::int64_t monotonic_ms) noexcept
{
  if (!motion_expected_) {
    return HallFeedbackDecision::kNotRequired;
  }
  if (feedback_valid) {
    feedback_acquired_ = true;
    loss_active_ = false;
    return HallFeedbackDecision::kValid;
  }
  if (monotonic_ms < motion_started_ms_) {
    return HallFeedbackDecision::kFault;
  }
  if (!feedback_acquired_) {
    return monotonic_ms - motion_started_ms_ >= acquisition_timeout_ms_ ?
           HallFeedbackDecision::kFault : HallFeedbackDecision::kAcquiring;
  }
  if (!loss_active_) {
    loss_started_ms_ = monotonic_ms;
    loss_active_ = true;
    return HallFeedbackDecision::kAcquiring;
  }
  if (monotonic_ms < loss_started_ms_ ||
    monotonic_ms - loss_started_ms_ >= loss_timeout_ms_)
  {
    return HallFeedbackDecision::kFault;
  }
  return HallFeedbackDecision::kAcquiring;
}

void HallFeedbackMonitor::reset() noexcept
{
  motion_started_ms_ = 0;
  command_direction_ = 0;
  motion_expected_ = false;
  feedback_acquired_ = false;
  loss_started_ms_ = 0;
  loss_active_ = false;
}

} // namespace autoracer_rc_adapter
