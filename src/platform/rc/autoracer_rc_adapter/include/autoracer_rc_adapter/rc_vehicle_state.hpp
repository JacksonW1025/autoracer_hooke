#ifndef AUTORACER_RC_ADAPTER__RC_VEHICLE_STATE_HPP_
#define AUTORACER_RC_ADAPTER__RC_VEHICLE_STATE_HPP_

#include "autoracer_rc_adapter/rc_serial_protocol.hpp"

#include <cstdint>

namespace autoracer_rc_adapter
{

// The RC chassis has no physical gearbox. This is the platform-side direction
// contract requested through the standard Autoware gear topic.
enum class LogicalGear : std::uint8_t
{
  kPark,
  kNeutral,
  kDrive,
  kReverse,
};

// This deliberately contains only modes that can be distinguished from the
// frozen firmware telemetry. It is independent of the requested ROS mode.
enum class ReportedControlMode : std::uint8_t
{
  kNoCommand,
  kAutonomous,
  kManual,
};

ReportedControlMode reported_control_mode(
  const ChassisFeedback & feedback) noexcept;

bool has_latched_chassis_fault(
  const ChassisFeedback & feedback) noexcept;

bool has_live_chassis_fault(
  const ChassisFeedback & feedback) noexcept;

// A clear is permitted only after a fresh enabled zero stream has made the
// firmware's automatic path observable and all current fault/override sources
// have disappeared. The caller separately proves that such a zero was sent.
bool fault_clear_is_permitted(
  const ChassisFeedback & feedback) noexcept;

bool steering_estimate_is_valid(
  const ChassisFeedback & feedback) noexcept;

enum class HallFeedbackDecision : std::uint8_t
{
  kNotRequired,
  kAcquiring,
  kValid,
  kFault,
};

enum class VelocityFeedbackState : std::uint8_t
{
  kUnavailable,
  kMeasuredMotion,
  kConfirmedStandstill,
  kInconsistent,
};

VelocityFeedbackState velocity_feedback_state(
  const ChassisFeedback & feedback) noexcept;

bool velocity_report_is_publishable(
  const ChassisFeedback & feedback,
  HallFeedbackDecision hall_decision) noexcept;

bool logical_gear_allows_speed(
  LogicalGear gear, double speed_mps) noexcept;

bool manual_stop_is_required(
  bool autonomous_requested, bool has_transmitted_command,
  bool stop_already_sent) noexcept;

class EmergencyStatusMonitor
{
public:
  explicit EmergencyStatusMonitor(std::int64_t timeout_ms);

  void observe(bool active, std::int64_t monotonic_ms) noexcept;

  bool fresh_and_clear(std::int64_t monotonic_ms) const noexcept;

  bool active() const noexcept;

private:
  std::int64_t timeout_ms_;
  std::int64_t last_observation_ms_{0};
  bool received_{false};
  bool active_{false};
};

class HallFeedbackMonitor
{
public:
  HallFeedbackMonitor(
    std::int64_t acquisition_timeout_ms,
    std::int64_t loss_timeout_ms);

  void note_effective_command(
    double speed_mps, std::int64_t monotonic_ms) noexcept;

  HallFeedbackDecision observe(
    bool feedback_valid, std::int64_t monotonic_ms) noexcept;

  void reset() noexcept;

private:
  std::int64_t acquisition_timeout_ms_;
  std::int64_t loss_timeout_ms_;
  std::int64_t motion_started_ms_{0};
  std::int64_t loss_started_ms_{0};
  int command_direction_{0};
  bool motion_expected_{false};
  bool feedback_acquired_{false};
  bool loss_active_{false};
};

} // namespace autoracer_rc_adapter

#endif // AUTORACER_RC_ADAPTER__RC_VEHICLE_STATE_HPP_
