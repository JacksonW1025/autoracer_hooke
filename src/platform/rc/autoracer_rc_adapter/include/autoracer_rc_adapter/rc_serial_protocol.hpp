#ifndef AUTORACER_RC_ADAPTER__RC_SERIAL_PROTOCOL_HPP_
#define AUTORACER_RC_ADAPTER__RC_SERIAL_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace autoracer_rc_adapter
{

constexpr std::uint8_t kRcFrameHead = 0x7BU;
constexpr std::uint8_t kRcFrameTail = 0x7DU;
constexpr std::uint8_t kRcAckermannCommandId = 0x01U;
constexpr std::uint8_t kRcCommandFlagEnable = 0x01U;
constexpr std::uint8_t kRcCommandFlagBrake = 0x02U;
constexpr std::uint8_t kRcCommandFlagClearFault = 0x04U;
constexpr std::uint8_t kRcCommandFlagSoftwareStop = 0x80U;
constexpr std::uint8_t kRcTelemetryProtocolId = 0xA1U;
constexpr std::size_t kRcCommandFrameSize = 11U;
constexpr std::size_t kRcFeedbackFrameSize = 24U;

constexpr std::uint8_t kRcTelemetryFlagAutoEnabled = 0x01U;
constexpr std::uint8_t kRcTelemetryFlagRcOverrideActive = 0x02U;
constexpr std::uint8_t kRcTelemetryFlagStopOverrideActive = 0x04U;
constexpr std::uint8_t kRcTelemetryFlagCommandTimeout = 0x08U;
constexpr std::uint8_t kRcTelemetryFlagBrakeActive = 0x10U;
constexpr std::uint8_t kRcTelemetryFlagFaultLatched = 0x20U;
constexpr std::uint8_t kRcTelemetryFlagSteeringIsMeasured = 0x40U;

constexpr std::uint32_t kRcStatusFaultLatched = 1UL << 0U;
constexpr std::uint32_t kRcStatusCommandTimeout = 1UL << 1U;
constexpr std::uint32_t kRcStatusRcOverrideActive = 1UL << 2U;
constexpr std::uint32_t kRcStatusStopOverrideActive = 1UL << 3U;
constexpr std::uint32_t kRcStatusBrakeActive = 1UL << 4U;
constexpr std::uint32_t kRcStatusAutoEnabled = 1UL << 5U;
constexpr std::uint32_t kRcStatusHallFeedbackValid = 1UL << 6U;
constexpr std::uint32_t kRcStatusHallFault = 1UL << 7U;
constexpr std::uint32_t kRcStatusSteeringEstimateValid = 1UL << 8U;
constexpr std::uint32_t kRcStatusSteeringIsMeasured = 1UL << 9U;
constexpr std::uint32_t kRcStatusRcInputFault = 1UL << 10U;
constexpr std::uint32_t kRcStatusBatteryValid = 1UL << 11U;
// Hall no-pulse standstill inference while the current control direction is
// zero. This is not an independent physical stop sensor.
constexpr std::uint32_t kRcStatusHallStandstillConfirmed = 1UL << 12U;
constexpr std::uint32_t kRcStatusSpeedSaturated = 1UL << 14U;
constexpr std::uint32_t kRcStatusSteeringSaturated = 1UL << 15U;
// Firmware speed-target slew activity, not measured or controlled vehicle
// acceleration.
constexpr std::uint32_t kRcStatusAccelerationLimited = 1UL << 16U;
constexpr std::uint32_t kRcStatusSteeringRateLimited = 1UL << 17U;
constexpr std::uint32_t kRcStatusFrameErrorSeen = 1UL << 18U;

struct ChassisCommand
{
  double speed_mps{0.0};
  double steering_tire_angle_rad{0.0};
  // This is the firmware's software command-enable bit, not a physical drive
  // enable. The minimal ROS bridge does not expose the firmware brake bit.
  bool enable{true};
  // This requests firmware neutral outputs; it is not a physical emergency stop.
  bool software_stop{false};
  // Internal diagnostic handshake only. It is never mapped from a ROS motion
  // command and is valid only as a disabled, zero-valued standalone frame.
  bool clear_fault{false};
};

struct ChassisFeedback
{
  // Compact firmware state; it is not a ROS control-mode report.
  std::uint8_t status_flags{0U};
  std::uint8_t sequence{0U};
  // Hall count and speed magnitudes are measured; their signs come from the
  // current automatic/RC direction or the last direction retained while the
  // wheel coasts. status_bits distinguishes measured motion, confirmed Hall
  // silence, and unavailable feedback.
  std::int32_t hall_delta_count_command_signed{0};
  double hall_speed_command_signed_mps{0.0};
  // Steering comes from output PWM calibration, not a steering sensor.
  double steering_angle_estimate_rad{0.0};
  // Yaw rate is a Hall-speed/PWM-steering kinematic estimate, not a gyro.
  double yaw_rate_estimate_rad_s{0.0};
  // Battery is converted from the chassis ADC.
  std::uint16_t battery_mv{0U};
  std::uint16_t dt_ms{0U};
  std::uint32_t status_bits{0U};
  std::uint8_t protocol_id{0U};
};

struct FeedbackDecoderStats
{
  std::uint64_t valid_frames{0U};
  std::uint64_t bcc_errors{0U};
  std::uint64_t tail_errors{0U};
  std::uint64_t protocol_errors{0U};
  std::uint64_t discarded_bytes{0U};
};

std::uint8_t calculate_bcc(const std::uint8_t * data, std::size_t size) noexcept;

std::array<std::uint8_t, kRcCommandFrameSize>
encode_command_frame(const ChassisCommand & command);

std::optional<ChassisFeedback> decode_feedback_frame(
  const std::array<std::uint8_t, kRcFeedbackFrameSize> & frame) noexcept;

class FeedbackStreamDecoder
{
public:
  std::vector<ChassisFeedback> append(
    const std::uint8_t * data,
    std::size_t size);

  const FeedbackDecoderStats & stats() const noexcept;

  void discard_buffer() noexcept;

private:
  std::vector<std::uint8_t> buffer_;
  FeedbackDecoderStats stats_;
};

} // namespace autoracer_rc_adapter

#endif // AUTORACER_RC_ADAPTER__RC_SERIAL_PROTOCOL_HPP_
