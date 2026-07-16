#include "autoracer_rc_adapter/rc_serial_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace autoracer_rc_adapter
{
namespace
{

std::int16_t quantize_milli(const double value, const char * const field_name)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string(field_name) + " must be finite");
  }

  const double scaled = value * 1000.0;
  constexpr double kMinimum =
    static_cast<double>(std::numeric_limits<std::int16_t>::min());
  constexpr double kMaximum =
    static_cast<double>(std::numeric_limits<std::int16_t>::max());
  constexpr double kFloatingPointTolerance = 1.0e-9;
  if (scaled < kMinimum - kFloatingPointTolerance ||
    scaled > kMaximum + kFloatingPointTolerance)
  {
    throw std::out_of_range(
            std::string(field_name) +
            " exceeds the signed 16-bit wire range");
  }

  return static_cast<std::int16_t>(std::lround(scaled));
}

void encode_i16_be(
  const std::int16_t value, std::uint8_t & high_byte,
  std::uint8_t & low_byte) noexcept
{
  const auto raw = static_cast<std::uint16_t>(value);
  high_byte = static_cast<std::uint8_t>((raw >> 8U) & 0xFFU);
  low_byte = static_cast<std::uint8_t>(raw & 0xFFU);
}

std::int16_t decode_i16_be(
  const std::uint8_t high_byte,
  const std::uint8_t low_byte) noexcept
{
  const std::uint16_t raw = static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(high_byte) << 8U) | low_byte);
  const std::int32_t signed_value =
    raw >= 0x8000U ? static_cast<std::int32_t>(raw) - 0x10000 :
    static_cast<std::int32_t>(raw);
  return static_cast<std::int16_t>(signed_value);
}

std::int32_t decode_i32_be(const std::uint8_t * const data) noexcept
{
  const std::uint32_t raw = (static_cast<std::uint32_t>(data[0]) << 24U) |
    (static_cast<std::uint32_t>(data[1]) << 16U) |
    (static_cast<std::uint32_t>(data[2]) << 8U) |
    static_cast<std::uint32_t>(data[3]);
  if (raw >= 0x80000000UL) {
    return static_cast<std::int32_t>(static_cast<std::int64_t>(raw) -
           0x100000000LL);
  }
  return static_cast<std::int32_t>(raw);
}

std::uint16_t decode_u16_be(const std::uint8_t * const data) noexcept
{
  return static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(data[0]) << 8U) | data[1]);
}

std::uint32_t decode_u32_be(const std::uint8_t * const data) noexcept
{
  return (static_cast<std::uint32_t>(data[0]) << 24U) |
         (static_cast<std::uint32_t>(data[1]) << 16U) |
         (static_cast<std::uint32_t>(data[2]) << 8U) |
         static_cast<std::uint32_t>(data[3]);
}

} // namespace

std::uint8_t calculate_bcc(
  const std::uint8_t * const data,
  const std::size_t size) noexcept
{
  std::uint8_t bcc = 0U;
  for (std::size_t index = 0U; index < size; ++index) {
    bcc ^= data[index];
  }
  return bcc;
}

std::array<std::uint8_t, kRcCommandFrameSize>
encode_command_frame(const ChassisCommand & command)
{
  double speed_mps = command.speed_mps;
  double steering_tire_angle_rad = command.steering_tire_angle_rad;

  if (!command.enable || command.software_stop) {
    speed_mps = 0.0;
    steering_tire_angle_rad = 0.0;
  }

  const std::int16_t speed_raw = quantize_milli(speed_mps, "speed");
  const std::int16_t steering_raw =
    quantize_milli(steering_tire_angle_rad, "steering");

  std::array<std::uint8_t, kRcCommandFrameSize> frame{};
  frame[0] = kRcFrameHead;
  frame[1] = kRcAckermannCommandId;
  if (command.enable) {
    frame[2] |= kRcCommandFlagEnable;
  }
  if (command.software_stop) {
    frame[2] |= kRcCommandFlagSoftwareStop;
  }
  encode_i16_be(speed_raw, frame[3], frame[4]);
  encode_i16_be(steering_raw, frame[5], frame[6]);
  // The chassis cannot execute Control acceleration or jerk. These protocol
  // bytes remain reserved rather than pretending to carry either field.
  frame[7] = 0U;
  frame[8] = 0U;
  frame[9] = calculate_bcc(frame.data(), 9U);
  frame[10] = kRcFrameTail;
  return frame;
}

std::optional<ChassisFeedback> decode_feedback_frame(
  const std::array<std::uint8_t, kRcFeedbackFrameSize> & frame) noexcept
{
  if (frame.front() != kRcFrameHead || frame.back() != kRcFrameTail ||
    frame[22] != calculate_bcc(frame.data(), 22U) ||
    frame[21] != kRcTelemetryProtocolId)
  {
    return std::nullopt;
  }

  ChassisFeedback feedback;
  feedback.status_flags = frame[1];
  feedback.sequence = frame[2];
  feedback.hall_delta_count_command_signed = decode_i32_be(&frame[3]);
  feedback.hall_speed_command_signed_mps =
    static_cast<double>(decode_i16_be(frame[7], frame[8])) / 1000.0;
  feedback.steering_angle_estimate_rad =
    static_cast<double>(decode_i16_be(frame[9], frame[10])) / 1000.0;
  feedback.yaw_rate_estimate_rad_s =
    static_cast<double>(decode_i16_be(frame[11], frame[12])) / 1000.0;
  feedback.battery_mv = decode_u16_be(&frame[13]);
  feedback.dt_ms = decode_u16_be(&frame[15]);
  feedback.status_bits = decode_u32_be(&frame[17]);
  feedback.protocol_id = frame[21];
  return feedback;
}

std::vector<ChassisFeedback>
FeedbackStreamDecoder::append(
  const std::uint8_t * const data,
  const std::size_t size)
{
  if (size > 0U) {
    buffer_.insert(buffer_.end(), data, data + size);
  }

  std::vector<ChassisFeedback> decoded;
  for (;; ) {
    const auto header = std::find(buffer_.begin(), buffer_.end(), kRcFrameHead);
    if (header == buffer_.end()) {
      stats_.discarded_bytes += buffer_.size();
      buffer_.clear();
      break;
    }

    if (header != buffer_.begin()) {
      const auto discarded =
        static_cast<std::size_t>(std::distance(buffer_.begin(), header));
      stats_.discarded_bytes += discarded;
      buffer_.erase(buffer_.begin(), header);
    }

    if (buffer_.size() < kRcFeedbackFrameSize) {
      break;
    }

    if (buffer_[kRcFeedbackFrameSize - 1U] != kRcFrameTail) {
      ++stats_.tail_errors;
      ++stats_.discarded_bytes;
      buffer_.erase(buffer_.begin());
      continue;
    }

    if (buffer_[22] != calculate_bcc(buffer_.data(), 22U)) {
      ++stats_.bcc_errors;
      ++stats_.discarded_bytes;
      buffer_.erase(buffer_.begin());
      continue;
    }

    if (buffer_[21] != kRcTelemetryProtocolId) {
      ++stats_.protocol_errors;
      stats_.discarded_bytes += kRcFeedbackFrameSize;
      buffer_.erase(
        buffer_.begin(),
        buffer_.begin() +
        static_cast<std::ptrdiff_t>(kRcFeedbackFrameSize));
      continue;
    }

    std::array<std::uint8_t, kRcFeedbackFrameSize> frame{};
    std::copy_n(buffer_.begin(), frame.size(), frame.begin());
    const auto feedback = decode_feedback_frame(frame);
    if (feedback.has_value()) {
      decoded.push_back(*feedback);
      ++stats_.valid_frames;
    }
    buffer_.erase(
      buffer_.begin(),
      buffer_.begin() + static_cast<std::ptrdiff_t>(frame.size()));
  }
  return decoded;
}

const FeedbackDecoderStats & FeedbackStreamDecoder::stats() const noexcept
{
  return stats_;
}

void FeedbackStreamDecoder::discard_buffer() noexcept
{
  stats_.discarded_bytes += buffer_.size();
  buffer_.clear();
}

} // namespace autoracer_rc_adapter
