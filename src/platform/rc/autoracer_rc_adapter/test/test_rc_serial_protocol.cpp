#include "autoracer_rc_adapter/rc_serial_protocol.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace autoracer_rc_adapter
{
namespace
{

void put_i16(
  std::array<std::uint8_t, kRcFeedbackFrameSize> & frame,
  const std::size_t offset, const std::int16_t value)
{
  const auto raw = static_cast<std::uint16_t>(value);
  frame[offset] = static_cast<std::uint8_t>((raw >> 8U) & 0xFFU);
  frame[offset + 1U] = static_cast<std::uint8_t>(raw & 0xFFU);
}

void put_i32(
  std::array<std::uint8_t, kRcFeedbackFrameSize> & frame,
  const std::size_t offset, const std::int32_t value)
{
  const auto raw = static_cast<std::uint32_t>(value);
  frame[offset] = static_cast<std::uint8_t>((raw >> 24U) & 0xFFU);
  frame[offset + 1U] = static_cast<std::uint8_t>((raw >> 16U) & 0xFFU);
  frame[offset + 2U] = static_cast<std::uint8_t>((raw >> 8U) & 0xFFU);
  frame[offset + 3U] = static_cast<std::uint8_t>(raw & 0xFFU);
}

void put_u16(
  std::array<std::uint8_t, kRcFeedbackFrameSize> & frame,
  const std::size_t offset, const std::uint16_t value)
{
  frame[offset] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
  frame[offset + 1U] = static_cast<std::uint8_t>(value & 0xFFU);
}

void put_u32(
  std::array<std::uint8_t, kRcFeedbackFrameSize> & frame,
  const std::size_t offset, const std::uint32_t value)
{
  frame[offset] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
  frame[offset + 1U] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
  frame[offset + 2U] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
  frame[offset + 3U] = static_cast<std::uint8_t>(value & 0xFFU);
}

std::array<std::uint8_t, kRcFeedbackFrameSize>
make_feedback_frame(
  const std::int16_t speed_mmps = -500,
  const std::int16_t steering_mrad = 262,
  const std::int16_t yaw_rate_mradps = -100,
  const std::int32_t hall_delta_count = -3,
  const std::uint16_t battery_mv = 12150U,
  const std::uint16_t dt_ms = 50U,
  const std::uint32_t status_bits = 0x00000144UL,
  const std::uint8_t status_flags = 0x02U,
  const std::uint8_t sequence = 0x5AU)
{
  std::array<std::uint8_t, kRcFeedbackFrameSize> frame{};
  frame[0] = kRcFrameHead;
  frame[1] = status_flags;
  frame[2] = sequence;
  put_i32(frame, 3U, hall_delta_count);
  put_i16(frame, 7U, speed_mmps);
  put_i16(frame, 9U, steering_mrad);
  put_i16(frame, 11U, yaw_rate_mradps);
  put_u16(frame, 13U, battery_mv);
  put_u16(frame, 15U, dt_ms);
  put_u32(frame, 17U, status_bits);
  frame[21] = kRcTelemetryProtocolId;
  frame[22] = calculate_bcc(frame.data(), 22U);
  frame[23] = kRcFrameTail;
  return frame;
}

TEST(RcSerialProtocol, EncodesExactAckermannAndSoftwareStopFrames) {
  ChassisCommand command;
  command.speed_mps = 0.500;
  command.steering_tire_angle_rad = 0.262;
  const std::array<std::uint8_t, kRcCommandFrameSize> expected{
    0x7BU, 0x01U, 0x01U, 0x01U, 0xF4U, 0x01U,
    0x06U, 0x00U, 0x00U, 0x89U, 0x7DU};
  EXPECT_EQ(encode_command_frame(command), expected);

  command.speed_mps = -3.0;
  command.steering_tire_angle_rad = -0.262;
  const std::array<std::uint8_t, kRcCommandFrameSize> expected_reverse{
    0x7BU, 0x01U, 0x01U, 0xF4U, 0x48U, 0xFEU,
    0xFAU, 0x00U, 0x00U, 0xC3U, 0x7DU};
  EXPECT_EQ(encode_command_frame(command), expected_reverse);

  command.enable = false;
  command.software_stop = true;
  const std::array<std::uint8_t, kRcCommandFrameSize> expected_stop{
    0x7BU, 0x01U, 0x80U, 0x00U, 0x00U, 0x00U,
    0x00U, 0x00U, 0x00U, 0xFAU, 0x7DU};
  EXPECT_EQ(encode_command_frame(command), expected_stop);
}

TEST(RcSerialProtocol, EncodesActiveZeroAndSignedBigEndianFields) {
  ChassisCommand command;
  const std::array<std::uint8_t, kRcCommandFrameSize> expected_zero{
    0x7BU, 0x01U, 0x01U, 0x00U, 0x00U, 0x00U,
    0x00U, 0x00U, 0x00U, 0x7BU, 0x7DU};
  EXPECT_EQ(encode_command_frame(command), expected_zero);

  command.speed_mps = -0.100;
  command.steering_tire_angle_rad = -0.025;
  const auto frame = encode_command_frame(command);
  EXPECT_EQ(frame[1], kRcAckermannCommandId);
  EXPECT_EQ(frame[2], kRcCommandFlagEnable);
  EXPECT_EQ(frame[3], 0xFFU);
  EXPECT_EQ(frame[4], 0x9CU);
  EXPECT_EQ(frame[5], 0xFFU);
  EXPECT_EQ(frame[6], 0xE7U);
  EXPECT_EQ(frame[7], 0x00U);
  EXPECT_EQ(frame[8], 0x00U);
  EXPECT_EQ(frame[9], calculate_bcc(frame.data(), 9U));
  EXPECT_EQ(frame[10], kRcFrameTail);
}

TEST(RcSerialProtocol, EncodesOnlyTheExactFirmwareClearFaultFrame) {
  ChassisCommand command;
  command.enable = false;
  command.clear_fault = true;
  const std::array<std::uint8_t, kRcCommandFrameSize> expected_clear{
    0x7BU, 0x01U, 0x04U, 0x00U, 0x00U, 0x00U,
    0x00U, 0x00U, 0x00U, 0x7EU, 0x7DU};
  EXPECT_EQ(encode_command_frame(command), expected_clear);

  command.enable = true;
  EXPECT_THROW((void)encode_command_frame(command), std::invalid_argument);

  command.enable = false;
  command.speed_mps = 0.001;
  EXPECT_THROW((void)encode_command_frame(command), std::invalid_argument);

  command.speed_mps = 0.0;
  command.steering_tire_angle_rad = -0.001;
  EXPECT_THROW((void)encode_command_frame(command), std::invalid_argument);

  command.steering_tire_angle_rad = 0.0;
  command.software_stop = true;
  EXPECT_THROW((void)encode_command_frame(command), std::invalid_argument);
}

TEST(RcSerialProtocol, ExposesExactFirmwareFlagAndStatusAssignments) {
  EXPECT_EQ(kRcCommandFlagEnable, 0x01U);
  EXPECT_EQ(kRcCommandFlagBrake, 0x02U);
  EXPECT_EQ(kRcCommandFlagClearFault, 0x04U);
  EXPECT_EQ(kRcCommandFlagSoftwareStop, 0x80U);

  EXPECT_EQ(kRcTelemetryFlagAutoEnabled, 0x01U);
  EXPECT_EQ(kRcTelemetryFlagRcOverrideActive, 0x02U);
  EXPECT_EQ(kRcTelemetryFlagStopOverrideActive, 0x04U);
  EXPECT_EQ(kRcTelemetryFlagCommandTimeout, 0x08U);
  EXPECT_EQ(kRcTelemetryFlagBrakeActive, 0x10U);
  EXPECT_EQ(kRcTelemetryFlagFaultLatched, 0x20U);
  EXPECT_EQ(kRcTelemetryFlagSteeringIsMeasured, 0x40U);

  EXPECT_EQ(kRcStatusFaultLatched, 1UL << 0U);
  EXPECT_EQ(kRcStatusCommandTimeout, 1UL << 1U);
  EXPECT_EQ(kRcStatusRcOverrideActive, 1UL << 2U);
  EXPECT_EQ(kRcStatusStopOverrideActive, 1UL << 3U);
  EXPECT_EQ(kRcStatusBrakeActive, 1UL << 4U);
  EXPECT_EQ(kRcStatusAutoEnabled, 1UL << 5U);
  EXPECT_EQ(kRcStatusHallFeedbackValid, 1UL << 6U);
  EXPECT_EQ(kRcStatusHallFault, 1UL << 7U);
  EXPECT_EQ(kRcStatusSteeringEstimateValid, 1UL << 8U);
  EXPECT_EQ(kRcStatusSteeringIsMeasured, 1UL << 9U);
  EXPECT_EQ(kRcStatusRcInputFault, 1UL << 10U);
  EXPECT_EQ(kRcStatusBatteryValid, 1UL << 11U);
  EXPECT_EQ(kRcStatusHallStandstillConfirmed, 1UL << 12U);
  EXPECT_EQ(kRcStatusSpeedSaturated, 1UL << 14U);
  EXPECT_EQ(kRcStatusSteeringSaturated, 1UL << 15U);
  EXPECT_EQ(kRcStatusAccelerationLimited, 1UL << 16U);
  EXPECT_EQ(kRcStatusSteeringRateLimited, 1UL << 17U);
  EXPECT_EQ(kRcStatusFrameErrorSeen, 1UL << 18U);
}

TEST(RcSerialProtocol, EnforcesWireRangeAndFiniteValues) {
  ChassisCommand command;
  command.speed_mps = 32.767;
  auto frame = encode_command_frame(command);
  EXPECT_EQ(frame[3], 0x7FU);
  EXPECT_EQ(frame[4], 0xFFU);

  command.speed_mps = -32.768;
  frame = encode_command_frame(command);
  EXPECT_EQ(frame[3], 0x80U);
  EXPECT_EQ(frame[4], 0x00U);

  command.speed_mps = 32.768;
  EXPECT_THROW((void)encode_command_frame(command), std::out_of_range);
  command.speed_mps = -32.769;
  EXPECT_THROW((void)encode_command_frame(command), std::out_of_range);
  command.speed_mps = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((void)encode_command_frame(command), std::invalid_argument);

  command.speed_mps = 0.0;
  command.steering_tire_angle_rad = std::numeric_limits<double>::infinity();
  EXPECT_THROW((void)encode_command_frame(command), std::invalid_argument);

  command.steering_tire_angle_rad = 32.767;
  frame = encode_command_frame(command);
  EXPECT_EQ(frame[5], 0x7FU);
  EXPECT_EQ(frame[6], 0xFFU);
  command.steering_tire_angle_rad = -32.768;
  frame = encode_command_frame(command);
  EXPECT_EQ(frame[5], 0x80U);
  EXPECT_EQ(frame[6], 0x00U);
  command.steering_tire_angle_rad = 32.768;
  EXPECT_THROW((void)encode_command_frame(command), std::out_of_range);
}

TEST(RcSerialProtocol, DecodesExactTelemetryLayoutSourcesAndUnits) {
  const std::array<std::uint8_t, kRcFeedbackFrameSize> golden{
    0x7BU, 0x02U, 0x5AU, 0xFFU, 0xFFU, 0xFFU, 0xFDU, 0xFEU,
    0x0CU, 0x01U, 0x06U, 0xFFU, 0x9CU, 0x2FU, 0x76U, 0x00U,
    0x32U, 0x00U, 0x00U, 0x01U, 0x44U, 0xA1U, 0x38U, 0x7DU};
  const auto feedback = decode_feedback_frame(golden);
  ASSERT_TRUE(feedback.has_value());
  EXPECT_EQ(feedback->status_flags, 0x02U);
  EXPECT_NE(
    feedback->status_flags & kRcTelemetryFlagRcOverrideActive,
    0U);
  EXPECT_EQ(feedback->sequence, 0x5AU);
  EXPECT_EQ(feedback->hall_delta_count_command_signed, -3);
  EXPECT_DOUBLE_EQ(feedback->hall_speed_command_signed_mps, -0.500);
  EXPECT_DOUBLE_EQ(feedback->steering_angle_estimate_rad, 0.262);
  EXPECT_DOUBLE_EQ(feedback->yaw_rate_estimate_rad_s, -0.100);
  EXPECT_EQ(feedback->battery_mv, 12150U);
  EXPECT_EQ(feedback->dt_ms, 50U);
  EXPECT_EQ(feedback->status_bits, 0x00000144UL);
  EXPECT_NE(feedback->status_bits & kRcStatusRcOverrideActive, 0U);
  EXPECT_NE(feedback->status_bits & kRcStatusHallFeedbackValid, 0U);
  EXPECT_NE(feedback->status_bits & kRcStatusSteeringEstimateValid, 0U);
  EXPECT_EQ(feedback->status_bits & kRcStatusSteeringIsMeasured, 0U);
  EXPECT_EQ(feedback->protocol_id, kRcTelemetryProtocolId);
}

TEST(RcSerialProtocol, DecodesHallConfirmedStandstillWithoutChangingLayout) {
  const auto frame = make_feedback_frame(
    0, 0, 0, 0, 12150U, 50U,
    kRcStatusHallStandstillConfirmed | kRcStatusSteeringEstimateValid,
    0U, 0x5BU);
  const auto feedback = decode_feedback_frame(frame);
  ASSERT_TRUE(feedback.has_value());
  EXPECT_DOUBLE_EQ(feedback->hall_speed_command_signed_mps, 0.0);
  EXPECT_NE(
    feedback->status_bits & kRcStatusHallStandstillConfirmed,
    0U);
  EXPECT_EQ(feedback->status_bits & kRcStatusHallFeedbackValid, 0U);
  EXPECT_EQ(feedback->protocol_id, kRcTelemetryProtocolId);
}

TEST(RcSerialProtocol, RejectsInvalidFeedbackFrames) {
  auto bad_header = make_feedback_frame();
  bad_header[0] = 0x00U;
  EXPECT_FALSE(decode_feedback_frame(bad_header).has_value());

  auto bad_tail = make_feedback_frame();
  bad_tail[23] = 0x00U;
  EXPECT_FALSE(decode_feedback_frame(bad_tail).has_value());

  auto bad_bcc = make_feedback_frame();
  bad_bcc[22] ^= 0x01U;
  EXPECT_FALSE(decode_feedback_frame(bad_bcc).has_value());

  auto wrong_protocol = make_feedback_frame();
  wrong_protocol[21] = 0xA0U;
  wrong_protocol[22] = calculate_bcc(wrong_protocol.data(), 22U);
  EXPECT_FALSE(decode_feedback_frame(wrong_protocol).has_value());
}

TEST(RcSerialProtocol, ReassemblesEveryTwoPartSplit) {
  const auto frame = make_feedback_frame();
  for (std::size_t split = 0U; split <= frame.size(); ++split) {
    FeedbackStreamDecoder decoder;
    const auto first = decoder.append(frame.data(), split);
    EXPECT_EQ(first.size(), split == frame.size() ? 1U : 0U)
      << "split=" << split;
    const auto second =
      decoder.append(frame.data() + split, frame.size() - split);
    const std::size_t total_frames = first.size() + second.size();
    EXPECT_EQ(total_frames, 1U) << "split=" << split;
    EXPECT_EQ(decoder.stats().valid_frames, 1U) << "split=" << split;
  }
}

TEST(RcSerialProtocol, HandlesConcatenatedFramesAndLeadingNoise) {
  const auto first_frame =
    make_feedback_frame(100, 10, 20, 7, 12000U, 49U, 0x60U, 1U, 9U);
  const auto second_frame =
    make_feedback_frame(-200, -10, -20, -8, 11900U, 51U, 0x44U, 2U, 10U);
  std::vector<std::uint8_t> bytes{0x00U, 0x55U, 0x7AU};
  bytes.insert(bytes.end(), first_frame.begin(), first_frame.end());
  bytes.insert(bytes.end(), second_frame.begin(), second_frame.end());

  FeedbackStreamDecoder decoder;
  const auto feedback = decoder.append(bytes.data(), bytes.size());
  ASSERT_EQ(feedback.size(), 2U);
  EXPECT_DOUBLE_EQ(feedback[0].hall_speed_command_signed_mps, 0.1);
  EXPECT_DOUBLE_EQ(feedback[1].hall_speed_command_signed_mps, -0.2);
  EXPECT_EQ(feedback[0].hall_delta_count_command_signed, 7);
  EXPECT_EQ(feedback[1].hall_delta_count_command_signed, -8);
  EXPECT_EQ(decoder.stats().valid_frames, 2U);
  EXPECT_EQ(decoder.stats().discarded_bytes, 3U);
}

TEST(RcSerialProtocol, RecoversAfterTailBccAndProtocolErrors) {
  const auto valid = make_feedback_frame();
  auto bad_tail = valid;
  bad_tail[23] = 0x00U;
  auto bad_bcc = valid;
  bad_bcc[22] ^= 0x80U;
  auto wrong_protocol = valid;
  wrong_protocol[21] = 0xA0U;
  wrong_protocol[22] = calculate_bcc(wrong_protocol.data(), 22U);

  std::vector<std::uint8_t> bytes;
  bytes.insert(bytes.end(), bad_tail.begin(), bad_tail.end());
  bytes.insert(bytes.end(), bad_bcc.begin(), bad_bcc.end());
  bytes.insert(bytes.end(), wrong_protocol.begin(), wrong_protocol.end());
  bytes.insert(bytes.end(), valid.begin(), valid.end());

  FeedbackStreamDecoder decoder;
  const auto feedback = decoder.append(bytes.data(), bytes.size());
  ASSERT_EQ(feedback.size(), 1U);
  EXPECT_EQ(decoder.stats().valid_frames, 1U);
  EXPECT_EQ(decoder.stats().tail_errors, 1U);
  EXPECT_EQ(decoder.stats().bcc_errors, 1U);
  EXPECT_EQ(decoder.stats().protocol_errors, 1U);
  EXPECT_GE(decoder.stats().discarded_bytes, 50U);
}

TEST(RcSerialProtocol, ResynchronizesToAHeaderInsideNoise) {
  const auto valid = make_feedback_frame(777, 0, 0, 12, 12345U);
  std::vector<std::uint8_t> bytes{kRcFrameHead, 0x01U, 0x02U, 0x03U,
    0x04U, 0x05U, 0x06U, 0x07U,
    0x08U, 0x09U, 0x0AU, kRcFrameHead};
  bytes.insert(bytes.end(), valid.begin() + 1, valid.end());

  FeedbackStreamDecoder decoder;
  const auto feedback = decoder.append(bytes.data(), bytes.size());
  ASSERT_EQ(feedback.size(), 1U);
  EXPECT_DOUBLE_EQ(feedback.front().hall_speed_command_signed_mps, 0.777);
  EXPECT_EQ(decoder.stats().tail_errors, 1U);
}

} // namespace
} // namespace autoracer_rc_adapter
