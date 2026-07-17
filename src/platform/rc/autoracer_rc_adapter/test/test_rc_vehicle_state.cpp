#include "autoracer_rc_adapter/rc_vehicle_state.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

namespace autoracer_rc_adapter
{
namespace
{

TEST(RcVehicleState, ReportsOnlyFirmwareObservableControlModes) {
  ChassisFeedback feedback;
  EXPECT_EQ(
    reported_control_mode(feedback),
    ReportedControlMode::kNoCommand);

  feedback.status_bits = kRcStatusAutoEnabled;
  EXPECT_EQ(
    reported_control_mode(feedback),
    ReportedControlMode::kAutonomous);

  feedback.status_bits = kRcStatusAutoEnabled | kRcStatusRcOverrideActive;
  EXPECT_EQ(
    reported_control_mode(feedback),
    ReportedControlMode::kManual);
}

TEST(RcVehicleState, DoesNotReportBlockedAutomaticCommandsAsAutonomous) {
  ChassisFeedback feedback;
  feedback.status_bits = kRcStatusAutoEnabled | kRcStatusCommandTimeout;
  EXPECT_EQ(
    reported_control_mode(feedback),
    ReportedControlMode::kNoCommand);

  feedback.status_bits = kRcStatusAutoEnabled | kRcStatusStopOverrideActive;
  EXPECT_EQ(
    reported_control_mode(feedback),
    ReportedControlMode::kNoCommand);

  feedback.status_bits = kRcStatusAutoEnabled | kRcStatusFaultLatched;
  EXPECT_EQ(
    reported_control_mode(feedback),
    ReportedControlMode::kNoCommand);

  feedback.status_bits = kRcStatusAutoEnabled | kRcStatusHallFault;
  EXPECT_EQ(
    reported_control_mode(feedback),
    ReportedControlMode::kNoCommand);

  feedback.status_bits = kRcStatusAutoEnabled | kRcStatusRcInputFault;
  EXPECT_EQ(
    reported_control_mode(feedback),
    ReportedControlMode::kNoCommand);

  feedback.status_bits = kRcStatusAutoEnabled | kRcStatusFrameErrorSeen;
  EXPECT_EQ(
    reported_control_mode(feedback),
    ReportedControlMode::kNoCommand);
}

TEST(RcVehicleState, SeparatesLiveFaultSourcesFromHistoricalLatch) {
  ChassisFeedback feedback;
  EXPECT_FALSE(has_latched_chassis_fault(feedback));
  EXPECT_FALSE(has_live_chassis_fault(feedback));

  feedback.status_bits = kRcStatusFaultLatched;
  EXPECT_TRUE(has_latched_chassis_fault(feedback));
  EXPECT_FALSE(has_live_chassis_fault(feedback));

  feedback.status_bits = kRcStatusHallFault;
  EXPECT_FALSE(has_latched_chassis_fault(feedback));
  EXPECT_FALSE(has_live_chassis_fault(feedback));

  feedback.status_bits = kRcStatusRcInputFault;
  EXPECT_TRUE(has_live_chassis_fault(feedback));

  feedback.status_bits = kRcStatusFrameErrorSeen;
  EXPECT_FALSE(has_live_chassis_fault(feedback));

  feedback.status_bits = kRcStatusFaultLatched | kRcStatusHallFault |
    kRcStatusRcInputFault | kRcStatusFrameErrorSeen;
  EXPECT_TRUE(has_latched_chassis_fault(feedback));
  EXPECT_TRUE(has_live_chassis_fault(feedback));
}

TEST(RcVehicleState, PermitsClearOnlyAfterZeroStreamOwnsTheChassis) {
  ChassisFeedback feedback;
  EXPECT_FALSE(fault_clear_is_permitted(feedback));

  feedback.status_bits = kRcStatusFaultLatched | kRcStatusAutoEnabled;
  EXPECT_TRUE(fault_clear_is_permitted(feedback));

  feedback.status_bits |= kRcStatusCommandTimeout;
  EXPECT_FALSE(fault_clear_is_permitted(feedback));

  feedback.status_bits = kRcStatusFaultLatched | kRcStatusAutoEnabled |
    kRcStatusRcOverrideActive;
  EXPECT_FALSE(fault_clear_is_permitted(feedback));

  feedback.status_bits = kRcStatusFaultLatched | kRcStatusAutoEnabled |
    kRcStatusStopOverrideActive;
  EXPECT_FALSE(fault_clear_is_permitted(feedback));

  feedback.status_bits = kRcStatusFaultLatched | kRcStatusAutoEnabled |
    kRcStatusHallFault;
  EXPECT_TRUE(fault_clear_is_permitted(feedback));

  feedback.status_bits = kRcStatusFaultLatched | kRcStatusAutoEnabled |
    kRcStatusRcInputFault;
  EXPECT_FALSE(fault_clear_is_permitted(feedback));

  feedback.status_bits = kRcStatusFaultLatched | kRcStatusAutoEnabled |
    kRcStatusFrameErrorSeen;
  EXPECT_TRUE(fault_clear_is_permitted(feedback));
}

TEST(RcVehicleState, ExposesSteeringOnlyWhenFirmwareMarksEstimateValid) {
  ChassisFeedback feedback;
  EXPECT_FALSE(steering_estimate_is_valid(feedback));

  feedback.status_bits = kRcStatusSteeringEstimateValid;
  EXPECT_TRUE(steering_estimate_is_valid(feedback));

  feedback.status_bits =
    kRcStatusSteeringEstimateValid | kRcStatusSteeringIsMeasured;
  EXPECT_TRUE(steering_estimate_is_valid(feedback));
}

TEST(RcVehicleState, ClassifiesOnlyMeasuredMotionAndConfirmedStandstillAsTruthful) {
  ChassisFeedback feedback;
  EXPECT_EQ(
    velocity_feedback_state(feedback),
    VelocityFeedbackState::kUnavailable);

  feedback.status_bits = kRcStatusHallFeedbackValid;
  EXPECT_EQ(
    velocity_feedback_state(feedback),
    VelocityFeedbackState::kInconsistent);

  feedback.hall_speed_command_signed_mps = 0.5;
  EXPECT_EQ(
    velocity_feedback_state(feedback),
    VelocityFeedbackState::kMeasuredMotion);

  feedback.status_bits = kRcStatusHallStandstillConfirmed;
  EXPECT_EQ(
    velocity_feedback_state(feedback),
    VelocityFeedbackState::kInconsistent);

  feedback.hall_speed_command_signed_mps = 0.0;
  EXPECT_EQ(
    velocity_feedback_state(feedback),
    VelocityFeedbackState::kConfirmedStandstill);

  feedback.status_bits = kRcStatusHallFeedbackValid |
    kRcStatusHallStandstillConfirmed;
  EXPECT_EQ(
    velocity_feedback_state(feedback),
    VelocityFeedbackState::kInconsistent);

  feedback.status_bits = 0U;
  feedback.hall_speed_command_signed_mps = -0.5;
  EXPECT_EQ(
    velocity_feedback_state(feedback),
    VelocityFeedbackState::kInconsistent);
}

TEST(RcVehicleState, PublishesVelocityOnlyFromTheMatchingHallState) {
  ChassisFeedback feedback;
  EXPECT_FALSE(velocity_report_is_publishable(
      feedback, HallFeedbackDecision::kNotRequired));

  feedback.status_bits = kRcStatusCommandTimeout;
  EXPECT_FALSE(velocity_report_is_publishable(
      feedback, HallFeedbackDecision::kNotRequired));

  feedback.status_bits = kRcStatusStopOverrideActive;
  EXPECT_FALSE(velocity_report_is_publishable(
      feedback, HallFeedbackDecision::kNotRequired));

  feedback.status_bits = kRcStatusHallFeedbackValid;
  feedback.hall_speed_command_signed_mps = -0.5;
  EXPECT_TRUE(velocity_report_is_publishable(
      feedback, HallFeedbackDecision::kValid));
  EXPECT_TRUE(velocity_report_is_publishable(
      feedback, HallFeedbackDecision::kNotRequired));

  feedback.status_bits = kRcStatusHallStandstillConfirmed;
  feedback.hall_speed_command_signed_mps = 0.0;
  EXPECT_TRUE(velocity_report_is_publishable(
      feedback, HallFeedbackDecision::kNotRequired));
  EXPECT_FALSE(velocity_report_is_publishable(
      feedback, HallFeedbackDecision::kAcquiring));
  EXPECT_FALSE(velocity_report_is_publishable(
      feedback, HallFeedbackDecision::kValid));
  EXPECT_FALSE(velocity_report_is_publishable(
      feedback, HallFeedbackDecision::kFault));
}

TEST(RcVehicleState, EnforcesLogicalDirectionWithoutInventingGearHardware) {
  EXPECT_TRUE(logical_gear_allows_speed(LogicalGear::kDrive, 0.5));
  EXPECT_TRUE(logical_gear_allows_speed(LogicalGear::kDrive, 0.0));
  EXPECT_FALSE(logical_gear_allows_speed(LogicalGear::kDrive, -0.5));

  EXPECT_TRUE(logical_gear_allows_speed(LogicalGear::kReverse, -0.5));
  EXPECT_TRUE(logical_gear_allows_speed(LogicalGear::kReverse, 0.0));
  EXPECT_FALSE(logical_gear_allows_speed(LogicalGear::kReverse, 0.5));

  EXPECT_TRUE(logical_gear_allows_speed(LogicalGear::kPark, 0.0));
  EXPECT_FALSE(logical_gear_allows_speed(LogicalGear::kPark, 0.5));
  EXPECT_FALSE(logical_gear_allows_speed(LogicalGear::kPark, -0.5));

  EXPECT_TRUE(logical_gear_allows_speed(LogicalGear::kNeutral, -0.0));
  EXPECT_FALSE(logical_gear_allows_speed(LogicalGear::kNeutral, 0.5));
  EXPECT_FALSE(logical_gear_allows_speed(LogicalGear::kNeutral, -0.5));
}

TEST(RcVehicleState, RequiresFreshClearEmergencyStatus) {
  EmergencyStatusMonitor monitor(250);
  EXPECT_FALSE(monitor.fresh_and_clear(0));

  monitor.observe(false, 1000);
  EXPECT_FALSE(monitor.active());
  EXPECT_TRUE(monitor.fresh_and_clear(1000));
  EXPECT_TRUE(monitor.fresh_and_clear(1250));
  EXPECT_FALSE(monitor.fresh_and_clear(1251));
  EXPECT_FALSE(monitor.fresh_and_clear(999));

  monitor.observe(true, 1300);
  EXPECT_TRUE(monitor.active());
  EXPECT_FALSE(monitor.fresh_and_clear(1300));

  monitor.observe(false, 1400);
  EXPECT_FALSE(monitor.active());
  EXPECT_TRUE(monitor.fresh_and_clear(1400));
  EXPECT_THROW((void)EmergencyStatusMonitor(0), std::invalid_argument);
}

TEST(RcVehicleState, GivesInitialHallAcquisitionADeadline) {
  HallFeedbackMonitor monitor(1500, 250);
  EXPECT_EQ(
    monitor.observe(false, 0),
    HallFeedbackDecision::kNotRequired);

  monitor.note_effective_command(0.5, 100);
  EXPECT_EQ(
    monitor.observe(false, 100),
    HallFeedbackDecision::kAcquiring);
  EXPECT_EQ(
    monitor.observe(false, 1599),
    HallFeedbackDecision::kAcquiring);
  EXPECT_EQ(
    monitor.observe(false, 1600),
    HallFeedbackDecision::kFault);
}

TEST(RcVehicleState, FailsClosedWhenAcquiredHallFeedbackIsLost) {
  HallFeedbackMonitor monitor(1500, 250);
  monitor.note_effective_command(-0.5, 100);
  EXPECT_EQ(
    monitor.observe(true, 800),
    HallFeedbackDecision::kValid);
  EXPECT_EQ(
    monitor.observe(false, 850),
    HallFeedbackDecision::kAcquiring);
  EXPECT_EQ(
    monitor.observe(true, 900),
    HallFeedbackDecision::kValid);
  EXPECT_EQ(
    monitor.observe(false, 950),
    HallFeedbackDecision::kAcquiring);
  EXPECT_EQ(
    monitor.observe(false, 1199),
    HallFeedbackDecision::kAcquiring);
  EXPECT_EQ(
    monitor.observe(false, 1200),
    HallFeedbackDecision::kFault);

  monitor.note_effective_command(0.0, 900);
  EXPECT_EQ(
    monitor.observe(false, 5000),
    HallFeedbackDecision::kNotRequired);

  monitor.note_effective_command(0.5, 5100);
  monitor.note_effective_command(-0.5, 5200);
  EXPECT_EQ(
    monitor.observe(false, 6699),
    HallFeedbackDecision::kAcquiring);
  EXPECT_EQ(
    monitor.observe(false, 6700),
    HallFeedbackDecision::kFault);

  monitor.reset();
  EXPECT_EQ(
    monitor.observe(false, 9000),
    HallFeedbackDecision::kNotRequired);
  EXPECT_THROW((void)HallFeedbackMonitor(-1, 250), std::invalid_argument);
  EXPECT_THROW((void)HallFeedbackMonitor(1500, 0), std::invalid_argument);
}

TEST(RcVehicleState, SendsManualStopOnlyForAStillUnstoppedCommandPath) {
  EXPECT_FALSE(manual_stop_is_required(false, false, false));
  EXPECT_TRUE(manual_stop_is_required(true, false, false));
  EXPECT_TRUE(manual_stop_is_required(false, true, false));
  EXPECT_TRUE(manual_stop_is_required(true, true, false));
  EXPECT_FALSE(manual_stop_is_required(true, true, true));
}

} // namespace
} // namespace autoracer_rc_adapter
