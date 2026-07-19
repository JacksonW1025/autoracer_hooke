#include "autoracer_rc_adapter/rc_serial_protocol.hpp"
#include "autoracer_rc_adapter/rc_vehicle_kinematics.hpp"
#include "autoracer_rc_adapter/rc_vehicle_state.hpp"

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <limits>
#include <memory>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>

#include <array>

namespace autoracer_rc_adapter
{
namespace
{

constexpr std::int64_t kExpectedBaudRate = 115200;
constexpr std::int64_t kExpectedFirmwareCommandTimeoutMs = 250;
constexpr std::int64_t kFutureStampToleranceNs = 50'000'000;
constexpr auto kSerialWriteTimeout = std::chrono::milliseconds(20);
constexpr auto kStatsLogPeriod = std::chrono::seconds(5);

std::int64_t steady_now_ms() noexcept
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

speed_t baud_to_termios(const std::int64_t baud_rate)
{
  if (baud_rate != kExpectedBaudRate) {
    throw std::invalid_argument("locked firmware requires baud_rate=115200");
  }
  return B115200;
}

} // namespace

class RcVehicleInterfaceNode final : public rclcpp::Node
{
public:
  RcVehicleInterfaceNode()
  : Node("rc_vehicle_interface")
  {
    serial_port_ = declare_parameter<std::string>(
      "serial_port",
      "/dev/autoracer_rc_chassis");
    baud_rate_ =
      declare_parameter<std::int64_t>("baud_rate", kExpectedBaudRate);
    command_limits_.maximum_command_speed_mps =
      declare_parameter<double>("maximum_command_speed_mps", 3.0);
    command_limits_.minimum_command_speed_mps =
      declare_parameter<double>("minimum_command_speed_mps", 0.3);
    command_limits_.max_steering_tire_angle_rad =
      declare_parameter<double>("max_steering_tire_angle_rad", 0.349);
    firmware_command_timeout_ms_ = declare_parameter<std::int64_t>(
      "firmware_command_timeout_ms", kExpectedFirmwareCommandTimeoutMs);
    emergency_status_timeout_ms_ = declare_parameter<std::int64_t>(
      "emergency_status_timeout_ms", 250);
    hall_feedback_acquisition_timeout_ms_ = declare_parameter<std::int64_t>(
      "hall_feedback_acquisition_timeout_ms", 1500);
    hall_feedback_loss_timeout_ms_ = declare_parameter<std::int64_t>(
      "hall_feedback_loss_timeout_ms", 250);
    serial_poll_period_ms_ =
      declare_parameter<std::int64_t>("serial_poll_period_ms", 10);
    reconnect_period_ms_ =
      declare_parameter<std::int64_t>("reconnect_period_ms", 1000);
    base_frame_id_ =
      declare_parameter<std::string>("base_frame_id", "base_link");

    validate_parameters();
    emergency_status_monitor_ =
      EmergencyStatusMonitor(emergency_status_timeout_ms_);
    hall_feedback_monitor_ =
      HallFeedbackMonitor(
      hall_feedback_acquisition_timeout_ms_, hall_feedback_loss_timeout_ms_);

    const auto command_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    command_subscription_ =
      create_subscription<autoware_control_msgs::msg::Control>(
      "/control/command/control_cmd", command_qos,
      std::bind(
        &RcVehicleInterfaceNode::on_control_command, this,
        std::placeholders::_1));
    gear_subscription_ =
      create_subscription<autoware_vehicle_msgs::msg::GearCommand>(
      "/control/command/gear_cmd", command_qos,
      std::bind(
        &RcVehicleInterfaceNode::on_gear_command, this,
        std::placeholders::_1));
    emergency_subscription_ =
      create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
      "/control/command/emergency_cmd", command_qos,
      std::bind(
        &RcVehicleInterfaceNode::on_emergency_status, this,
        std::placeholders::_1));
    velocity_publisher_ =
      create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
      "/vehicle/status/velocity_status", command_qos);
    steering_publisher_ =
      create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(
      "/vehicle/status/steering_status", command_qos);
    gear_publisher_ =
      create_publisher<autoware_vehicle_msgs::msg::GearReport>(
      "/vehicle/status/gear_status", command_qos);
    control_mode_publisher_ =
      create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>(
      "/vehicle/status/control_mode", command_qos);
    control_mode_service_ =
      create_service<autoware_vehicle_msgs::srv::ControlModeCommand>(
      "/control/control_mode_request",
      std::bind(
        &RcVehicleInterfaceNode::on_control_mode_request, this,
        std::placeholders::_1, std::placeholders::_2));

    next_reconnect_attempt_ = std::chrono::steady_clock::now();
    last_stats_log_ = std::chrono::steady_clock::now();
    serial_timer_ = create_wall_timer(
      std::chrono::milliseconds(serial_poll_period_ms_),
      std::bind(&RcVehicleInterfaceNode::on_serial_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "RC vehicle interface configured for %s at 115200 8N1; it "
      "does not write merely because the serial port opened; active motion "
      "requires a fresh clear emergency status, accepted AUTONOMOUS mode "
      "and a fresh Control message; a formal emergency or safety fault "
      "sends one firmware software-stop frame; Control velocity/steering map "
      "directly to firmware speed/steering; Control acceleration and "
      "jerk are unsupported by this chassis and intentionally ignored; "
      "gear is a platform-side direction contract, not chassis hardware; "
      "a historical firmware fault latch is cleared only by a one-shot, "
      "zero-valued diagnostic handshake before motion is released",
      serial_port_.c_str());
  }

  ~RcVehicleInterfaceNode() override
  {
    if (serial_fd_ >= 0 && has_transmitted_command_) {
      (void)transmit_software_stop("orderly shutdown");
    }
    if (serial_fd_ >= 0) {
      (void)::close(serial_fd_);
      serial_fd_ = -1;
    }
  }

private:
  void validate_parameters()
  {
    if (serial_port_.empty()) {
      throw std::invalid_argument("serial_port must not be empty");
    }
    (void)baud_to_termios(baud_rate_);
    if (firmware_command_timeout_ms_ != kExpectedFirmwareCommandTimeoutMs) {
      throw std::invalid_argument(
              "locked firmware requires firmware_command_timeout_ms=250");
    }
    if (serial_poll_period_ms_ <= 0 || reconnect_period_ms_ <= 0) {
      throw std::invalid_argument(
              "serial poll and reconnect periods must be positive");
    }
    if (emergency_status_timeout_ms_ <= 0 ||
      hall_feedback_acquisition_timeout_ms_ <= 0 ||
      hall_feedback_loss_timeout_ms_ <= 0)
    {
      throw std::invalid_argument(
              "emergency and Hall feedback timeouts must be positive");
    }
    if (base_frame_id_.empty()) {
      throw std::invalid_argument("base_frame_id must not be empty");
    }

    // Validate the product command boundaries without fabricating a motion
    // command.
    (void)control_to_chassis_motion(0.0, 0.0, command_limits_);
  }

  bool configure_serial(const int file_descriptor) const
  {
    termios settings{};
    if (::tcgetattr(file_descriptor, &settings) != 0) {
      RCLCPP_ERROR(
        get_logger(), "tcgetattr failed for %s: %s",
        serial_port_.c_str(), std::strerror(errno));
      return false;
    }

    ::cfmakeraw(&settings);
    const speed_t speed = baud_to_termios(baud_rate_);
    if (::cfsetispeed(&settings, speed) != 0 ||
      ::cfsetospeed(&settings, speed) != 0)
    {
      RCLCPP_ERROR(
        get_logger(), "failed to configure baud rate for %s: %s",
        serial_port_.c_str(), std::strerror(errno));
      return false;
    }

    settings.c_cflag &=
      static_cast<tcflag_t>(~(CSIZE | CSTOPB | PARENB | CRTSCTS));
    settings.c_cflag |= static_cast<tcflag_t>(CS8 | CLOCAL | CREAD);
    settings.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY));
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    if (::tcsetattr(file_descriptor, TCSANOW, &settings) != 0) {
      RCLCPP_ERROR(
        get_logger(), "tcsetattr failed for %s: %s",
        serial_port_.c_str(), std::strerror(errno));
      return false;
    }
    return true;
  }

  void try_open_serial()
  {
    if (serial_fd_ >= 0) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now < next_reconnect_attempt_) {
      return;
    }
    next_reconnect_attempt_ =
      now + std::chrono::milliseconds(reconnect_period_ms_);

    const int candidate = ::open(
      serial_port_.c_str(),
      O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (candidate < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "cannot open %s: %s", serial_port_.c_str(),
        std::strerror(errno));
      return;
    }
    if (!configure_serial(candidate)) {
      (void)::close(candidate);
      return;
    }

    feedback_decoder_.discard_buffer();
    serial_fd_ = candidate;
    ++connection_count_;
    RCLCPP_INFO(
      get_logger(),
      "opened %s in raw 115200 8N1 mode (connection=%llu)",
      serial_port_.c_str(),
      static_cast<unsigned long long>(connection_count_));
  }

  void close_serial(const char * const reason)
  {
    if (serial_fd_ < 0) {
      return;
    }
    (void)::close(serial_fd_);
    serial_fd_ = -1;
    autonomous_requested_ = false;
    has_transmitted_command_ = false;
    safety_stop_sent_ = false;
    rc_override_active_ = false;
    has_received_feedback_ = false;
    reset_fault_clear_handshake();
    hall_feedback_monitor_.reset();
    feedback_decoder_.discard_buffer();
    ++disconnect_count_;
    next_reconnect_attempt_ = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(reconnect_period_ms_);
    RCLCPP_WARN(
      get_logger(), "closed %s after %s (disconnects=%llu)",
      serial_port_.c_str(), reason,
      static_cast<unsigned long long>(disconnect_count_));
  }

  bool write_frame(const std::array<std::uint8_t, kRcCommandFrameSize> & frame)
  {
    if (serial_fd_ < 0) {
      return false;
    }

    std::size_t offset = 0U;
    const auto deadline =
      std::chrono::steady_clock::now() + kSerialWriteTimeout;
    while (offset < frame.size()) {
      const ssize_t result =
        ::write(serial_fd_, frame.data() + offset, frame.size() - offset);
      if (result > 0) {
        offset += static_cast<std::size_t>(result);
        continue;
      }
      if (result < 0 && errno == EINTR) {
        continue;
      }
      if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        const auto remaining =
          std::chrono::duration_cast<std::chrono::milliseconds>(
          deadline - std::chrono::steady_clock::now());
        if (remaining.count() <= 0) {
          ++write_error_count_;
          close_serial("serial write timeout");
          return false;
        }
        pollfd descriptor{};
        descriptor.fd = serial_fd_;
        descriptor.events = POLLOUT;
        const int poll_result =
          ::poll(&descriptor, 1, static_cast<int>(remaining.count()));
        if (poll_result > 0 && (descriptor.revents & POLLOUT) != 0) {
          continue;
        }
        ++write_error_count_;
        close_serial(
          poll_result == 0 ? "serial write timeout" :
          "serial write poll error");
        return false;
      }

      ++write_error_count_;
      close_serial("serial write error");
      return false;
    }

    ++transmitted_frame_count_;
    return true;
  }

  bool transmit_software_stop(const char * const context)
  {
    ChassisCommand software_stop;
    software_stop.enable = false;
    software_stop.software_stop = true;
    try {
      const auto frame = encode_command_frame(software_stop);
      if (!write_frame(frame)) {
        return false;
      }
      has_transmitted_command_ = false;
      RCLCPP_INFO(
        get_logger(),
        "sent one firmware software-stop frame for %s; this is not a "
        "physical emergency stop",
        context);
      return true;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "could not encode software-stop frame for %s: %s",
        context, error.what());
      return false;
    }
  }

  bool transmit_clear_fault()
  {
    ChassisCommand clear_fault;
    clear_fault.enable = false;
    clear_fault.clear_fault = true;
    try {
      const auto frame = encode_command_frame(clear_fault);
      if (!write_frame(frame)) {
        return false;
      }
      ++fault_clear_frame_count_;
      RCLCPP_INFO(
        get_logger(),
        "sent one disabled zero-valued firmware clear-fault frame after "
        "fresh automatic zero-command ownership was confirmed");
      return true;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "could not encode clear-fault frame: %s",
        error.what());
      return false;
    }
  }

  void reset_fault_clear_handshake() noexcept
  {
    fault_clear_pending_ = false;
    fault_clear_zero_sent_ = false;
    fault_clear_sent_ = false;
  }

  void latch_safety_stop(const char * const reason)
  {
    const bool newly_latched = !safety_stop_latched_;
    safety_stop_latched_ = true;
    autonomous_requested_ = false;
    reset_fault_clear_handshake();
    hall_feedback_monitor_.reset();

    if (newly_latched) {
      RCLCPP_ERROR(
        get_logger(),
        "latched RC software safety stop after %s; normal Control is blocked "
        "until a fresh clear emergency status and a new AUTONOMOUS request; "
        "this is not a physical emergency stop",
        reason);
    }
    if (serial_fd_ >= 0 && !safety_stop_sent_) {
      safety_stop_sent_ = transmit_software_stop(reason);
    }
  }

  bool
  command_stamp_is_fresh(
    const builtin_interfaces::msg::Time & message_stamp,
    std::int64_t & stamp_ns)
  {
    const rclcpp::Time now = get_clock()->now();
    const rclcpp::Time stamp(message_stamp, get_clock()->get_clock_type());
    stamp_ns = stamp.nanoseconds();
    const std::int64_t age_ns = now.nanoseconds() - stamp_ns;
    const std::int64_t maximum_age_ns =
      firmware_command_timeout_ms_ * 1'000'000;

    if (age_ns < -kFutureStampToleranceNs) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "rejected Control timestamp %.3f ms in the future",
        static_cast<double>(-age_ns) / 1.0e6);
      return false;
    }
    if (age_ns > maximum_age_ns) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "rejected stale Control command aged %.3f ms",
        static_cast<double>(age_ns) / 1.0e6);
      return false;
    }
    if (stamp_ns < last_transmitted_command_stamp_ns_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "rejected out-of-order Control timestamp");
      return false;
    }
    return true;
  }

  void on_gear_command(
    const autoware_vehicle_msgs::msg::GearCommand::SharedPtr message)
  {
    using GearCommand = autoware_vehicle_msgs::msg::GearCommand;
    using GearReport = autoware_vehicle_msgs::msg::GearReport;

    LogicalGear requested_gear;
    std::uint8_t requested_report;
    switch (message->command) {
      case GearCommand::PARK:
        requested_gear = LogicalGear::kPark;
        requested_report = GearReport::PARK;
        break;
      case GearCommand::NEUTRAL:
        requested_gear = LogicalGear::kNeutral;
        requested_report = GearReport::NEUTRAL;
        break;
      case GearCommand::DRIVE:
        requested_gear = LogicalGear::kDrive;
        requested_report = GearReport::DRIVE;
        break;
      case GearCommand::REVERSE:
        requested_gear = LogicalGear::kReverse;
        requested_report = GearReport::REVERSE;
        break;
      default:
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "ignored unsupported logical gear command=%u; RC supports only "
          "PARK, NEUTRAL, DRIVE and REVERSE direction contracts",
          static_cast<unsigned int>(message->command));
        return;
    }

    if (requested_report != logical_gear_report_) {
      RCLCPP_INFO(
        get_logger(),
        "accepted logical gear/direction command=%u; the RC chassis has no "
        "physical gearbox",
        static_cast<unsigned int>(message->command));
    }
    logical_gear_ = requested_gear;
    logical_gear_report_ = requested_report;
  }

  void on_emergency_status(
    const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::SharedPtr message)
  {
    emergency_status_monitor_.observe(message->emergency, steady_now_ms());
    if (message->emergency) {
      latch_safety_stop("formal emergency status");
    }
  }

  void on_control_mode_request(
    const autoware_vehicle_msgs::srv::ControlModeCommand::Request::SharedPtr
    request,
    const autoware_vehicle_msgs::srv::ControlModeCommand::Response::SharedPtr
    response)
  {
    using ControlModeCommand =
      autoware_vehicle_msgs::srv::ControlModeCommand;

    if (request->mode == ControlModeCommand::Request::AUTONOMOUS) {
      if (serial_fd_ < 0) {
        response->success = false;
        RCLCPP_WARN(
          get_logger(),
          "rejected AUTONOMOUS mode because the chassis serial port is "
          "disconnected");
        return;
      }
      if (!emergency_status_monitor_.fresh_and_clear(steady_now_ms())) {
        response->success = false;
        RCLCPP_WARN(
          get_logger(),
          "rejected AUTONOMOUS mode because the formal emergency status is "
          "active, missing, or older than %lld ms",
          static_cast<long long>(emergency_status_timeout_ms_));
        return;
      }
      if (!has_received_feedback_) {
        response->success = false;
        RCLCPP_WARN(
          get_logger(),
          "rejected AUTONOMOUS mode until the first valid chassis feedback "
          "frame is received");
        return;
      }

      ChassisFeedback latest_feedback;
      latest_feedback.status_bits = last_status_bits_;
      if (has_live_chassis_fault(latest_feedback)) {
        response->success = false;
        RCLCPP_ERROR(
          get_logger(),
          "rejected AUTONOMOUS mode because firmware status_bits=0x%08x "
          "contains a persistent RC-input fault",
          static_cast<unsigned int>(last_status_bits_));
        return;
      }

      safety_stop_latched_ = false;
      safety_stop_sent_ = false;
      hall_feedback_monitor_.reset();
      autonomous_requested_ = true;
      if (has_latched_chassis_fault(latest_feedback)) {
        if (!fault_clear_pending_) {
          fault_clear_pending_ = true;
          fault_clear_zero_sent_ = false;
          fault_clear_sent_ = false;
          RCLCPP_WARN(
            get_logger(),
            "accepted AUTONOMOUS authorization for a zero-only diagnostic "
            "clear handshake; nonzero Control remains blocked until firmware "
            "telemetry confirms the historical fault latch is clear");
        }
      } else {
        reset_fault_clear_handshake();
      }
      response->success = true;
      RCLCPP_INFO(
        get_logger(),
        "accepted AUTONOMOUS mode request; firmware mode remains feedback-"
        "derived and will change only after a fresh Control frame");
      return;
    }

    if (request->mode == ControlModeCommand::Request::MANUAL ||
      request->mode == ControlModeCommand::Request::NO_COMMAND)
    {
      const bool stop_required = manual_stop_is_required(
        autonomous_requested_, has_transmitted_command_, safety_stop_sent_);
      autonomous_requested_ = false;
      reset_fault_clear_handshake();
      hall_feedback_monitor_.reset();
      if (serial_fd_ < 0 || !stop_required) {
        response->success = true;
      } else {
        safety_stop_sent_ =
          transmit_software_stop("MANUAL/NO_COMMAND mode request");
        response->success = safety_stop_sent_;
      }
      RCLCPP_INFO(
        get_logger(),
        "accepted MANUAL/NO_COMMAND request=%s; stop_frame=%s; automatic "
        "Control forwarding is disabled",
        response->success ? "true" : "false",
        stop_required ? (safety_stop_sent_ ? "sent" : "failed") :
        "not_required");
      return;
    }

    response->success = false;
    RCLCPP_WARN(
      get_logger(),
      "rejected unsupported partial control mode=%u; RC exposes only full "
      "AUTONOMOUS and MANUAL/NO_COMMAND",
      static_cast<unsigned int>(request->mode));
  }

  void on_control_command(
    const autoware_control_msgs::msg::Control::SharedPtr message)
  {
    if (!autonomous_requested_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "discarded Control command because AUTONOMOUS mode has not been "
        "accepted");
      return;
    }
    if (!emergency_status_monitor_.fresh_and_clear(steady_now_ms())) {
      latch_safety_stop(
        "active, missing, or stale formal emergency status");
      return;
    }
    if (serial_fd_ < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "discarded Control command because the chassis "
        "serial port is disconnected");
      return;
    }

    std::int64_t stamp_ns = 0;
    if (!command_stamp_is_fresh(message->stamp, stamp_ns)) {
      return;
    }

    try {
      const ChassisMotionSetpoint setpoint = control_to_chassis_motion(
        static_cast<double>(message->longitudinal.velocity),
        static_cast<double>(message->lateral.steering_tire_angle),
        command_limits_);
      if (setpoint.speed_saturated || setpoint.steering_saturated) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "clamped Control command to confirmed boundaries: "
          "speed=%.3f m/s steering=%.3f rad",
          setpoint.speed_mps,
          setpoint.steering_tire_angle_rad);
      }
      if (setpoint.speed_below_minimum) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "mapped |Control velocity| below 0.300 m/s to "
          "zero; steering command remains %.3f rad",
          setpoint.steering_tire_angle_rad);
      }
      ChassisCommand command;
      if (fault_clear_pending_) {
        if (setpoint.speed_mps != 0.0 ||
          setpoint.steering_tire_angle_rad != 0.0)
        {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "blocked nonzero Control during firmware fault-clear handshake; "
            "sending enabled zero speed and zero steering instead");
        }
        command.speed_mps = 0.0;
        command.steering_tire_angle_rad = 0.0;
      } else {
        command.speed_mps = logical_gear_allows_speed(
          logical_gear_, setpoint.speed_mps) ? setpoint.speed_mps : 0.0;
        if (command.speed_mps != setpoint.speed_mps) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "mapped Control speed %.3f m/s to zero because logical "
            "gear/direction report=%u does not permit its sign",
            setpoint.speed_mps,
            static_cast<unsigned int>(logical_gear_report_));
        }
        command.steering_tire_angle_rad = setpoint.steering_tire_angle_rad;
      }
      command.enable = true;
      command.software_stop = false;
      const auto frame = encode_command_frame(command);
      if (write_frame(frame)) {
        last_transmitted_command_stamp_ns_ = stamp_ns;
        has_transmitted_command_ = true;
        if (fault_clear_pending_) {
          fault_clear_zero_sent_ = true;
          hall_feedback_monitor_.reset();
        } else if (rc_override_active_) {
          hall_feedback_monitor_.reset();
        } else {
          hall_feedback_monitor_.note_effective_command(
            command.speed_mps, steady_now_ms());
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "rejected invalid Control command: %s",
        error.what());
    }
  }

  void publish_feedback(const ChassisFeedback & feedback)
  {
    has_received_feedback_ = true;
    const VelocityFeedbackState velocity_state =
      velocity_feedback_state(feedback);
    rc_override_active_ =
      (feedback.status_bits & kRcStatusRcOverrideActive) != 0U;
    if (rc_override_active_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "firmware reports RC override active; standard control mode is MANUAL");
    }
    if ((feedback.status_bits & kRcStatusCommandTimeout) != 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "firmware reports its 250 ms automatic-command timeout state");
    }
    if ((feedback.status_bits & kRcStatusStopOverrideActive) != 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "firmware reports a software stop override; this is not a physical emergency stop");
    }
    if ((feedback.status_bits & kRcStatusFaultLatched) != 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "firmware reports a historical diagnostic fault latch; its live "
        "source may already have disappeared from status_bits");
    }
    if (has_live_chassis_fault(feedback)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "firmware reports a current chassis fault source in "
        "status_bits=0x%08x",
        static_cast<unsigned int>(feedback.status_bits));
    }
    if (velocity_state == VelocityFeedbackState::kUnavailable) {
      ++hall_feedback_invalid_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "firmware reports neither measured Hall motion nor Hall-confirmed "
        "standstill; VelocityReport is withheld");
    } else if (velocity_state == VelocityFeedbackState::kInconsistent) {
      ++hall_feedback_contract_error_count_;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "firmware Hall status and speed field are inconsistent; "
        "VelocityReport is withheld");
      if (autonomous_requested_ || has_transmitted_command_) {
        latch_safety_stop("inconsistent firmware Hall feedback contract");
      }
    }
    if ((feedback.status_bits & kRcStatusSteeringEstimateValid) == 0U) {
      ++steering_estimate_invalid_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "firmware marks its PWM-derived steering estimate invalid");
    }
    if ((feedback.status_bits & kRcStatusSteeringIsMeasured) != 0U) {
      ++unexpected_measured_steering_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "firmware claims measured steering, but the frozen "
        "vehicle contract has no steering "
        "angle sensor");
    }

    last_battery_mv_ = feedback.battery_mv;
    last_status_flags_ = feedback.status_flags;
    last_status_bits_ = feedback.status_bits;
    last_sequence_ = feedback.sequence;
    last_hall_delta_count_command_signed_ =
      feedback.hall_delta_count_command_signed;
    last_feedback_dt_ms_ = feedback.dt_ms;
    last_hall_speed_command_signed_mps_ =
      feedback.hall_speed_command_signed_mps;
    last_steering_estimate_rad_ = feedback.steering_angle_estimate_rad;
    last_yaw_rate_estimate_rad_s_ = feedback.yaw_rate_estimate_rad_s;

    const bool live_chassis_fault = has_live_chassis_fault(feedback);
    const bool latched_chassis_fault =
      has_latched_chassis_fault(feedback);
    if (live_chassis_fault &&
      (autonomous_requested_ || has_transmitted_command_))
    {
      latch_safety_stop("firmware live chassis fault");
    }

    if (!live_chassis_fault && fault_clear_pending_) {
      if (latched_chassis_fault) {
        if (fault_clear_zero_sent_ && !fault_clear_sent_ &&
          fault_clear_is_permitted(feedback))
        {
          fault_clear_sent_ = transmit_clear_fault();
        }
      } else if (
        reported_control_mode(feedback) ==
        ReportedControlMode::kAutonomous)
      {
        reset_fault_clear_handshake();
        RCLCPP_INFO(
          get_logger(),
          "firmware telemetry confirmed the historical fault latch is "
          "clear and automatic zero-command ownership is active; normal "
          "Control forwarding is released");
      }
    } else if (!live_chassis_fault && latched_chassis_fault &&
      autonomous_requested_)
    {
      latch_safety_stop(
        "a new firmware diagnostic fault latch during automatic control");
    }

    const bool firmware_command_blocked = rc_override_active_ ||
      (feedback.status_bits &
      (kRcStatusCommandTimeout | kRcStatusStopOverrideActive)) != 0U ||
      live_chassis_fault || latched_chassis_fault;
    HallFeedbackDecision hall_decision =
      HallFeedbackDecision::kNotRequired;
    if (firmware_command_blocked) {
      hall_feedback_monitor_.reset();
    } else {
      hall_decision = hall_feedback_monitor_.observe(
        velocity_state == VelocityFeedbackState::kMeasuredMotion,
        steady_now_ms());
    }
    if (hall_decision == HallFeedbackDecision::kFault) {
      ++hall_feedback_fault_count_;
      latch_safety_stop("persistent Hall feedback loss during commanded motion");
    }

    const auto stamp = get_clock()->now();
    if (velocity_report_is_publishable(
        feedback, hall_decision))
    {
      autoware_vehicle_msgs::msg::VelocityReport velocity_report;
      velocity_report.header.stamp = stamp;
      velocity_report.header.frame_id = base_frame_id_;
      const bool confirmed_standstill =
        velocity_state == VelocityFeedbackState::kConfirmedStandstill;
      velocity_report.longitudinal_velocity = confirmed_standstill ? 0.0F :
        static_cast<float>(feedback.hall_speed_command_signed_mps);
      velocity_report.lateral_velocity = 0.0F;
      velocity_report.heading_rate = confirmed_standstill ? 0.0F :
        static_cast<float>(feedback.yaw_rate_estimate_rad_s);
      velocity_publisher_->publish(velocity_report);
    }

    if (steering_estimate_is_valid(feedback)) {
      autoware_vehicle_msgs::msg::SteeringReport steering_report;
      steering_report.stamp = stamp;
      steering_report.steering_tire_angle =
        static_cast<float>(feedback.steering_angle_estimate_rad);
      steering_publisher_->publish(steering_report);
    }

    autoware_vehicle_msgs::msg::GearReport gear_report;
    gear_report.stamp = stamp;
    gear_report.report = logical_gear_report_;
    gear_publisher_->publish(gear_report);

    autoware_vehicle_msgs::msg::ControlModeReport control_mode_report;
    control_mode_report.stamp = stamp;
    switch (reported_control_mode(feedback)) {
      case ReportedControlMode::kAutonomous:
        control_mode_report.mode =
          autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
        break;
      case ReportedControlMode::kManual:
        control_mode_report.mode =
          autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;
        break;
      case ReportedControlMode::kNoCommand:
        control_mode_report.mode =
          autoware_vehicle_msgs::msg::ControlModeReport::NO_COMMAND;
        break;
    }
    control_mode_publisher_->publish(control_mode_report);
  }

  void read_serial()
  {
    std::array<std::uint8_t, 512U> input{};
    for (;; ) {
      const ssize_t result = ::read(serial_fd_, input.data(), input.size());
      if (result > 0) {
        const auto before = feedback_decoder_.stats();
        const auto feedback_frames = feedback_decoder_.append(
          input.data(), static_cast<std::size_t>(result));
        const auto after = feedback_decoder_.stats();
        if (after.bcc_errors > before.bcc_errors ||
          after.tail_errors > before.tail_errors ||
          after.protocol_errors > before.protocol_errors)
        {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "feedback framing error: valid=%llu bcc=%llu tail=%llu "
            "protocol=%llu discarded=%llu",
            static_cast<unsigned long long>(after.valid_frames),
            static_cast<unsigned long long>(after.bcc_errors),
            static_cast<unsigned long long>(after.tail_errors),
            static_cast<unsigned long long>(after.protocol_errors),
            static_cast<unsigned long long>(after.discarded_bytes));
        }
        for (const auto & feedback : feedback_frames) {
          publish_feedback(feedback);
        }
        continue;
      }
      if (result < 0 && errno == EINTR) {
        continue;
      }
      if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        return;
      }
      if (result == 0) {
        return;
      }

      ++read_error_count_;
      close_serial("serial read error");
      return;
    }
  }

  void log_stats_if_due()
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_stats_log_ < kStatsLogPeriod) {
      return;
    }
    last_stats_log_ = now;
    const auto & decoder_stats = feedback_decoder_.stats();
    RCLCPP_INFO(
      get_logger(),
      "serial stats connected=%s valid=%llu bcc=%llu tail=%llu protocol=%llu "
      "discarded=%llu tx=%llu "
      "read_errors=%llu write_errors=%llu connections=%llu disconnects=%llu "
      "battery_mv=%u "
      "status_flags=0x%02x status_bits=0x%08x seq=%u dt_ms=%u "
      "hall_delta_command_signed=%d hall_speed_command_signed_mps=%.3f "
      "steering_pwm_estimate_rad=%.3f "
      "yaw_kinematic_estimate_rad_s=%.3f "
      "safety_stop_latched=%s fault_clear_pending=%s "
      "fault_clear_sent=%s clear_frames=%llu hall_invalid=%llu "
      "hall_contract_errors=%llu hall_faults=%llu "
      "steering_estimate_invalid=%llu "
      "unexpected_measured_steering=%llu",
      serial_fd_ >= 0 ? "true" : "false",
      static_cast<unsigned long long>(decoder_stats.valid_frames),
      static_cast<unsigned long long>(decoder_stats.bcc_errors),
      static_cast<unsigned long long>(decoder_stats.tail_errors),
      static_cast<unsigned long long>(decoder_stats.protocol_errors),
      static_cast<unsigned long long>(decoder_stats.discarded_bytes),
      static_cast<unsigned long long>(transmitted_frame_count_),
      static_cast<unsigned long long>(read_error_count_),
      static_cast<unsigned long long>(write_error_count_),
      static_cast<unsigned long long>(connection_count_),
      static_cast<unsigned long long>(disconnect_count_),
      static_cast<unsigned int>(last_battery_mv_),
      static_cast<unsigned int>(last_status_flags_),
      static_cast<unsigned int>(last_status_bits_),
      static_cast<unsigned int>(last_sequence_),
      static_cast<unsigned int>(last_feedback_dt_ms_),
      last_hall_delta_count_command_signed_,
      last_hall_speed_command_signed_mps_, last_steering_estimate_rad_,
      last_yaw_rate_estimate_rad_s_,
      safety_stop_latched_ ? "true" : "false",
      fault_clear_pending_ ? "true" : "false",
      fault_clear_sent_ ? "true" : "false",
      static_cast<unsigned long long>(fault_clear_frame_count_),
      static_cast<unsigned long long>(hall_feedback_invalid_count_),
      static_cast<unsigned long long>(hall_feedback_contract_error_count_),
      static_cast<unsigned long long>(hall_feedback_fault_count_),
      static_cast<unsigned long long>(steering_estimate_invalid_count_),
      static_cast<unsigned long long>(unexpected_measured_steering_count_));
  }

  void on_serial_timer()
  {
    try_open_serial();
    if (autonomous_requested_ &&
      !emergency_status_monitor_.fresh_and_clear(steady_now_ms()))
    {
      latch_safety_stop("missing or stale formal emergency status");
    }
    if (serial_fd_ >= 0) {
      pollfd descriptor{};
      descriptor.fd = serial_fd_;
      descriptor.events = POLLIN;
      const int poll_result = ::poll(&descriptor, 1, 0);
      if (poll_result < 0 && errno != EINTR) {
        ++read_error_count_;
        close_serial("serial read poll error");
      } else if (poll_result > 0 &&
        (descriptor.revents &
        static_cast<short>(POLLERR | POLLHUP | POLLNVAL)) != 0)
      {
        close_serial("serial hangup");
      } else if (poll_result > 0 && (descriptor.revents & POLLIN) != 0) {
        read_serial();
      }
    }
    log_stats_if_due();
  }

  std::string serial_port_;
  std::string base_frame_id_;
  std::int64_t baud_rate_{kExpectedBaudRate};
  std::int64_t firmware_command_timeout_ms_{kExpectedFirmwareCommandTimeoutMs};
  std::int64_t emergency_status_timeout_ms_{250};
  std::int64_t hall_feedback_acquisition_timeout_ms_{1500};
  std::int64_t hall_feedback_loss_timeout_ms_{250};
  std::int64_t serial_poll_period_ms_{10};
  std::int64_t reconnect_period_ms_{1000};
  VehicleCommandLimits command_limits_;

  int serial_fd_{-1};
  FeedbackStreamDecoder feedback_decoder_;
  EmergencyStatusMonitor emergency_status_monitor_{250};
  HallFeedbackMonitor hall_feedback_monitor_{1500, 250};
  bool autonomous_requested_{false};
  bool has_transmitted_command_{false};
  bool safety_stop_latched_{false};
  bool safety_stop_sent_{false};
  bool rc_override_active_{false};
  bool has_received_feedback_{false};
  bool fault_clear_pending_{false};
  bool fault_clear_zero_sent_{false};
  bool fault_clear_sent_{false};
  LogicalGear logical_gear_{LogicalGear::kPark};
  std::uint8_t logical_gear_report_{
    autoware_vehicle_msgs::msg::GearReport::PARK};
  std::int64_t last_transmitted_command_stamp_ns_{
    std::numeric_limits<std::int64_t>::min()};
  std::chrono::steady_clock::time_point next_reconnect_attempt_;
  std::chrono::steady_clock::time_point last_stats_log_;
  std::uint16_t last_battery_mv_{0U};
  std::uint16_t last_feedback_dt_ms_{0U};
  std::uint8_t last_status_flags_{0U};
  std::uint8_t last_sequence_{0U};
  std::int32_t last_hall_delta_count_command_signed_{0};
  std::uint32_t last_status_bits_{0U};
  double last_hall_speed_command_signed_mps_{0.0};
  double last_steering_estimate_rad_{0.0};
  double last_yaw_rate_estimate_rad_s_{0.0};
  std::uint64_t transmitted_frame_count_{0U};
  std::uint64_t read_error_count_{0U};
  std::uint64_t write_error_count_{0U};
  std::uint64_t connection_count_{0U};
  std::uint64_t disconnect_count_{0U};
  std::uint64_t fault_clear_frame_count_{0U};
  std::uint64_t hall_feedback_invalid_count_{0U};
  std::uint64_t hall_feedback_contract_error_count_{0U};
  std::uint64_t hall_feedback_fault_count_{0U};
  std::uint64_t steering_estimate_invalid_count_{0U};
  std::uint64_t unexpected_measured_steering_count_{0U};

  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr
    command_subscription_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr
    gear_subscription_;
  rclcpp::Subscription<
    tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr
    emergency_subscription_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr
    velocity_publisher_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr
    steering_publisher_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr
    gear_publisher_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr
    control_mode_publisher_;
  rclcpp::Service<autoware_vehicle_msgs::srv::ControlModeCommand>::SharedPtr
    control_mode_service_;
  rclcpp::TimerBase::SharedPtr serial_timer_;
};

} // namespace autoracer_rc_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<autoracer_rc_adapter::RcVehicleInterfaceNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("rc_vehicle_interface"), "node failed: %s",
      error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
