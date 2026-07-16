#include "autoracer_rc_adapter/rc_serial_protocol.hpp"
#include "autoracer_rc_adapter/rc_vehicle_kinematics.hpp"

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <rclcpp/rclcpp.hpp>

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
      declare_parameter<double>("max_steering_tire_angle_rad", 0.262);
    firmware_command_timeout_ms_ = declare_parameter<std::int64_t>(
      "firmware_command_timeout_ms", kExpectedFirmwareCommandTimeoutMs);
    serial_poll_period_ms_ =
      declare_parameter<std::int64_t>("serial_poll_period_ms", 10);
    reconnect_period_ms_ =
      declare_parameter<std::int64_t>("reconnect_period_ms", 1000);
    base_frame_id_ =
      declare_parameter<std::string>("base_frame_id", "base_link");

    validate_parameters();

    const auto command_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    command_subscription_ =
      create_subscription<autoware_control_msgs::msg::Control>(
      "/control/command/control_cmd", command_qos,
      std::bind(
        &RcVehicleInterfaceNode::on_control_command, this,
        std::placeholders::_1));
    velocity_publisher_ =
      create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
      "/vehicle/status/velocity_status", command_qos);

    next_reconnect_attempt_ = std::chrono::steady_clock::now();
    last_stats_log_ = std::chrono::steady_clock::now();
    serial_timer_ = create_wall_timer(
      std::chrono::milliseconds(serial_poll_period_ms_),
      std::bind(&RcVehicleInterfaceNode::on_serial_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "RC vehicle interface configured for %s at 115200 8N1; it "
      "writes nothing until a fresh "
      "Control message is received; Control velocity/steering map "
      "directly to firmware speed/steering; Control acceleration and "
      "jerk are unsupported by this chassis and intentionally ignored",
      serial_port_.c_str());
  }

  ~RcVehicleInterfaceNode() override
  {
    if (serial_fd_ >= 0 && has_transmitted_command_) {
      ChassisCommand software_stop;
      software_stop.enable = false;
      software_stop.software_stop = true;
      try {
        const auto frame = encode_command_frame(software_stop);
        if (write_frame(frame)) {
          RCLCPP_INFO(
            get_logger(), "sent one firmware software-stop frame "
            "during orderly shutdown; this is not a "
            "physical emergency stop");
        }
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(),
          "could not encode shutdown software-stop frame: %s",
          error.what());
      }
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

  void on_control_command(
    const autoware_control_msgs::msg::Control::SharedPtr message)
  {
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
      command.speed_mps = setpoint.speed_mps;
      command.steering_tire_angle_rad = setpoint.steering_tire_angle_rad;
      command.enable = true;
      command.software_stop = false;
      const auto frame = encode_command_frame(command);
      if (write_frame(frame)) {
        last_transmitted_command_stamp_ns_ = stamp_ns;
        has_transmitted_command_ = true;
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
    if ((feedback.status_bits & kRcStatusRcOverrideActive) != 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "firmware reports RC override active; this is raw chassis state, not a published "
        "control-mode message");
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
        "firmware reports a latched diagnostic fault; inspect status_bits for its source");
    }
    if ((feedback.status_bits & kRcStatusHallFeedbackValid) == 0U) {
      ++hall_feedback_invalid_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "firmware marks Hall feedback invalid; its speed "
        "field is zero when direction or Hall "
        "feedback is unavailable");
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
    autoware_vehicle_msgs::msg::VelocityReport report;
    report.header.stamp = get_clock()->now();
    report.header.frame_id = base_frame_id_;
    report.longitudinal_velocity =
      static_cast<float>(feedback.hall_speed_command_signed_mps);
    report.lateral_velocity = 0.0F;
    report.heading_rate = static_cast<float>(feedback.yaw_rate_estimate_rad_s);
    velocity_publisher_->publish(report);
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
      "hall_invalid=%llu steering_estimate_invalid=%llu "
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
      static_cast<unsigned long long>(hall_feedback_invalid_count_),
      static_cast<unsigned long long>(steering_estimate_invalid_count_),
      static_cast<unsigned long long>(unexpected_measured_steering_count_));
  }

  void on_serial_timer()
  {
    try_open_serial();
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
  std::int64_t serial_poll_period_ms_{10};
  std::int64_t reconnect_period_ms_{1000};
  VehicleCommandLimits command_limits_;

  int serial_fd_{-1};
  FeedbackStreamDecoder feedback_decoder_;
  bool has_transmitted_command_{false};
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
  std::uint64_t hall_feedback_invalid_count_{0U};
  std::uint64_t steering_estimate_invalid_count_{0U};
  std::uint64_t unexpected_measured_steering_count_{0U};

  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr
    command_subscription_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr
    velocity_publisher_;
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
