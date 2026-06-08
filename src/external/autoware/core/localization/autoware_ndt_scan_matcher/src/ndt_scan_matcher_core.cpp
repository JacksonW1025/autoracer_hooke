// Copyright 2015-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <autoware/localization_util/matrix_type.hpp>
#include <autoware/localization_util/tree_structured_parzen_estimator.hpp>
#include <autoware/localization_util/util_func.hpp>
#include <autoware/ndt_scan_matcher/initial_pose_offsets.hpp>
#include <autoware/ndt_scan_matcher/ndt_omp/estimate_covariance.hpp>
#include <autoware/ndt_scan_matcher/ndt_scan_matcher_core.hpp>
#include <autoware/ndt_scan_matcher/particle.hpp>
#include <autoware/ndt_scan_matcher/runtime_multistart.hpp>
#include <autoware/ndt_scan_matcher/time_offset.hpp>
#include <autoware/ndt_scan_matcher/validation.hpp>
#include <autoware/qos_utils/qos_compatibility.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_pcl/transforms.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <thread>

namespace autoware::ndt_scan_matcher
{
using autoware::localization_util::exchange_color_crc;
using autoware::localization_util::matrix4f_to_pose;
using autoware::localization_util::point_to_vector3d;
using autoware::localization_util::pose_to_matrix4f;

using autoware::localization_util::SmartPoseBuffer;
using autoware::localization_util::TreeStructuredParzenEstimator;
using autoware_utils_diagnostics::DiagnosticsInterface;

autoware_internal_debug_msgs::msg::Float32Stamped make_float32_stamped(
  const builtin_interfaces::msg::Time & stamp, const float data)
{
  using T = autoware_internal_debug_msgs::msg::Float32Stamped;
  return autoware_internal_debug_msgs::build<T>().stamp(stamp).data(data);
}

autoware_internal_debug_msgs::msg::Int32Stamped make_int32_stamped(
  const builtin_interfaces::msg::Time & stamp, const int32_t data)
{
  using T = autoware_internal_debug_msgs::msg::Int32Stamped;
  return autoware_internal_debug_msgs::build<T>().stamp(stamp).data(data);
}

std::array<double, 36> rotate_covariance(
  const std::array<double, 36> & src_covariance, const Eigen::Matrix3d & rotation)
{
  std::array<double, 36> ret_covariance = src_covariance;

  Eigen::Matrix3d src_cov;
  src_cov << src_covariance[0], src_covariance[1], src_covariance[2], src_covariance[6],
    src_covariance[7], src_covariance[8], src_covariance[12], src_covariance[13],
    src_covariance[14];

  Eigen::Matrix3d ret_cov;
  ret_cov = rotation * src_cov * rotation.transpose();

  for (Eigen::Index i = 0; i < 3; ++i) {
    ret_covariance[i] = ret_cov(0, i);
    ret_covariance[i + 6] = ret_cov(1, i);
    ret_covariance[i + 12] = ret_cov(2, i);
  }

  return ret_covariance;
}

double normalize_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double yaw_from_pose(const geometry_msgs::msg::Pose & pose)
{
  return autoware::localization_util::get_rpy(pose).z;
}

geometry_msgs::msg::Pose offset_pose_in_body_frame(
  const geometry_msgs::msg::Pose & pose, const double along_m, const double cross_m,
  const double yaw_offset_rad)
{
  geometry_msgs::msg::Pose ret = pose;
  const auto rpy = autoware::localization_util::get_rpy(pose);
  const double yaw = rpy.z;
  ret.position.x += std::cos(yaw) * along_m - std::sin(yaw) * cross_m;
  ret.position.y += std::sin(yaw) * along_m + std::cos(yaw) * cross_m;

  tf2::Quaternion quaternion;
  quaternion.setRPY(rpy.x, rpy.y, normalize_angle(yaw + yaw_offset_rad));
  ret.orientation = tf2::toMsg(quaternion);
  return ret;
}

template <class TransformationArray>
std::vector<geometry_msgs::msg::Pose> transformation_array_to_poses(
  const TransformationArray & transformation_array)
{
  std::vector<geometry_msgs::msg::Pose> poses;
  poses.reserve(transformation_array.size());
  for (const auto & pose_matrix : transformation_array) {
    poses.push_back(matrix4f_to_pose(pose_matrix));
  }
  return poses;
}

RuntimeCandidateScoringOptions make_runtime_scoring_options(const HyperParameters & param)
{
  RuntimeCandidateScoringOptions options;
  if (param.score_estimation.converged_param_type == ConvergedParamType::TRANSFORM_PROBABILITY) {
    options.min_transform_probability = param.runtime_multistart.min_transform_probability;
    options.min_nearest_voxel_transformation_likelihood =
      -std::numeric_limits<double>::infinity();
  } else {
    options.min_transform_probability = -std::numeric_limits<double>::infinity();
    options.min_nearest_voxel_transformation_likelihood =
      param.runtime_multistart.min_nearest_voxel_transformation_likelihood;
  }
  options.max_initial_to_result_distance_m =
    param.runtime_multistart.max_initial_to_result_distance_m;
  options.max_prior_innovation_m = param.runtime_multistart.max_prior_innovation_m;
  options.max_prior_along_m = param.runtime_multistart.max_prior_along_m;
  options.max_prior_cross_m = param.runtime_multistart.max_prior_cross_m;
  options.max_prior_yaw_deg = param.runtime_multistart.max_prior_yaw_deg;
  options.min_total_score = param.runtime_multistart.min_total_score;
  options.max_iteration_num = param.ndt.max_iterations;
  options.score_weight = param.runtime_multistart.score_weight;
  options.transform_probability_weight = param.runtime_multistart.transform_probability_weight;
  options.innovation_xy_penalty_weight = param.runtime_multistart.innovation_xy_penalty_weight;
  options.innovation_along_penalty_weight =
    param.runtime_multistart.innovation_along_penalty_weight;
  options.innovation_cross_penalty_weight =
    param.runtime_multistart.innovation_cross_penalty_weight;
  options.innovation_yaw_penalty_weight = param.runtime_multistart.innovation_yaw_penalty_weight;
  options.initial_to_result_penalty_weight =
    param.runtime_multistart.initial_to_result_penalty_weight;
  options.covariance_condition_penalty_weight =
    param.runtime_multistart.covariance_condition_penalty_weight;
  options.base_candidate_raw_score_margin = param.runtime_multistart.base_candidate_raw_score_margin;
  options.raw_score_override_margin = param.runtime_multistart.raw_score_override_margin;
  options.raw_score_override_max_total_score_drop =
    param.runtime_multistart.raw_score_override_max_total_score_drop;
  options.raw_score_override_max_abs_along_m =
    param.runtime_multistart.raw_score_override_max_abs_along_m;
  options.raw_score_override_max_abs_cross_m =
    param.runtime_multistart.raw_score_override_max_abs_cross_m;
  options.raw_score_override_max_abs_yaw_deg =
    param.runtime_multistart.raw_score_override_max_abs_yaw_deg;
  options.raw_score_override_max_initial_to_result_distance_m =
    param.runtime_multistart.raw_score_override_max_initial_to_result_distance_m;
  return options;
}

RuntimeCandidateTierOptions make_runtime_tier_options(const HyperParameters & param)
{
  RuntimeCandidateTierOptions options;
  options.tier1_max_abs_along_m = param.runtime_multistart.tier1_max_abs_along_m;
  options.tier1_max_abs_cross_m = param.runtime_multistart.tier1_max_abs_cross_m;
  options.tier1_max_abs_yaw_deg = param.runtime_multistart.tier1_max_abs_yaw_deg;
  options.tracking_along_health_period_sec =
    param.runtime_multistart.tracking_along_health_period_sec;
  options.tracking_far_tier_period_sec = param.runtime_multistart.tracking_far_tier_period_sec;
  options.ambiguity_score_margin = param.runtime_multistart.ambiguity_score_margin;
  options.ambiguity_along_spread_m = param.runtime_multistart.ambiguity_along_spread_m;
  options.recovery_stable_max_innovation_m =
    param.runtime_multistart.recovery_stable_max_innovation_m;
  options.recovery_stable_max_yaw_deg = param.runtime_multistart.recovery_stable_max_yaw_deg;
  options.recovery_far_tier_period_sec = param.runtime_multistart.recovery_far_tier_period_sec;
  options.recovery_far_tier_min_scan_interval =
    param.runtime_multistart.recovery_far_tier_min_scan_interval;
  return options;
}

NDTScanMatcher::NDTScanMatcher(const rclcpp::NodeOptions & options)
: Node("ndt_scan_matcher", options),
  tf2_broadcaster_(*this),
  tf2_buffer_(this->get_clock()),
  tf2_listener_(tf2_buffer_),
  ndt_ptr_(new NormalDistributionsTransform),
  is_activated_(false),
  param_(this)
{
  timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::CallbackGroup::SharedPtr initial_pose_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::CallbackGroup::SharedPtr sensor_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto initial_pose_sub_opt = rclcpp::SubscriptionOptions();
  initial_pose_sub_opt.callback_group = initial_pose_callback_group;
  auto sensor_sub_opt = rclcpp::SubscriptionOptions();
  sensor_sub_opt.callback_group = sensor_callback_group;

  constexpr double map_update_dt = 1.0;
  constexpr auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(map_update_dt));
  map_update_timer_ = rclcpp::create_timer(
    this, this->get_clock(), period_ns, std::bind(&NDTScanMatcher::callback_timer, this),
    timer_callback_group_);
  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_pose_with_covariance", 10,
    std::bind(&NDTScanMatcher::callback_initial_pose, this, std::placeholders::_1),
    initial_pose_sub_opt);
  sensor_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&NDTScanMatcher::callback_sensor_points, this, std::placeholders::_1),
    sensor_sub_opt);

  // Only if regularization is enabled, subscribe to the regularization base pose
  if (param_.ndt_regularization_enable) {
    // NOTE: The reason that the regularization subscriber does not belong to the
    // sensor_callback_group is to ensure that the regularization callback is called even if
    // sensor_callback takes long time to process.
    // Both callback_initial_pose and callback_regularization_pose must not miss receiving data for
    // proper interpolation.
    regularization_pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "regularization_pose_with_covariance", 10,
        std::bind(&NDTScanMatcher::callback_regularization_pose, this, std::placeholders::_1),
        initial_pose_sub_opt);
    const double value_as_unlimited = 1000.0;
    regularization_pose_buffer_ =
      std::make_unique<SmartPoseBuffer>(this->get_logger(), value_as_unlimited, value_as_unlimited);

    diagnostics_regularization_pose_ =
      std::make_unique<DiagnosticsInterface>(this, "regularization_pose_subscriber_status");
  }

  sensor_aligned_pose_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("points_aligned", 10);
  no_ground_points_aligned_pose_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("points_aligned_no_ground", 10);
  ndt_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ndt_pose", 10);
  ndt_pose_with_covariance_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "ndt_pose_with_covariance", 10);
  initial_pose_with_covariance_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initial_pose_with_covariance", 10);
  multi_ndt_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("multi_ndt_pose", 10);
  multi_initial_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>("multi_initial_pose", 10);
  exe_time_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>("exe_time_ms", 10);
  transform_probability_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "transform_probability", 10);
  nearest_voxel_transformation_likelihood_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "nearest_voxel_transformation_likelihood", 10);
  voxel_score_points_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_score_points", 10);
  no_ground_transform_probability_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "no_ground_transform_probability", 10);
  no_ground_nearest_voxel_transformation_likelihood_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "no_ground_nearest_voxel_transformation_likelihood", 10);
  iteration_num_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Int32Stamped>("iteration_num", 10);
  initial_to_result_relative_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("initial_to_result_relative_pose", 10);
  initial_to_result_distance_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "initial_to_result_distance", 10);
  initial_to_result_distance_old_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "initial_to_result_distance_old", 10);
  initial_to_result_distance_new_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "initial_to_result_distance_new", 10);
  ndt_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("ndt_marker", 10);
  ndt_monte_carlo_initial_pose_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "monte_carlo_initial_pose_marker", 10);
  runtime_multistart_debug_pub_ =
    param_.runtime_multistart.debug_topic.empty()
      ? nullptr
      : this->create_publisher<std_msgs::msg::String>(
          param_.runtime_multistart.debug_topic, rclcpp::QoS{10});

  service_ =
    this->create_service<autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped>(
      "ndt_align_srv",
      std::bind(
        &NDTScanMatcher::service_ndt_align, this, std::placeholders::_1, std::placeholders::_2),
      AUTOWARE_DEFAULT_SERVICES_QOS_PROFILE(), sensor_callback_group);
  service_trigger_node_ = this->create_service<std_srvs::srv::SetBool>(
    "trigger_node_srv",
    std::bind(
      &NDTScanMatcher::service_trigger_node, this, std::placeholders::_1, std::placeholders::_2),
    AUTOWARE_DEFAULT_SERVICES_QOS_PROFILE(), sensor_callback_group);

  ndt_ptr_->setParams(param_.ndt);

  initial_pose_buffer_ = std::make_unique<SmartPoseBuffer>(
    this->get_logger(), param_.validation.initial_pose_timeout_sec,
    param_.validation.initial_pose_distance_tolerance_m);

  map_update_module_ =
    std::make_unique<MapUpdateModule>(this, &ndt_ptr_mtx_, ndt_ptr_, param_.dynamic_map_loading);

  diagnostics_scan_points_ = std::make_unique<DiagnosticsInterface>(this, "scan_matching_status");
  diagnostics_initial_pose_ =
    std::make_unique<DiagnosticsInterface>(this, "initial_pose_subscriber_status");
  diagnostics_map_update_ = std::make_unique<DiagnosticsInterface>(this, "map_update_status");
  diagnostics_ndt_align_ = std::make_unique<DiagnosticsInterface>(this, "ndt_align_service_status");
  diagnostics_trigger_node_ =
    std::make_unique<DiagnosticsInterface>(this, "trigger_node_service_status");

  logger_configure_ = std::make_unique<autoware_utils_logging::LoggerLevelConfigure>(this);
}

void NDTScanMatcher::callback_timer()
{
  const rclcpp::Time ros_time_now = this->now();

  diagnostics_map_update_->clear();

  diagnostics_map_update_->add_key_value("timer_callback_time_stamp", ros_time_now.nanoseconds());

  map_update_module_->callback_timer(is_activated_, latest_ekf_position_, diagnostics_map_update_);

  diagnostics_map_update_->publish(ros_time_now);
}

void NDTScanMatcher::callback_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initial_pose_msg_ptr)
{
  diagnostics_initial_pose_->clear();

  callback_initial_pose_main(initial_pose_msg_ptr);

  diagnostics_initial_pose_->publish(initial_pose_msg_ptr->header.stamp);
}

void NDTScanMatcher::callback_initial_pose_main(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initial_pose_msg_ptr)
{
  diagnostics_initial_pose_->add_key_value(
    "topic_time_stamp",
    static_cast<rclcpp::Time>(initial_pose_msg_ptr->header.stamp).nanoseconds());

  // check is_activated
  diagnostics_initial_pose_->add_key_value("is_activated", static_cast<bool>(is_activated_));
  if (!is_activated_) {
    std::stringstream message;
    message << "Node is not activated.";
    diagnostics_initial_pose_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return;
  }

  // check is_expected_frame_id
  const bool is_expected_frame_id =
    (initial_pose_msg_ptr->header.frame_id == param_.frame.map_frame);
  diagnostics_initial_pose_->add_key_value("is_expected_frame_id", is_expected_frame_id);
  if (!is_expected_frame_id) {
    std::stringstream message;
    message << "Received initial pose message with frame_id "
            << initial_pose_msg_ptr->header.frame_id << ", but expected " << param_.frame.map_frame
            << ". Please check the frame_id in the input topic and ensure it is correct.";
    diagnostics_initial_pose_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());
    return;
  }

  initial_pose_buffer_->push_back(initial_pose_msg_ptr);

  {
    // latest_ekf_position_ is also used by callback_timer, so it is necessary to acquire the lock
    std::lock_guard<std::mutex> lock(latest_ekf_position_mtx_);
    latest_ekf_position_ = initial_pose_msg_ptr->pose.pose.position;
  }
}

void NDTScanMatcher::callback_regularization_pose(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr)
{
  diagnostics_regularization_pose_->clear();

  diagnostics_regularization_pose_->add_key_value(
    "topic_time_stamp", static_cast<rclcpp::Time>(pose_conv_msg_ptr->header.stamp).nanoseconds());

  regularization_pose_buffer_->push_back(pose_conv_msg_ptr);

  diagnostics_regularization_pose_->publish(pose_conv_msg_ptr->header.stamp);
}

void NDTScanMatcher::callback_sensor_points(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_msg_in_sensor_frame)
{
  // clear diagnostics
  diagnostics_scan_points_->clear();

  // scan matching
  const bool is_succeed_scan_matching =
    callback_sensor_points_main(sensor_points_msg_in_sensor_frame);

  // check skipping_publish_num
  static int64_t skipping_publish_num = 0;
  skipping_publish_num =
    ((is_succeed_scan_matching || !is_activated_) ? 0 : (skipping_publish_num + 1));
  diagnostics_scan_points_->add_key_value("skipping_publish_num", skipping_publish_num);
  if (skipping_publish_num >= param_.validation.skipping_publish_num) {
    std::stringstream message;
    message << "skipping_publish_num exceed limit (" << skipping_publish_num << " times).";
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }

  diagnostics_scan_points_->publish(sensor_points_msg_in_sensor_frame->header.stamp);
}

bool NDTScanMatcher::callback_sensor_points_main(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_msg_in_sensor_frame)
{
  const auto exe_start_time = std::chrono::system_clock::now();

  // check topic_time_stamp
  const rclcpp::Time sensor_ros_time = sensor_points_msg_in_sensor_frame->header.stamp;
  diagnostics_scan_points_->add_key_value("topic_time_stamp", sensor_ros_time.nanoseconds());

  // check sensor_points_size
  const size_t sensor_points_size = sensor_points_msg_in_sensor_frame->width;
  diagnostics_scan_points_->add_key_value("sensor_points_size", sensor_points_size);
  if (sensor_points_size == 0) {
    std::stringstream message;
    message << "Sensor points is empty.";
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return false;
  }

  // check sensor_points_delay_time_sec
  const double sensor_points_delay_time_sec =
    (this->now() - sensor_points_msg_in_sensor_frame->header.stamp).seconds();
  diagnostics_scan_points_->add_key_value(
    "sensor_points_delay_time_sec", sensor_points_delay_time_sec);
  if (sensor_points_delay_time_sec > param_.sensor_points.timeout_sec) {
    std::stringstream message;
    message << "sensor points is experiencing latency."
            << "The delay time is " << sensor_points_delay_time_sec << "[sec] "
            << "(the tolerance is " << param_.sensor_points.timeout_sec << "[sec]).";
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());

    // If the delay time of the LiDAR topic exceeds the delay compensation time of ekf_localizer,
    // even if further processing continues, the estimated result will be rejected by ekf_localizer.
    // Therefore, it would be acceptable to exit the function here.
    // However, for now, we will continue the processing as it is.

    // return false;
  }

  // preprocess input pointcloud
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_sensor_frame(
    new pcl::PointCloud<PointSource>);
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_baselink_frame(
    new pcl::PointCloud<PointSource>);
  const std::string & sensor_frame = sensor_points_msg_in_sensor_frame->header.frame_id;

  pcl::fromROSMsg(*sensor_points_msg_in_sensor_frame, *sensor_points_in_sensor_frame);

  // transform sensor points from sensor-frame to base_link
  try {
    transform_sensor_measurement(
      sensor_frame, param_.frame.base_frame, sensor_points_in_sensor_frame,
      sensor_points_in_baselink_frame);
  } catch (const std::exception & ex) {
    std::stringstream message;
    message << ex.what() << ". Please publish TF " << sensor_frame << " to "
            << param_.frame.base_frame;
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    diagnostics_scan_points_->add_key_value("is_succeed_transform_sensor_points", false);
    return false;
  }
  diagnostics_scan_points_->add_key_value("is_succeed_transform_sensor_points", true);

  // check sensor_points_max_distance
  double max_distance = 0.0;
  for (const auto & point : sensor_points_in_baselink_frame->points) {
    const double distance = std::hypot(point.x, point.y, point.z);
    max_distance = std::max(max_distance, distance);
  }

  diagnostics_scan_points_->add_key_value("sensor_points_max_distance", max_distance);
  if (max_distance < param_.sensor_points.required_distance) {
    std::stringstream message;
    message << "Max distance of sensor points = " << std::fixed << std::setprecision(3)
            << max_distance << " [m] < " << param_.sensor_points.required_distance << " [m]";
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return false;
  }

  // lock mutex
  std::lock_guard<std::mutex> lock(ndt_ptr_mtx_);

  // set sensor points to ndt class
  ndt_ptr_->setInputSource(sensor_points_in_baselink_frame);
  latest_sensor_points_stamp_ = sensor_ros_time;
  has_latest_sensor_points_stamp_ = true;

  // check is_activated
  diagnostics_scan_points_->add_key_value("is_activated", static_cast<bool>(is_activated_));
  if (!is_activated_) {
    std::stringstream message;
    message << "Node is not activated.";
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return false;
  }

  // calculate initial pose
  std::optional<SmartPoseBuffer::InterpolateResult> interpolation_result_opt =
    initial_pose_buffer_->interpolate(sensor_ros_time);

  // check is_succeed_interpolate_initial_pose
  const bool is_succeed_interpolate_initial_pose = (interpolation_result_opt != std::nullopt);
  diagnostics_scan_points_->add_key_value(
    "is_succeed_interpolate_initial_pose", is_succeed_interpolate_initial_pose);
  if (!is_succeed_interpolate_initial_pose) {
    std::stringstream message;
    message << "Couldn't interpolate pose. Please verify that "
               "(1) the initial pose topic (primarily come from the EKF) is being published, and "
               "(2) the timestamps of the sensor PCD messages and pose messages are synchronized "
               "correctly.";
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return false;
  }

  initial_pose_buffer_->pop_old(sensor_ros_time);
  const SmartPoseBuffer::InterpolateResult & interpolation_result =
    interpolation_result_opt.value();

  // if regularization is enabled and available, set pose to NDT for regularization
  if (param_.ndt_regularization_enable) {
    add_regularization_pose(sensor_ros_time);
  }

  // Warn if the lidar has gone out of the map range
  if (map_update_module_->out_of_map_range(
        interpolation_result.interpolated_pose.pose.pose.position)) {
    std::stringstream msg;

    msg << "Lidar has gone out of the map range";
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, msg.str());

    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, msg.str());
  }

  // check is_set_map_points
  const bool is_set_map_points = (ndt_ptr_->getInputTarget() != nullptr);
  diagnostics_scan_points_->add_key_value("is_set_map_points", is_set_map_points);
  if (!is_set_map_points) {
    std::stringstream message;
    message << "Map points is not set.";
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return false;
  }

  struct RuntimeAlignment
  {
    std::size_t index{};
    double offset_along_m{};
    double offset_cross_m{};
    double offset_yaw_deg{};
    geometry_msgs::msg::Pose initial_pose{};
    Eigen::Matrix4f initial_pose_matrix{};
    pcl::shared_ptr<pcl::PointCloud<PointSource>> output_cloud{};
    pclomp::NdtResult ndt_result{};
    geometry_msgs::msg::Pose result_pose{};
    std::vector<geometry_msgs::msg::Pose> transformation_msg_array{};
    RuntimeCandidate candidate{};
    double score{};
    bool is_ok_score{};
    bool is_ok_iteration_num{};
    bool is_local_optimal_solution_oscillation{};
  };

  // perform NDT scan matching for one or more runtime candidates
  const geometry_msgs::msg::Pose prior_pose = interpolation_result.interpolated_pose.pose.pose;
  std::vector<double> offset_along_m =
    param_.runtime_multistart.enable ? param_.runtime_multistart.offset_along_m
                                     : std::vector<double>{0.0};
  std::vector<double> offset_cross_m =
    param_.runtime_multistart.enable ? param_.runtime_multistart.offset_cross_m
                                     : std::vector<double>{0.0};
  std::vector<double> offset_yaw_deg =
    param_.runtime_multistart.enable ? param_.runtime_multistart.offset_yaw_deg
                                     : std::vector<double>{0.0};
  if (offset_along_m.empty()) {
    offset_along_m = {0.0};
    offset_cross_m = {0.0};
    offset_yaw_deg = {0.0};
  }

  const double prior_yaw = yaw_from_pose(prior_pose);
  const double prior_forward_x = std::cos(prior_yaw);
  const double prior_forward_y = std::sin(prior_yaw);
  const double prior_lateral_x = -std::sin(prior_yaw);
  const double prior_lateral_y = std::cos(prior_yaw);

  std::vector<RuntimeAlignment> runtime_alignments;
  runtime_alignments.reserve(offset_along_m.size());
  std::vector<RuntimeCandidate> runtime_candidates;
  runtime_candidates.reserve(offset_along_m.size());
  const RuntimeCandidateTierOptions runtime_tier_options = make_runtime_tier_options(param_);
  std::vector<std::size_t> small_tier_indices;
  std::vector<std::size_t> along_health_indices;
  std::vector<std::size_t> far_tier_indices;
  for (std::size_t i = 1; i < offset_along_m.size(); ++i) {
    RuntimeCandidate offset_probe;
    offset_probe.offset_along_m = offset_along_m[i];
    offset_probe.offset_cross_m = offset_cross_m[i];
    offset_probe.offset_yaw_deg = offset_yaw_deg[i];
    if (is_far_runtime_candidate(offset_probe, runtime_tier_options)) {
      far_tier_indices.push_back(i);
    } else {
      small_tier_indices.push_back(i);
      if (is_along_health_runtime_candidate(offset_probe, runtime_tier_options)) {
        along_health_indices.push_back(i);
      }
    }
  }
  bool tier2_evaluated = false;
  bool along_health_evaluated = false;
  std::string tier2_trigger_reason;
  bool small_tier_ambiguous = false;

  auto run_runtime_alignment = [&](const std::size_t i) -> bool {
    RuntimeAlignment alignment;
    alignment.index = i;
    alignment.offset_along_m = offset_along_m[i];
    alignment.offset_cross_m = offset_cross_m[i];
    alignment.offset_yaw_deg = offset_yaw_deg[i];
    alignment.initial_pose = offset_pose_in_body_frame(
      prior_pose, alignment.offset_along_m, alignment.offset_cross_m,
      alignment.offset_yaw_deg * M_PI / 180.0);
    alignment.initial_pose_matrix = pose_to_matrix4f(alignment.initial_pose);
    alignment.output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();
    ndt_ptr_->align(*alignment.output_cloud, alignment.initial_pose_matrix);
    alignment.ndt_result = ndt_ptr_->getResult();
    alignment.result_pose = matrix4f_to_pose(alignment.ndt_result.pose);
    alignment.transformation_msg_array =
      transformation_array_to_poses(alignment.ndt_result.transformation_array);

    alignment.is_ok_iteration_num =
      alignment.ndt_result.iteration_num < ndt_ptr_->getMaximumIterations();
    constexpr int oscillation_num_threshold = 10;
    alignment.is_local_optimal_solution_oscillation =
      count_oscillation(alignment.transformation_msg_array) > oscillation_num_threshold;

    if (
      param_.score_estimation.converged_param_type ==
      ConvergedParamType::TRANSFORM_PROBABILITY) {
      alignment.score = alignment.ndt_result.transform_probability;
      alignment.is_ok_score =
        alignment.score > param_.score_estimation.converged_param_transform_probability;
    } else if (
      param_.score_estimation.converged_param_type ==
      ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD) {
      alignment.score = alignment.ndt_result.nearest_voxel_transformation_likelihood;
      alignment.is_ok_score =
        alignment.score >
        param_.score_estimation.converged_param_nearest_voxel_transformation_likelihood;
    } else {
      std::stringstream message;
      message << "Unknown converged param type. Please check `score_estimation.converged_param_type`";
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());
      return false;
    }

    const auto initial_to_result = static_cast<double>(
      autoware::localization_util::norm(alignment.initial_pose.position, alignment.result_pose.position));
    const double prior_dx = alignment.result_pose.position.x - prior_pose.position.x;
    const double prior_dy = alignment.result_pose.position.y - prior_pose.position.y;
    RuntimeCandidate candidate;
    candidate.index = alignment.index;
    candidate.converged =
      (alignment.is_ok_iteration_num || alignment.is_local_optimal_solution_oscillation) &&
      alignment.is_ok_score;
    candidate.iteration_num = alignment.ndt_result.iteration_num;
    candidate.max_iterations = ndt_ptr_->getMaximumIterations();
    candidate.transform_probability = alignment.ndt_result.transform_probability;
    candidate.nearest_voxel_transformation_likelihood =
      alignment.ndt_result.nearest_voxel_transformation_likelihood;
    candidate.initial_to_result_distance_m = initial_to_result;
    candidate.innovation_along_m = prior_dx * prior_forward_x + prior_dy * prior_forward_y;
    candidate.innovation_cross_m = prior_dx * prior_lateral_x + prior_dy * prior_lateral_y;
    candidate.innovation_yaw_rad = normalize_angle(yaw_from_pose(alignment.result_pose) - prior_yaw);
    candidate.offset_along_m = alignment.offset_along_m;
    candidate.offset_cross_m = alignment.offset_cross_m;
    candidate.offset_yaw_deg = alignment.offset_yaw_deg;
    candidate.covariance_condition_number = 1.0;
    alignment.candidate = candidate;
    runtime_candidates.push_back(candidate);
    runtime_alignments.push_back(alignment);
    return true;
  };

  if (!run_runtime_alignment(0)) {
    return false;
  }
  ++runtime_scans_since_last_far_tier_;

  const RuntimeCandidate & base_candidate = runtime_candidates.front();
  double base_score_threshold = 0.0;
  if (param_.score_estimation.converged_param_type == ConvergedParamType::TRANSFORM_PROBABILITY) {
    base_score_threshold = param_.score_estimation.converged_param_transform_probability;
  } else {
    base_score_threshold =
      param_.score_estimation.converged_param_nearest_voxel_transformation_likelihood;
  }
  const bool runtime_stamp_enabled =
    sensor_ros_time.seconds() >= param_.runtime_multistart.min_stamp_sec;
  const bool base_score_has_margin =
    runtime_alignments.front().score >=
    base_score_threshold + param_.runtime_multistart.trigger_score_margin;
  const bool base_large_translation =
    base_candidate.initial_to_result_distance_m >
    param_.runtime_multistart.trigger_initial_to_result_distance_m;
  const bool base_large_yaw =
    std::abs(base_candidate.innovation_yaw_rad) * 180.0 / M_PI >
    param_.runtime_multistart.trigger_yaw_delta_deg;
  const bool periodic_tier1_refresh =
    param_.runtime_multistart.enable && runtime_stamp_enabled &&
    should_refresh_tracking_tier1(
      sensor_ros_time.seconds(), runtime_last_tier1_stamp_sec_,
      param_.runtime_multistart.tracking_tier1_period_sec);
  const bool periodic_along_health_refresh =
    param_.runtime_multistart.enable && runtime_stamp_enabled &&
    should_refresh_tracking_tier1(
      sensor_ros_time.seconds(), runtime_last_along_health_stamp_sec_,
      param_.runtime_multistart.tracking_along_health_period_sec);
  const bool should_expand_candidates =
    param_.runtime_multistart.enable && runtime_stamp_enabled &&
    (!base_candidate.converged || base_large_translation || base_large_yaw ||
     !base_score_has_margin || periodic_tier1_refresh || runtime_recovery_active_);

  if (should_expand_candidates || periodic_along_health_refresh) {
    const auto & tracking_indices = should_expand_candidates ? small_tier_indices : along_health_indices;
    for (const auto index : tracking_indices) {
      if (!run_runtime_alignment(index)) {
        return false;
      }
    }
    if (should_expand_candidates && !small_tier_indices.empty()) {
      runtime_last_tier1_stamp_sec_ = sensor_ros_time.seconds();
    }
    if (!should_expand_candidates && !along_health_indices.empty()) {
      along_health_evaluated = true;
      runtime_last_along_health_stamp_sec_ = sensor_ros_time.seconds();
    }
    RuntimeCandidateScoringOptions tier1_scoring_options = make_runtime_scoring_options(param_);
    RuntimeCandidateSelection tier1_selection =
      select_runtime_candidate(runtime_candidates, tier1_scoring_options);
    small_tier_ambiguous =
      runtime_small_tier_is_ambiguous(runtime_candidates, tier1_selection, runtime_tier_options);
    const double seconds_since_last_far_tier =
      runtime_last_far_tier_stamp_sec_ < 0.0
        ? std::numeric_limits<double>::infinity()
        : sensor_ros_time.seconds() - runtime_last_far_tier_stamp_sec_;
    const bool should_run_far_tier =
      !far_tier_indices.empty() &&
      should_evaluate_far_runtime_tier(
        runtime_candidates, tier1_selection, runtime_tier_options, runtime_rejected_scan_streak_,
        runtime_recovery_active_, seconds_since_last_far_tier, small_tier_ambiguous,
        runtime_scans_since_last_far_tier_);
    if (should_run_far_tier) {
      tier2_evaluated = true;
      runtime_last_far_tier_stamp_sec_ = sensor_ros_time.seconds();
      runtime_scans_since_last_far_tier_ = 0;
      if (runtime_recovery_active_) {
        tier2_trigger_reason = "recovery_active";
      } else if (runtime_rejected_scan_streak_ > 0) {
        tier2_trigger_reason = "rejected_scan_streak";
      } else if (!tier1_selection.has_selected_candidate) {
        tier2_trigger_reason = "small_tier_rejected";
      } else if (small_tier_ambiguous) {
        tier2_trigger_reason = "small_tier_ambiguous";
      } else if (
        param_.runtime_multistart.tracking_far_tier_period_sec > 0.0 &&
        seconds_since_last_far_tier >= param_.runtime_multistart.tracking_far_tier_period_sec) {
        tier2_trigger_reason = "tracking_far_tier_periodic";
      } else {
        tier2_trigger_reason = "small_tier_selection_missing";
      }
      for (const auto index : far_tier_indices) {
        if (!run_runtime_alignment(index)) {
          return false;
        }
      }
    }
  }

  RuntimeCandidateScoringOptions runtime_scoring_options = make_runtime_scoring_options(param_);
  if (runtime_alignments.size() == 1) {
    // Single-start frames keep the original NDT acceptance semantics.  Runtime prior gates are
    // only for choosing between multiple basins; applying them to the only candidate creates
    // avoidable coverage holes and can prevent the predictor from recovering after startup.
    disable_runtime_selection_gates_for_single_start(runtime_scoring_options);
  }
  RuntimeCandidateSelection runtime_selection =
    select_runtime_candidate(runtime_candidates, runtime_scoring_options);
  const RuntimeCandidateSpreadCovariance runtime_spread_covariance =
    estimate_runtime_candidate_spread_covariance(
      runtime_candidates, runtime_selection, runtime_scoring_options,
      param_.runtime_multistart.ambiguity_score_margin);
  auto selected_alignment_iter = std::find_if(
    runtime_alignments.begin(), runtime_alignments.end(),
    [&runtime_selection](const RuntimeAlignment & item) {
      return runtime_selection.has_selected_candidate &&
             item.index == runtime_selection.selected_candidate_index;
    });
  const RuntimeCandidate * selected_runtime_candidate =
    runtime_selection.has_selected_candidate
      ? find_runtime_candidate_by_index(runtime_candidates, runtime_selection.selected_candidate_index)
      : nullptr;
  const bool recovery_attempt =
    tier2_evaluated || runtime_recovery_active_ || runtime_rejected_scan_streak_ > 0 ||
    !runtime_selection.has_selected_candidate;
  const bool selected_stable =
    selected_runtime_candidate != nullptr &&
    runtime_candidate_is_stable_for_recovery(*selected_runtime_candidate, runtime_tier_options);
  const RuntimeRecoveryState recovery_state = update_runtime_recovery_state(
    runtime_selection.has_selected_candidate, selected_stable, recovery_attempt,
    runtime_recovery_active_, runtime_recovery_stable_frames_, runtime_rejected_scan_streak_,
    std::max(1, param_.runtime_multistart.recovery_stable_required_frames));
  runtime_rejected_scan_streak_ = recovery_state.rejected_scan_streak;
  runtime_recovery_active_ = recovery_state.recovery_active;
  runtime_recovery_stable_frames_ = recovery_state.recovery_stable_frames;
  const bool recovery_verified = recovery_state.recovery_verified;

  if (param_.runtime_multistart.enable && runtime_multistart_debug_pub_ != nullptr) {
    std_msgs::msg::String debug_msg;
    std::ostringstream payload;
    payload << std::fixed << std::setprecision(6);
    payload << "{\"stamp_sec\":" << sensor_ros_time.seconds()
            << ",\"reason\":\"runtime_scored_multistart\""
            << ",\"uses_gnss_or_gt\":false"
            << ",\"candidate_count\":" << runtime_alignments.size()
            << ",\"along_health_evaluated\":" << (along_health_evaluated ? "true" : "false")
            << ",\"tier2_evaluated\":" << (tier2_evaluated ? "true" : "false")
            << ",\"tier2_trigger_reason\":\"" << tier2_trigger_reason << "\""
            << ",\"small_tier_ambiguous\":" << (small_tier_ambiguous ? "true" : "false")
            << ",\"spread_covariance_ambiguous\":"
            << (runtime_spread_covariance.ambiguous ? "true" : "false")
            << ",\"spread_covariance_contender_count\":"
            << runtime_spread_covariance.contender_count
            << ",\"spread_covariance_along_m2\":"
            << runtime_spread_covariance.along_variance_m2
            << ",\"spread_covariance_cross_m2\":"
            << runtime_spread_covariance.cross_variance_m2
            << ",\"scans_since_last_far_tier\":" << runtime_scans_since_last_far_tier_
            << ",\"rejected_scan_streak\":" << runtime_rejected_scan_streak_
            << ",\"recovery_active\":" << (runtime_recovery_active_ ? "true" : "false")
            << ",\"recovery_verified\":" << (recovery_verified ? "true" : "false")
            << ",\"recovery_verified_stable_frames\":" << runtime_recovery_stable_frames_
            << ",\"has_selected_candidate\":"
            << (runtime_selection.has_selected_candidate ? "true" : "false")
            << ",\"selected_candidate_index\":";
    if (runtime_selection.has_selected_candidate) {
      payload << runtime_selection.selected_candidate_index;
    } else {
      payload << "null";
    }
    payload << ",\"candidates\":[";
    for (std::size_t i = 0; i < runtime_alignments.size(); ++i) {
      const auto & alignment = runtime_alignments[i];
      const auto & candidate_score = runtime_selection.candidate_scores[i];
      if (i > 0) {
        payload << ",";
      }
      payload << "{\"index\":" << alignment.index << ",\"offset_along_m\":"
              << alignment.offset_along_m << ",\"offset_cross_m\":" << alignment.offset_cross_m
              << ",\"offset_yaw_deg\":" << alignment.offset_yaw_deg
              << ",\"tier\":\""
              << (is_far_runtime_candidate(alignment.candidate, runtime_tier_options) ? "far"
                                                                                     : "small")
              << "\""
              << ",\"transform_probability\":" << alignment.ndt_result.transform_probability
              << ",\"nearest_voxel_transformation_likelihood\":"
              << alignment.ndt_result.nearest_voxel_transformation_likelihood
              << ",\"iteration_num\":" << alignment.ndt_result.iteration_num
              << ",\"converged\":" << (alignment.candidate.converged ? "true" : "false")
              << ",\"initial_to_result_distance_m\":"
              << alignment.candidate.initial_to_result_distance_m
              << ",\"result_x\":" << alignment.result_pose.position.x
              << ",\"result_y\":" << alignment.result_pose.position.y
              << ",\"result_z\":" << alignment.result_pose.position.z
              << ",\"result_yaw_deg\":" << yaw_from_pose(alignment.result_pose) * 180.0 / M_PI
              << ",\"innovation_along_m\":" << alignment.candidate.innovation_along_m
              << ",\"innovation_cross_m\":" << alignment.candidate.innovation_cross_m
              << ",\"innovation_yaw_deg\":"
              << alignment.candidate.innovation_yaw_rad * 180.0 / M_PI
              << ",\"total_score\":";
      if (std::isfinite(candidate_score.total_score)) {
        payload << candidate_score.total_score;
      } else {
        payload << "null";
      }
      payload << ",\"reject_reason\":\"" << candidate_score.reject_reason << "\"}";
    }
    payload << "]}";
    debug_msg.data = payload.str();
    runtime_multistart_debug_pub_->publish(debug_msg);
  }

  if (!runtime_selection.has_selected_candidate || selected_alignment_iter == runtime_alignments.end()) {
    std::stringstream message;
    message << "Runtime multi-start rejected all " << runtime_alignments.size() << " candidates.";
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    return false;
  }

  const RuntimeAlignment & selected_alignment = *selected_alignment_iter;
  const Eigen::Matrix4f initial_pose_matrix = selected_alignment.initial_pose_matrix;
  const auto output_cloud = selected_alignment.output_cloud;
  const pclomp::NdtResult ndt_result = selected_alignment.ndt_result;
  const geometry_msgs::msg::Pose result_pose_msg = selected_alignment.result_pose;
  const std::vector<geometry_msgs::msg::Pose> transformation_msg_array =
    selected_alignment.transformation_msg_array;

  geometry_msgs::msg::PoseArray runtime_multi_initial_pose_msg;
  geometry_msgs::msg::PoseArray runtime_multi_ndt_pose_msg;
  runtime_multi_initial_pose_msg.header.stamp = sensor_ros_time;
  runtime_multi_initial_pose_msg.header.frame_id = param_.frame.map_frame;
  runtime_multi_ndt_pose_msg.header = runtime_multi_initial_pose_msg.header;
  for (const auto & alignment : runtime_alignments) {
    runtime_multi_initial_pose_msg.poses.push_back(alignment.initial_pose);
    runtime_multi_ndt_pose_msg.poses.push_back(alignment.result_pose);
  }
  if (param_.runtime_multistart.enable && runtime_alignments.size() > 1) {
    multi_initial_pose_pub_->publish(runtime_multi_initial_pose_msg);
    multi_ndt_pose_pub_->publish(runtime_multi_ndt_pose_msg);
  }

  // check iteration_num
  diagnostics_scan_points_->add_key_value("iteration_num", ndt_result.iteration_num);
  const bool is_ok_iteration_num = (ndt_result.iteration_num < ndt_ptr_->getMaximumIterations());
  if (!is_ok_iteration_num) {
    std::stringstream message;
    message << "The number of iterations has reached its upper limit. The number of iterations: "
            << ndt_result.iteration_num << ", Limit: " << ndt_ptr_->getMaximumIterations() << ".";
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }

  // check local_optimal_solution_oscillation_num
  constexpr int oscillation_num_threshold = 10;
  const int oscillation_num = count_oscillation(transformation_msg_array);
  diagnostics_scan_points_->add_key_value(
    "local_optimal_solution_oscillation_num", oscillation_num);
  const bool is_local_optimal_solution_oscillation = (oscillation_num > oscillation_num_threshold);
  if (is_local_optimal_solution_oscillation) {
    std::stringstream message;
    message << "There is a possibility of oscillation in a local minimum";
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }

  // check score
  diagnostics_scan_points_->add_key_value(
    "transform_probability", ndt_result.transform_probability);
  diagnostics_scan_points_->add_key_value(
    "nearest_voxel_transformation_likelihood", ndt_result.nearest_voxel_transformation_likelihood);
  double score = 0.0;
  double score_threshold = 0.0;
  if (param_.score_estimation.converged_param_type == ConvergedParamType::TRANSFORM_PROBABILITY) {
    score = ndt_result.transform_probability;
    score_threshold = param_.score_estimation.converged_param_transform_probability;
  } else if (
    param_.score_estimation.converged_param_type ==
    ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD) {
    score = ndt_result.nearest_voxel_transformation_likelihood;
    score_threshold =
      param_.score_estimation.converged_param_nearest_voxel_transformation_likelihood;
  } else {
    std::stringstream message;
    message << "Unknown converged param type. Please check `score_estimation.converged_param_type`";
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());
    return false;
  }

  // check score diff
  const std::vector<float> & tp_array = ndt_result.transform_probability_array;
  if (static_cast<int>(tp_array.size()) != ndt_result.iteration_num + 1) {
    // only publish warning to /diagnostics, not skip publishing pose
    std::stringstream message;
    message << "transform_probability_array size is not equal to iteration_num + 1."
            << " transform_probability_array size: " << tp_array.size()
            << ", iteration_num: " << ndt_result.iteration_num;
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  } else {
    const float diff = tp_array.back() - tp_array.front();
    diagnostics_scan_points_->add_key_value("transform_probability_diff", diff);
    diagnostics_scan_points_->add_key_value("transform_probability_before", tp_array.front());
  }
  const std::vector<float> & nvtl_array = ndt_result.nearest_voxel_transformation_likelihood_array;
  if (static_cast<int>(nvtl_array.size()) != ndt_result.iteration_num + 1) {
    // only publish warning to /diagnostics, not skip publishing pose
    std::stringstream message;
    message
      << "nearest_voxel_transformation_likelihood_array size is not equal to iteration_num + 1."
      << " nearest_voxel_transformation_likelihood_array size: " << nvtl_array.size()
      << ", iteration_num: " << ndt_result.iteration_num;
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  } else {
    const float diff = nvtl_array.back() - nvtl_array.front();
    diagnostics_scan_points_->add_key_value("nearest_voxel_transformation_likelihood_diff", diff);
    diagnostics_scan_points_->add_key_value(
      "nearest_voxel_transformation_likelihood_before", nvtl_array.front());
  }

  bool is_ok_score = (score > score_threshold);
  if (!is_ok_score) {
    std::stringstream message;
    message << "Score is below the threshold. Score: " << score
            << ", Threshold: " << score_threshold;
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
  }

  // check is_converged
  bool is_converged = (is_ok_iteration_num || is_local_optimal_solution_oscillation) && is_ok_score;

  // covariance estimation
  const Eigen::Quaterniond map_to_base_link_quat = Eigen::Quaterniond(
    result_pose_msg.orientation.w, result_pose_msg.orientation.x, result_pose_msg.orientation.y,
    result_pose_msg.orientation.z);
  const Eigen::Matrix3d map_to_base_link_rotation =
    map_to_base_link_quat.normalized().toRotationMatrix();

  std::array<double, 36> ndt_covariance =
    rotate_covariance(param_.covariance.output_pose_covariance, map_to_base_link_rotation);
  if (
    param_.covariance.covariance_estimation.covariance_estimation_type !=
    CovarianceEstimationType::FIXED_VALUE) {
    const Eigen::Matrix2d estimated_covariance_2d =
      estimate_covariance(ndt_result, initial_pose_matrix, sensor_ros_time);
    const Eigen::Matrix2d estimated_covariance_2d_scaled =
      estimated_covariance_2d * param_.covariance.covariance_estimation.scale_factor;
    const double default_cov_xx = param_.covariance.output_pose_covariance[0];
    const double default_cov_yy = param_.covariance.output_pose_covariance[7];
    const Eigen::Matrix2d estimated_covariance_2d_adj = pclomp::adjust_diagonal_covariance(
      estimated_covariance_2d_scaled, ndt_result.pose, default_cov_xx, default_cov_yy);
    ndt_covariance[0 + 6 * 0] = estimated_covariance_2d_adj(0, 0);
    ndt_covariance[1 + 6 * 1] = estimated_covariance_2d_adj(1, 1);
    ndt_covariance[1 + 6 * 0] = estimated_covariance_2d_adj(1, 0);
    ndt_covariance[0 + 6 * 1] = estimated_covariance_2d_adj(0, 1);
  }
  if (runtime_spread_covariance.ambiguous) {
    Eigen::Matrix2d body_to_map;
    body_to_map << prior_forward_x, prior_lateral_x, prior_forward_y, prior_lateral_y;
    Eigen::Matrix2d spread_covariance_body = Eigen::Matrix2d::Zero();
    spread_covariance_body(0, 0) = runtime_spread_covariance.along_variance_m2;
    spread_covariance_body(1, 1) = runtime_spread_covariance.cross_variance_m2;
    const Eigen::Matrix2d spread_covariance_map =
      body_to_map * spread_covariance_body * body_to_map.transpose();
    ndt_covariance[0 + 6 * 0] += spread_covariance_map(0, 0);
    ndt_covariance[1 + 6 * 1] += spread_covariance_map(1, 1);
    ndt_covariance[1 + 6 * 0] += spread_covariance_map(1, 0);
    ndt_covariance[0 + 6 * 1] += spread_covariance_map(0, 1);
    diagnostics_scan_points_->add_key_value(
      "runtime_spread_covariance_along_m2", runtime_spread_covariance.along_variance_m2);
    diagnostics_scan_points_->add_key_value(
      "runtime_spread_covariance_cross_m2", runtime_spread_covariance.cross_variance_m2);
  }
  // check distance_initial_to_result
  const auto distance_initial_to_result = static_cast<double>(autoware::localization_util::norm(
    selected_alignment.initial_pose.position, result_pose_msg.position));
  diagnostics_scan_points_->add_key_value("distance_initial_to_result", distance_initial_to_result);
  if (!is_initial_to_result_distance_valid(
        distance_initial_to_result, param_.validation.initial_to_result_distance_tolerance_m)) {
    std::stringstream message;
    message << "distance_initial_to_result is too large (" << distance_initial_to_result
            << " [m]).";
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    is_converged = false;
  }

  // check execution_time
  const auto exe_end_time = std::chrono::system_clock::now();
  const auto duration_micro_sec =
    std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count();
  const auto exe_time = static_cast<float>(duration_micro_sec) / 1000.0f;
  diagnostics_scan_points_->add_key_value("execution_time", exe_time);
  if (exe_time > param_.validation.critical_upper_bound_exe_time_ms) {
    std::stringstream message;
    message << "NDT exe time is too long (took " << exe_time << " [ms]).";
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }

  // publish
  const rclcpp::Time output_pose_ros_time =
    apply_output_pose_time_offset(sensor_ros_time, param_.output_pose_time_offset_sec);
  diagnostics_scan_points_->add_key_value(
    "output_pose_time_offset_sec", param_.output_pose_time_offset_sec);
  diagnostics_scan_points_->add_key_value(
    "output_pose_time_stamp", output_pose_ros_time.nanoseconds());
  geometry_msgs::msg::PoseWithCovarianceStamped selected_initial_pose_with_covariance =
    interpolation_result.interpolated_pose;
  selected_initial_pose_with_covariance.pose.pose = selected_alignment.initial_pose;
  initial_pose_with_covariance_pub_->publish(selected_initial_pose_with_covariance);
  exe_time_pub_->publish(make_float32_stamped(output_pose_ros_time, exe_time));
  transform_probability_pub_->publish(
    make_float32_stamped(output_pose_ros_time, ndt_result.transform_probability));
  nearest_voxel_transformation_likelihood_pub_->publish(
    make_float32_stamped(output_pose_ros_time, ndt_result.nearest_voxel_transformation_likelihood));
  iteration_num_pub_->publish(make_int32_stamped(output_pose_ros_time, ndt_result.iteration_num));
  publish_tf(output_pose_ros_time, result_pose_msg);
  publish_pose(output_pose_ros_time, result_pose_msg, ndt_covariance, is_converged);
  publish_marker(output_pose_ros_time, transformation_msg_array);
  publish_initial_to_result(
    output_pose_ros_time, result_pose_msg, selected_initial_pose_with_covariance,
    interpolation_result.old_pose, interpolation_result.new_pose);

  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_map_ptr(
    new pcl::PointCloud<PointSource>);
  autoware_utils_pcl::transform_pointcloud(
    *sensor_points_in_baselink_frame, *sensor_points_in_map_ptr, ndt_result.pose);
  publish_point_cloud(sensor_ros_time, param_.frame.map_frame, sensor_points_in_map_ptr);

  // check each of point score
  const float lower_nvs = 1.0f;
  const float upper_nvs = 3.5f;
  if (voxel_score_points_pub_->get_subscription_count() > 0) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr nvs_points_in_map_ptr_rgb{
      new pcl::PointCloud<pcl::PointXYZRGB>};
    nvs_points_in_map_ptr_rgb =
      visualize_point_score(sensor_points_in_map_ptr, lower_nvs, upper_nvs);
    sensor_msgs::msg::PointCloud2 nvs_points_msg_in_map;
    pcl::toROSMsg(*nvs_points_in_map_ptr_rgb, nvs_points_msg_in_map);
    nvs_points_msg_in_map.header.stamp = sensor_ros_time;
    nvs_points_msg_in_map.header.frame_id = param_.frame.map_frame;
    voxel_score_points_pub_->publish(nvs_points_msg_in_map);
  }

  // whether use no ground points to calculate score
  if (param_.score_estimation.no_ground_points.enable) {
    // remove ground
    pcl::shared_ptr<pcl::PointCloud<PointSource>> no_ground_points_in_map_ptr(
      new pcl::PointCloud<PointSource>);
    for (std::size_t i = 0; i < sensor_points_in_map_ptr->size(); i++) {
      const float point_z = sensor_points_in_map_ptr->points[i].z;  // NOLINT
      if (
        point_z - matrix4f_to_pose(ndt_result.pose).position.z >
        param_.score_estimation.no_ground_points.z_margin_for_ground_removal) {
        no_ground_points_in_map_ptr->points.push_back(sensor_points_in_map_ptr->points[i]);
      }
    }
    // pub remove-ground points
    sensor_msgs::msg::PointCloud2 no_ground_points_msg_in_map;
    pcl::toROSMsg(*no_ground_points_in_map_ptr, no_ground_points_msg_in_map);
    no_ground_points_msg_in_map.header.stamp = sensor_ros_time;
    no_ground_points_msg_in_map.header.frame_id = param_.frame.map_frame;
    no_ground_points_aligned_pose_pub_->publish(no_ground_points_msg_in_map);
    // calculate score
    const auto no_ground_transform_probability = static_cast<float>(
      ndt_ptr_->calculateTransformationProbability(*no_ground_points_in_map_ptr));
    const auto no_ground_nearest_voxel_transformation_likelihood = static_cast<float>(
      ndt_ptr_->calculateNearestVoxelTransformationLikelihood(*no_ground_points_in_map_ptr));
    // pub score
    no_ground_transform_probability_pub_->publish(
      make_float32_stamped(sensor_ros_time, no_ground_transform_probability));
    no_ground_nearest_voxel_transformation_likelihood_pub_->publish(
      make_float32_stamped(sensor_ros_time, no_ground_nearest_voxel_transformation_likelihood));
  }

  return is_converged;
}

void NDTScanMatcher::transform_sensor_measurement(
  const std::string & source_frame, const std::string & target_frame,
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_input_ptr,
  pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_output_ptr)
{
  if (source_frame == target_frame) {
    sensor_points_output_ptr = sensor_points_input_ptr;
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    throw;
  }

  const geometry_msgs::msg::PoseStamped target_to_source_pose_stamped =
    autoware_utils_geometry::transform2pose(transform);
  const Eigen::Matrix4f base_to_sensor_matrix =
    pose_to_matrix4f(target_to_source_pose_stamped.pose);
  autoware_utils_pcl::transform_pointcloud(
    *sensor_points_input_ptr, *sensor_points_output_ptr, base_to_sensor_matrix);
}

void NDTScanMatcher::publish_tf(
  const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg)
{
  geometry_msgs::msg::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = param_.frame.map_frame;
  result_pose_stamped_msg.pose = result_pose_msg;
  tf2_broadcaster_.sendTransform(
    autoware_utils_geometry::pose2transform(result_pose_stamped_msg, param_.frame.ndt_base_frame));
}

void NDTScanMatcher::publish_pose(
  const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg,
  const std::array<double, 36> & ndt_covariance, const bool is_converged)
{
  geometry_msgs::msg::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = param_.frame.map_frame;
  result_pose_stamped_msg.pose = result_pose_msg;

  geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.stamp = sensor_ros_time;
  result_pose_with_cov_msg.header.frame_id = param_.frame.map_frame;
  result_pose_with_cov_msg.pose.pose = result_pose_msg;
  result_pose_with_cov_msg.pose.covariance = ndt_covariance;

  if (is_converged) {
    ndt_pose_pub_->publish(result_pose_stamped_msg);
    ndt_pose_with_covariance_pub_->publish(result_pose_with_cov_msg);
  }
}

void NDTScanMatcher::publish_point_cloud(
  const rclcpp::Time & sensor_ros_time, const std::string & frame_id,
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_in_map_ptr)
{
  sensor_msgs::msg::PointCloud2 sensor_points_msg_in_map;
  pcl::toROSMsg(*sensor_points_in_map_ptr, sensor_points_msg_in_map);
  sensor_points_msg_in_map.header.stamp = sensor_ros_time;
  sensor_points_msg_in_map.header.frame_id = frame_id;
  sensor_aligned_pose_pub_->publish(sensor_points_msg_in_map);
}

void NDTScanMatcher::publish_marker(
  const rclcpp::Time & sensor_ros_time, const std::vector<geometry_msgs::msg::Pose> & pose_array)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = sensor_ros_time;
  marker.header.frame_id = param_.frame.map_frame;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale = autoware_utils_visualization::create_marker_scale(0.3, 0.1, 0.1);
  int i = 0;
  marker.ns = "result_pose_matrix_array";
  marker.action = visualization_msgs::msg::Marker::ADD;
  for (const auto & pose_msg : pose_array) {
    marker.id = i++;
    marker.pose = pose_msg;
    marker.color = exchange_color_crc((1.0 * i) / 15.0);
    marker_array.markers.push_back(marker);
  }

  // TODO(Tier IV): delete old marker
  for (; i < ndt_ptr_->getMaximumIterations() + 2;) {
    marker.id = i++;
    marker.pose = geometry_msgs::msg::Pose();
    marker.color = exchange_color_crc(0);
    marker_array.markers.push_back(marker);
  }
  ndt_marker_pub_->publish(marker_array);
}

void NDTScanMatcher::publish_initial_to_result(
  const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_cov_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_old_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_new_msg)
{
  geometry_msgs::msg::PoseStamped initial_to_result_relative_pose_stamped;
  initial_to_result_relative_pose_stamped.pose = autoware_utils_geometry::inverse_transform_pose(
    result_pose_msg, initial_pose_cov_msg.pose.pose);
  initial_to_result_relative_pose_stamped.header.stamp = sensor_ros_time;
  initial_to_result_relative_pose_stamped.header.frame_id = param_.frame.map_frame;
  initial_to_result_relative_pose_pub_->publish(initial_to_result_relative_pose_stamped);

  const auto initial_to_result_distance = static_cast<float>(autoware::localization_util::norm(
    initial_pose_cov_msg.pose.pose.position, result_pose_msg.position));
  initial_to_result_distance_pub_->publish(
    make_float32_stamped(sensor_ros_time, initial_to_result_distance));

  const auto initial_to_result_distance_old = static_cast<float>(autoware::localization_util::norm(
    initial_pose_old_msg.pose.pose.position, result_pose_msg.position));
  initial_to_result_distance_old_pub_->publish(
    make_float32_stamped(sensor_ros_time, initial_to_result_distance_old));

  const auto initial_to_result_distance_new = static_cast<float>(autoware::localization_util::norm(
    initial_pose_new_msg.pose.pose.position, result_pose_msg.position));
  initial_to_result_distance_new_pub_->publish(
    make_float32_stamped(sensor_ros_time, initial_to_result_distance_new));
}

int NDTScanMatcher::count_oscillation(
  const std::vector<geometry_msgs::msg::Pose> & result_pose_msg_array)
{
  constexpr double inversion_vector_threshold = -0.9;

  int oscillation_cnt = 0;
  int max_oscillation_cnt = 0;

  for (size_t i = 2; i < result_pose_msg_array.size(); ++i) {
    const Eigen::Vector3d current_pose = point_to_vector3d(result_pose_msg_array.at(i).position);
    const Eigen::Vector3d prev_pose = point_to_vector3d(result_pose_msg_array.at(i - 1).position);
    const Eigen::Vector3d prev_prev_pose =
      point_to_vector3d(result_pose_msg_array.at(i - 2).position);
    const auto current_vec = (current_pose - prev_pose).normalized();
    const auto prev_vec = (prev_pose - prev_prev_pose).normalized();
    const double cosine_value = current_vec.dot(prev_vec);
    const bool oscillation = cosine_value < inversion_vector_threshold;
    if (oscillation) {
      oscillation_cnt++;  // count consecutive oscillation
    } else {
      oscillation_cnt = 0;  // reset
    }
    max_oscillation_cnt = std::max(max_oscillation_cnt, oscillation_cnt);
  }
  return max_oscillation_cnt;
}

Eigen::Matrix2d NDTScanMatcher::estimate_covariance(
  const pclomp::NdtResult & ndt_result, const Eigen::Matrix4f & initial_pose_matrix,
  const rclcpp::Time & sensor_ros_time)
{
  geometry_msgs::msg::PoseArray multi_ndt_result_msg;
  geometry_msgs::msg::PoseArray multi_initial_pose_msg;
  multi_ndt_result_msg.header.stamp = sensor_ros_time;
  multi_ndt_result_msg.header.frame_id = param_.frame.map_frame;
  multi_initial_pose_msg.header.stamp = sensor_ros_time;
  multi_initial_pose_msg.header.frame_id = param_.frame.map_frame;
  multi_ndt_result_msg.poses.push_back(matrix4f_to_pose(ndt_result.pose));
  multi_initial_pose_msg.poses.push_back(matrix4f_to_pose(initial_pose_matrix));

  if (
    param_.covariance.covariance_estimation.covariance_estimation_type ==
    CovarianceEstimationType::LAPLACE_APPROXIMATION) {
    return pclomp::estimate_xy_covariance_by_laplace_approximation(ndt_result.hessian);
  } else if (
    param_.covariance.covariance_estimation.covariance_estimation_type ==
    CovarianceEstimationType::MULTI_NDT) {
    const std::vector<Eigen::Matrix4f> poses_to_search = pclomp::propose_poses_to_search(
      ndt_result, param_.covariance.covariance_estimation.initial_pose_offset_model_x,
      param_.covariance.covariance_estimation.initial_pose_offset_model_y);
    const pclomp::ResultOfMultiNdtCovarianceEstimation result_of_multi_ndt_covariance_estimation =
      estimate_xy_covariance_by_multi_ndt(ndt_result, ndt_ptr_, poses_to_search);
    for (size_t i = 0; i < result_of_multi_ndt_covariance_estimation.ndt_initial_poses.size();
         i++) {
      multi_ndt_result_msg.poses.push_back(
        matrix4f_to_pose(result_of_multi_ndt_covariance_estimation.ndt_results[i].pose));
      multi_initial_pose_msg.poses.push_back(
        matrix4f_to_pose(result_of_multi_ndt_covariance_estimation.ndt_initial_poses[i]));
    }
    multi_ndt_pose_pub_->publish(multi_ndt_result_msg);
    multi_initial_pose_pub_->publish(multi_initial_pose_msg);
    return result_of_multi_ndt_covariance_estimation.covariance;
  } else if (
    param_.covariance.covariance_estimation.covariance_estimation_type ==
    CovarianceEstimationType::MULTI_NDT_SCORE) {
    const std::vector<Eigen::Matrix4f> poses_to_search = pclomp::propose_poses_to_search(
      ndt_result, param_.covariance.covariance_estimation.initial_pose_offset_model_x,
      param_.covariance.covariance_estimation.initial_pose_offset_model_y);
    const pclomp::ResultOfMultiNdtCovarianceEstimation
      result_of_multi_ndt_score_covariance_estimation = estimate_xy_covariance_by_multi_ndt_score(
        ndt_result, ndt_ptr_, poses_to_search, param_.covariance.covariance_estimation.temperature);
    for (const auto & sub_initial_pose_matrix : poses_to_search) {
      multi_initial_pose_msg.poses.push_back(matrix4f_to_pose(sub_initial_pose_matrix));
    }
    multi_initial_pose_pub_->publish(multi_initial_pose_msg);
    return result_of_multi_ndt_score_covariance_estimation.covariance;
  } else {
    return Eigen::Matrix2d::Identity() * param_.covariance.output_pose_covariance[0 + 6 * 0];
  }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr NDTScanMatcher::visualize_point_score(
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_in_map_ptr,
  const float & lower_nvs, const float & upper_nvs)
{
  pcl::PointCloud<pcl::PointXYZI> nvs_points_in_map_ptr_i;
  nvs_points_in_map_ptr_i =
    ndt_ptr_->calculateNearestVoxelScoreEachPoint(*sensor_points_in_map_ptr);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr nvs_points_in_map_ptr_rgb{
    new pcl::PointCloud<pcl::PointXYZRGB>};

  const float range = upper_nvs - lower_nvs;
  for (std::size_t i = 0; i < nvs_points_in_map_ptr_i.size(); i++) {
    pcl::PointXYZRGB point;
    point.x = nvs_points_in_map_ptr_i.points[i].x;
    point.y = nvs_points_in_map_ptr_i.points[i].y;
    point.z = nvs_points_in_map_ptr_i.points[i].z;
    std_msgs::msg::ColorRGBA color =
      exchange_color_crc((nvs_points_in_map_ptr_i.points[i].intensity - lower_nvs) / range);
    point.r = color.r * 255;
    point.g = color.g * 255;
    point.b = color.b * 255;
    nvs_points_in_map_ptr_rgb->points.push_back(point);
  }
  return nvs_points_in_map_ptr_rgb;
}

void NDTScanMatcher::add_regularization_pose(const rclcpp::Time & sensor_ros_time)
{
  ndt_ptr_->unsetRegularizationPose();
  std::optional<SmartPoseBuffer::InterpolateResult> interpolation_result_opt =
    regularization_pose_buffer_->interpolate(sensor_ros_time);
  if (!interpolation_result_opt) {
    return;
  }
  regularization_pose_buffer_->pop_old(sensor_ros_time);
  const SmartPoseBuffer::InterpolateResult & interpolation_result =
    interpolation_result_opt.value();
  const Eigen::Matrix4f pose = pose_to_matrix4f(interpolation_result.interpolated_pose.pose.pose);
  ndt_ptr_->setRegularizationPose(pose);
}

void NDTScanMatcher::service_trigger_node(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  const rclcpp::Time ros_time_now = this->now();

  diagnostics_trigger_node_->clear();
  diagnostics_trigger_node_->add_key_value("service_call_time_stamp", ros_time_now.nanoseconds());

  is_activated_ = req->data;
  if (is_activated_) {
    initial_pose_buffer_->clear();
  }
  res->success = true;

  diagnostics_trigger_node_->add_key_value("is_activated", static_cast<bool>(is_activated_));
  diagnostics_trigger_node_->add_key_value("is_succeed_service", res->success);
  diagnostics_trigger_node_->publish(ros_time_now);
}

void NDTScanMatcher::service_ndt_align(
  const autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
  autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res)
{
  const rclcpp::Time ros_time_now = this->now();

  diagnostics_ndt_align_->clear();

  diagnostics_ndt_align_->add_key_value("service_call_time_stamp", ros_time_now.nanoseconds());

  service_ndt_align_main(req, res);

  // check is_succeed_service
  bool is_succeed_service = res->success;
  diagnostics_ndt_align_->add_key_value("is_succeed_service", is_succeed_service);
  if (!is_succeed_service) {
    std::stringstream message;
    message << "ndt_align_service is failed.";
    diagnostics_ndt_align_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }

  diagnostics_ndt_align_->publish(ros_time_now);
}

void NDTScanMatcher::service_ndt_align_main(
  const autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
  autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res)
{
  const rclcpp::Time requested_pose_stamp(req->pose_with_covariance.header.stamp);
  const auto wait_timeout = std::chrono::duration<double>(
    std::max(0.0, param_.initial_pose_estimation.sensor_points_stamp_wait_timeout_sec));
  const auto wait_deadline = std::chrono::steady_clock::now() +
                             std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                               wait_timeout);
  auto latest_sensor_stamp_is_fresh = [&]() {
    std::lock_guard<std::mutex> lock(ndt_ptr_mtx_);
    return has_latest_sensor_points_stamp_ &&
           is_alignment_sensor_stamp_fresh(
             requested_pose_stamp, latest_sensor_points_stamp_,
             param_.initial_pose_estimation.sensor_points_stamp_tolerance_sec);
  };
  while (
    rclcpp::ok() && !latest_sensor_stamp_is_fresh() &&
    std::chrono::steady_clock::now() < wait_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  {
    std::lock_guard<std::mutex> lock(ndt_ptr_mtx_);
    diagnostics_ndt_align_->add_key_value(
      "requested_pose_stamp", requested_pose_stamp.nanoseconds());
    diagnostics_ndt_align_->add_key_value(
      "latest_sensor_points_stamp", latest_sensor_points_stamp_.nanoseconds());
    diagnostics_ndt_align_->add_key_value(
      "is_latest_sensor_points_stamp_fresh",
      has_latest_sensor_points_stamp_ &&
        is_alignment_sensor_stamp_fresh(
          requested_pose_stamp, latest_sensor_points_stamp_,
          param_.initial_pose_estimation.sensor_points_stamp_tolerance_sec));
  }

  // get TF from pose_frame to map_frame
  const std::string & target_frame = param_.frame.map_frame;
  const std::string & source_frame = req->pose_with_covariance.header.frame_id;

  geometry_msgs::msg::TransformStamped transform_s2t;
  try {
    transform_s2t = tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    // Note: Up to AWSIMv1.1.0, there is a known bug where the GNSS frame_id is incorrectly set to
    // "gnss_link" instead of "map". The ndt_align is designed to return identity when this issue
    // occurs. However, in the future, converting to a non-existent frame_id should be prohibited.

    diagnostics_ndt_align_->add_key_value("is_succeed_transform_initial_pose", false);

    std::stringstream message;
    message << "Please publish TF " << target_frame.c_str() << " to " << source_frame.c_str();
    diagnostics_ndt_align_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    res->success = false;
    return;
  }
  diagnostics_ndt_align_->add_key_value("is_succeed_transform_initial_pose", true);

  // transform pose_frame to map_frame
  auto initial_pose_msg_in_map_frame =
    autoware::localization_util::transform(req->pose_with_covariance, transform_s2t);
  initial_pose_msg_in_map_frame.header.stamp = req->pose_with_covariance.header.stamp;
  map_update_module_->update_map(
    initial_pose_msg_in_map_frame.pose.pose.position, diagnostics_ndt_align_);

  // mutex Map
  std::lock_guard<std::mutex> lock(ndt_ptr_mtx_);

  // check is_set_map_points
  bool is_set_map_points = (ndt_ptr_->getInputTarget() != nullptr);
  diagnostics_ndt_align_->add_key_value("is_set_map_points", is_set_map_points);
  if (!is_set_map_points) {
    std::stringstream message;
    message << "No InputTarget. Please check the map file and the map_loader service";
    diagnostics_ndt_align_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    res->success = false;
    return;
  }

  // check is_set_sensor_points
  bool is_set_sensor_points = (ndt_ptr_->getInputSource() != nullptr);
  diagnostics_ndt_align_->add_key_value("is_set_sensor_points", is_set_sensor_points);
  if (!is_set_sensor_points) {
    std::stringstream message;
    message << "No InputSource. Please check the input lidar topic";
    diagnostics_ndt_align_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    res->success = false;
    return;
  }

  // estimate initial pose
  const auto [pose_with_covariance, score] = align_pose(initial_pose_msg_in_map_frame);

  // check reliability of initial pose result
  res->reliable =
    (param_.score_estimation.converged_param_nearest_voxel_transformation_likelihood < score);
  if (!res->reliable) {
    RCLCPP_WARN_STREAM(
      this->get_logger(), "Initial Pose Estimation is Unstable. Score is " << score);
  }
  res->success = true;
  res->pose_with_covariance = pose_with_covariance;
  res->pose_with_covariance.pose.covariance = req->pose_with_covariance.pose.covariance;
}

std::tuple<geometry_msgs::msg::PoseWithCovarianceStamped, double> NDTScanMatcher::align_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov)
{
  autoware::localization_util::output_pose_with_cov_to_log(
    get_logger(), "align_pose_input", initial_pose_with_cov);

  const auto base_rpy = autoware::localization_util::get_rpy(initial_pose_with_cov);
  const Eigen::Map<const autoware::localization_util::RowMatrixXd> covariance = {
    initial_pose_with_cov.pose.covariance.data(), 6, 6};
  const double stddev_x = std::sqrt(covariance(0, 0));
  const double stddev_y = std::sqrt(covariance(1, 1));
  const double stddev_z = std::sqrt(covariance(2, 2));
  const double stddev_roll = std::sqrt(covariance(3, 3));
  const double stddev_pitch = std::sqrt(covariance(4, 4));

  // Since only yaw is uniformly sampled, we define the mean and standard deviation for the others.
  const std::vector<double> sample_mean{
    initial_pose_with_cov.pose.pose.position.x,  // trans_x
    initial_pose_with_cov.pose.pose.position.y,  // trans_y
    initial_pose_with_cov.pose.pose.position.z,  // trans_z
    base_rpy.x,                                  // angle_x
    base_rpy.y                                   // angle_y
  };
  const std::vector<double> sample_stddev{stddev_x, stddev_y, stddev_z, stddev_roll, stddev_pitch};

  // Optimizing (x, y, z, roll, pitch, yaw) 6 dimensions.
  TreeStructuredParzenEstimator tpe(
    TreeStructuredParzenEstimator::Direction::MAXIMIZE,
    param_.initial_pose_estimation.n_startup_trials, sample_mean, sample_stddev);

  std::vector<Particle> particle_array;
  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();

  // publish the estimated poses in 20 times to see the progress and to avoid dropping data
  visualization_msgs::msg::MarkerArray marker_array;
  constexpr int64_t publish_num = 20;
  const int64_t publish_interval =
    std::max<int64_t>(1, param_.initial_pose_estimation.particles_num / publish_num);
  const std::size_t deterministic_offset_count =
    param_.initial_pose_estimation.deterministic_offsets_enable
      ? count_complete_initial_pose_offsets(
          param_.initial_pose_estimation.deterministic_offset_along_m,
          param_.initial_pose_estimation.deterministic_offset_cross_m,
          param_.initial_pose_estimation.deterministic_offset_yaw_deg)
      : 0U;

  for (int64_t i = 0; i < param_.initial_pose_estimation.particles_num; i++) {
    geometry_msgs::msg::Pose initial_pose;
    if (static_cast<std::size_t>(i) < deterministic_offset_count) {
      initial_pose = apply_initial_pose_offset(
        initial_pose_with_cov.pose.pose,
        param_.initial_pose_estimation.deterministic_offset_along_m[static_cast<std::size_t>(i)],
        param_.initial_pose_estimation.deterministic_offset_cross_m[static_cast<std::size_t>(i)],
        param_.initial_pose_estimation.deterministic_offset_yaw_deg[static_cast<std::size_t>(i)]);
    } else {
      TreeStructuredParzenEstimator::Input input =
        i == 0 && param_.initial_pose_estimation.include_initial_pose
          ? TreeStructuredParzenEstimator::Input{
              sample_mean[0], sample_mean[1], sample_mean[2], sample_mean[3], sample_mean[4],
              base_rpy.z}
          : tpe.get_next_input();
      if (param_.initial_pose_estimation.force_initial_yaw) {
        input[5] = base_rpy.z;
      }

      initial_pose.position.x = input[0];
      initial_pose.position.y = input[1];
      initial_pose.position.z = input[2];
      geometry_msgs::msg::Vector3 init_rpy;
      init_rpy.x = input[3];
      init_rpy.y = input[4];
      init_rpy.z = input[5];
      tf2::Quaternion tf_quaternion;
      tf_quaternion.setRPY(init_rpy.x, init_rpy.y, init_rpy.z);
      initial_pose.orientation = tf2::toMsg(tf_quaternion);
    }

    const Eigen::Matrix4f initial_pose_matrix = pose_to_matrix4f(initial_pose);
    ndt_ptr_->align(*output_cloud, initial_pose_matrix);
    const pclomp::NdtResult ndt_result = ndt_ptr_->getResult();

    Particle particle(
      initial_pose, matrix4f_to_pose(ndt_result.pose),
      ndt_result.nearest_voxel_transformation_likelihood, ndt_result.iteration_num);
    particle_array.push_back(particle);
    push_debug_markers(marker_array, get_clock()->now(), param_.frame.map_frame, particle, i);
    if (
      (i + 1) % publish_interval == 0 || (i + 1) == param_.initial_pose_estimation.particles_num) {
      ndt_monte_carlo_initial_pose_marker_pub_->publish(marker_array);
      marker_array.markers.clear();
    }

    const geometry_msgs::msg::Pose pose = matrix4f_to_pose(ndt_result.pose);
    const geometry_msgs::msg::Vector3 rpy = autoware::localization_util::get_rpy(pose);

    TreeStructuredParzenEstimator::Input result(6);
    result[0] = pose.position.x;
    result[1] = pose.position.y;
    result[2] = pose.position.z;
    result[3] = rpy.x;
    result[4] = rpy.y;
    result[5] = rpy.z;
    tpe.add_trial(TreeStructuredParzenEstimator::Trial{result, ndt_result.transform_probability});

    auto sensor_points_in_map_ptr = std::make_shared<pcl::PointCloud<PointSource>>();
    autoware_utils_pcl::transform_pointcloud(
      *ndt_ptr_->getInputSource(), *sensor_points_in_map_ptr, ndt_result.pose);
    publish_point_cloud(
      initial_pose_with_cov.header.stamp, param_.frame.map_frame, sensor_points_in_map_ptr);
  }

  auto best_particle_ptr = std::max_element(
    std::begin(particle_array), std::end(particle_array),
    [](const Particle & lhs, const Particle & rhs) { return lhs.score < rhs.score; });

  geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
  const rclcpp::Time request_stamp(initial_pose_with_cov.header.stamp);
  const rclcpp::Time sensor_points_stamp =
    has_latest_sensor_points_stamp_ ? latest_sensor_points_stamp_ : request_stamp;
  result_pose_with_cov_msg.header.stamp = select_alignment_output_stamp(
    request_stamp, sensor_points_stamp, param_.initial_pose_estimation.use_sensor_points_stamp);
  result_pose_with_cov_msg.header.frame_id = param_.frame.map_frame;
  result_pose_with_cov_msg.pose.pose = best_particle_ptr->result_pose;
  if (param_.initial_pose_estimation.output_initial_yaw) {
    const auto output_rpy =
      autoware::localization_util::get_rpy(result_pose_with_cov_msg.pose.pose);
    tf2::Quaternion output_quaternion;
    output_quaternion.setRPY(output_rpy.x, output_rpy.y, base_rpy.z);
    result_pose_with_cov_msg.pose.pose.orientation = tf2::toMsg(output_quaternion);
  }

  autoware::localization_util::output_pose_with_cov_to_log(
    get_logger(), "align_pose_output", result_pose_with_cov_msg);
  diagnostics_ndt_align_->add_key_value("best_particle_score", best_particle_ptr->score);

  return std::make_tuple(result_pose_with_cov_msg, best_particle_ptr->score);
}

}  // namespace autoware::ndt_scan_matcher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::ndt_scan_matcher::NDTScanMatcher)
