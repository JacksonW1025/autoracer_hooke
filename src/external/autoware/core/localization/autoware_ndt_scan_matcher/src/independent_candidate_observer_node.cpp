// Copyright 2026
//
// Independent runtime NDT candidate observer.  This process does not share the
// live NDTScanMatcher object or its input path; it owns its own map cache and
// NDT instance, subscribes to scan/pose/GNSS topics, and publishes candidate
// JSON for offline/online selector experiments.

#include "autoware/ndt_scan_matcher/ndt_omp/multigrid_ndt_omp.h"

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <autoware_map_msgs/srv/get_differential_point_cloud_map.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>
#include <vector>

namespace
{
using GetDifferentialPointCloudMap = autoware_map_msgs::srv::GetDifferentialPointCloudMap;
using PointT = pcl::PointXYZ;
using NdtType = pclomp::MultiGridNormalDistributionsTransform<PointT, PointT>;

double stamp_to_sec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

double normalize_angle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double yaw_from_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

Eigen::Matrix4f pose_to_matrix(const geometry_msgs::msg::Pose & pose)
{
  const Eigen::Translation3f translation(
    static_cast<float>(pose.position.x), static_cast<float>(pose.position.y),
    static_cast<float>(pose.position.z));
  const Eigen::Quaternionf rotation(
    static_cast<float>(pose.orientation.w), static_cast<float>(pose.orientation.x),
    static_cast<float>(pose.orientation.y), static_cast<float>(pose.orientation.z));
  return (translation * rotation).matrix();
}

geometry_msgs::msg::Pose matrix_to_pose(const Eigen::Matrix4f & matrix)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = matrix(0, 3);
  pose.position.y = matrix(1, 3);
  pose.position.z = matrix(2, 3);
  const Eigen::Matrix3f rot = matrix.block<3, 3>(0, 0);
  const Eigen::Quaternionf q(rot);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

geometry_msgs::msg::Pose offset_pose(
  const geometry_msgs::msg::Pose & pose, const double along_m, const double cross_m,
  const double yaw_deg)
{
  geometry_msgs::msg::Pose out = pose;
  const double yaw = yaw_from_pose(pose);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  out.position.x += cos_yaw * along_m - sin_yaw * cross_m;
  out.position.y += sin_yaw * along_m + cos_yaw * cross_m;
  const double out_yaw = normalize_angle(yaw + yaw_deg * M_PI / 180.0);
  out.orientation.x = 0.0;
  out.orientation.y = 0.0;
  out.orientation.z = std::sin(out_yaw * 0.5);
  out.orientation.w = std::cos(out_yaw * 0.5);
  return out;
}

std::vector<double> parse_csv_doubles(const std::string & value, const std::vector<double> & fallback)
{
  std::vector<double> parsed;
  std::stringstream ss(value);
  std::string token;
  while (std::getline(ss, token, ',')) {
    try {
      const double item = std::stod(token);
      if (std::isfinite(item)) parsed.push_back(item);
    } catch (...) {
    }
  }
  return parsed.empty() ? fallback : parsed;
}

std::string json_string(const std::string & value)
{
  std::ostringstream out;
  out << '"';
  for (const char c : value) {
    if (c == '"' || c == '\\') {
      out << '\\' << c;
    } else if (c == '\n') {
      out << "\\n";
    } else {
      out << c;
    }
  }
  out << '"';
  return out.str();
}

struct Candidate
{
  std::size_t index{};
  double offset_along_m{};
  double offset_cross_m{};
  double offset_yaw_deg{};
  geometry_msgs::msg::Pose initial_pose{};
  geometry_msgs::msg::Pose result_pose{};
  int iteration_count{};
  int max_iterations{};
  bool converged{};
  bool hit_max_iteration{};
  double transform_probability{};
  double nvtl{};
  double initial_to_result_distance_m{};
  double initial_to_result_yaw_deg{};
  double innovation_along_m{};
  double innovation_cross_m{};
  double innovation_yaw_deg{};
  std::string rejection_reason{};
};

struct MapTile
{
  std::string id{};
  std::string path{};
  double x{};
  double y{};
};

template <typename T>
struct TimedValue
{
  double stamp_sec{};
  T value{};
};

bool parse_tile_origin_from_name(const std::string & name, double & x, double & y)
{
  const auto x_pos = name.find("_x");
  const auto y_pos = name.find("_y", x_pos == std::string::npos ? 0 : x_pos + 2);
  const auto suffix_pos = name.rfind(".pcd");
  if (x_pos == std::string::npos || y_pos == std::string::npos || suffix_pos == std::string::npos) {
    return false;
  }
  try {
    x = std::stod(name.substr(x_pos + 2, y_pos - (x_pos + 2)));
    y = std::stod(name.substr(y_pos + 2, suffix_pos - (y_pos + 2)));
  } catch (...) {
    return false;
  }
  return std::isfinite(x) && std::isfinite(y);
}

class IndependentNdtCandidateObserver : public rclcpp::Node
{
public:
  IndependentNdtCandidateObserver() : Node("independent_ndt_candidate_observer")
  {
    const auto points_topic =
      declare_parameter<std::string>("points_topic", "/sensing/lidar/concatenated/pointcloud_accumulated");
    const auto initial_pose_topic = declare_parameter<std::string>(
      "initial_pose_topic", "/localization/pose_twist_fusion_filter/pose_with_covariance");
    const auto main_ndt_pose_topic = declare_parameter<std::string>(
      "main_ndt_pose_topic", "/localization/pose_estimator/pose_with_covariance");
    gnss_topic_ =
      declare_parameter<std::string>("gnss_weak_prior_topic", "/sensing/gnss/pose_with_covariance");
    output_topic_ =
      declare_parameter<std::string>("output_topic", "/localization/candidate_observer/candidates");
    debug_topic_ =
      declare_parameter<std::string>("debug_topic", "/localization/candidate_observer/diagnostics");
    map_service_name_ =
      declare_parameter<std::string>("map_service", "/map/get_differential_pointcloud_map");
    map_source_ = declare_parameter<std::string>("map_source", "loaded_map_topic");
    map_topic_ = declare_parameter<std::string>("map_topic", "/debug/loaded_pointcloud_map");
    map_directory_ = declare_parameter<std::string>("map_directory", "");

    publish_min_period_sec_ = declare_parameter<double>("publish_min_period_sec", 5.0);
    alignment_wall_delay_ms_ = declare_parameter<int>("alignment_wall_delay_ms", 0);
    health_trigger_enable_ = declare_parameter<bool>("health_trigger_enable", false);
    health_trigger_min_period_sec_ = declare_parameter<double>("health_trigger_min_period_sec", 1.0);
    health_trigger_max_metric_age_sec_ =
      declare_parameter<double>("health_trigger_max_metric_age_sec", 0.3);
    health_trigger_i2r_m_ = declare_parameter<double>("health_trigger_i2r_m", 1.5);
    health_trigger_min_nvtl_ = declare_parameter<double>("health_trigger_min_nvtl", 1.3);
    health_trigger_max_iteration_count_ =
      declare_parameter<int>("health_trigger_max_iteration_count", 80);
    map_radius_m_ = declare_parameter<double>("map_radius_m", 150.0);
    map_update_distance_m_ = declare_parameter<double>("map_update_distance_m", 40.0);
    map_tile_resolution_m_ = declare_parameter<double>("map_tile_resolution_m", 20.0);
    map_tile_load_radius_margin_m_ = declare_parameter<double>("map_tile_load_radius_margin_m", 15.0);
    max_tiles_per_update_ = declare_parameter<int>("max_tiles_per_update", 240);
    map_voxel_leaf_size_m_ = declare_parameter<double>("map_voxel_leaf_size_m", 0.0);
    map_service_timeout_ms_ = declare_parameter<int>("map_service_timeout_ms", 5000);
    min_points_ = declare_parameter<int>("min_points", 200);
    scan_voxel_leaf_size_m_ = declare_parameter<double>("scan_voxel_leaf_size_m", 0.0);
    enable_alignment_ = declare_parameter<bool>("enable_alignment", true);
    enable_gnss_weak_prior_ = declare_parameter<bool>("enable_gnss_weak_prior", false);
    gnss_max_age_sec_ = declare_parameter<double>("gnss_weak_prior_max_age_sec", 0.5);
    gnss_sigma_m_ = declare_parameter<double>("gnss_weak_prior_sigma_m", 5.0);
    gnss_max_penalty_ = declare_parameter<double>("gnss_weak_prior_max_penalty", 8.0);
    min_nvtl_ = declare_parameter<double>("min_nearest_voxel_transformation_likelihood", 2.3);
    max_initial_to_result_m_ = declare_parameter<double>("max_initial_to_result_m", 5.0);
    max_candidates_per_scan_ = declare_parameter<int>("max_candidates_per_scan", 0);

    offset_along_m_ = parse_csv_doubles(
      declare_parameter<std::string>("offset_along_m", "-1.0,0.0,1.0"), {-1.0, 0.0, 1.0});
    offset_cross_m_ =
      parse_csv_doubles(declare_parameter<std::string>("offset_cross_m", "0.0"), {0.0});
    offset_yaw_deg_ =
      parse_csv_doubles(declare_parameter<std::string>("offset_yaw_deg", "0.0"), {0.0});

    pclomp::NdtParams ndt_params;
    ndt_params.trans_epsilon = declare_parameter<double>("ndt_trans_epsilon", 0.01);
    ndt_params.step_size = declare_parameter<double>("ndt_step_size", 0.1);
    ndt_params.resolution = static_cast<float>(declare_parameter<double>("ndt_resolution", 2.0));
    ndt_params.max_iterations = declare_parameter<int>("ndt_max_iterations", 40);
    ndt_params.num_threads = std::max<int>(
      1, static_cast<int>(declare_parameter<int>("ndt_num_threads", 1)));
    ndt_params.search_method = pclomp::KDTREE;
    ndt_params.regularization_scale_factor = 0.0F;
    ndt_.setParams(ndt_params);

    output_pub_ = create_publisher<std_msgs::msg::String>(output_topic_, rclcpp::QoS{10});
    debug_pub_ = create_publisher<std_msgs::msg::String>(debug_topic_, rclcpp::QoS{10});
    if (map_source_ == "service") {
      map_client_ = create_client<GetDifferentialPointCloudMap>(map_service_name_);
    } else if (map_source_ == "loaded_map_topic") {
      rclcpp::QoS map_qos{1};
      map_qos.reliable();
      map_qos.transient_local();
      map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        map_topic_, map_qos,
        std::bind(&IndependentNdtCandidateObserver::on_loaded_map, this, std::placeholders::_1));
    } else if (map_source_ != "pcd_tiles") {
      last_map_failure_reason_ = "unsupported_map_source";
    }

    rclcpp::QoS pose_qos{1};
    pose_qos.best_effort();
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic, pose_qos,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        latest_pose_ = *msg;
      });
    if (enable_gnss_weak_prior_) {
      gnss_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        gnss_topic_, pose_qos,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(pose_mutex_);
          latest_gnss_ = *msg;
        });
    }
    if (health_trigger_enable_) {
      main_nvtl_sub_ =
        create_subscription<autoware_internal_debug_msgs::msg::Float32Stamped>(
          "/nearest_voxel_transformation_likelihood", 10,
          [this](const autoware_internal_debug_msgs::msg::Float32Stamped::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(main_health_mutex_);
            latest_main_nvtl_ = TimedValue<double>{stamp_to_sec(msg->stamp), msg->data};
          });
      main_i2r_sub_ =
        create_subscription<autoware_internal_debug_msgs::msg::Float32Stamped>(
          "/initial_to_result_distance", 10,
          [this](const autoware_internal_debug_msgs::msg::Float32Stamped::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(main_health_mutex_);
            latest_main_i2r_ = TimedValue<double>{stamp_to_sec(msg->stamp), msg->data};
          });
      main_iteration_sub_ =
        create_subscription<autoware_internal_debug_msgs::msg::Int32Stamped>(
          "/iteration_num", 10,
          [this](const autoware_internal_debug_msgs::msg::Int32Stamped::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(main_health_mutex_);
            latest_main_iteration_ = TimedValue<int>{stamp_to_sec(msg->stamp), msg->data};
          });
      main_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        main_ndt_pose_topic, pose_qos,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(main_health_mutex_);
          latest_main_pose_stamp_sec_ = stamp_to_sec(msg->header.stamp);
        });
    }

    rclcpp::SensorDataQoS points_qos;
    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic, points_qos,
      std::bind(&IndependentNdtCandidateObserver::on_points, this, std::placeholders::_1));
  }

private:
  void on_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const double stamp_sec = stamp_to_sec(msg->header.stamp);
    const bool health_triggered = main_ndt_health_degraded(stamp_sec);
    if (last_publish_stamp_sec_.has_value()) {
      const double dt = stamp_sec - last_publish_stamp_sec_.value();
      const double min_period =
        health_triggered ? health_trigger_min_period_sec_ : publish_min_period_sec_;
      if (dt >= 0.0 && dt < min_period) return;
    }
    last_publish_stamp_sec_ = stamp_sec;

    if (!enable_alignment_) {
      publish_debug(stamp_sec, "alignment_disabled", 0, false);
      return;
    }

    std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> prior;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      prior = latest_pose_;
    }
    if (!prior.has_value()) {
      publish_debug(stamp_sec, "no_initial_pose", 0, false);
      return;
    }

    if (alignment_wall_delay_ms_ > 0) {
      schedule_delayed_alignment(msg, stamp_sec, prior.value());
      return;
    }
    process_alignment(msg, stamp_sec, prior.value());
  }

  void schedule_delayed_alignment(
    const sensor_msgs::msg::PointCloud2::SharedPtr & msg, const double stamp_sec,
    const geometry_msgs::msg::PoseWithCovarianceStamped & prior)
  {
    const auto delay = std::chrono::milliseconds(alignment_wall_delay_ms_);
    auto timer_holder = std::make_shared<rclcpp::TimerBase::SharedPtr>();
    *timer_holder = create_wall_timer(delay, [this, msg, stamp_sec, prior, timer_holder]() {
      if (*timer_holder) {
        (*timer_holder)->cancel();
      }
      process_alignment(msg, stamp_sec, prior);
      std::lock_guard<std::mutex> lock(delayed_timers_mutex_);
      delayed_timers_.erase(
        std::remove(delayed_timers_.begin(), delayed_timers_.end(), *timer_holder),
        delayed_timers_.end());
    });
    std::lock_guard<std::mutex> lock(delayed_timers_mutex_);
    delayed_timers_.push_back(*timer_holder);
  }

  void process_alignment(
    const sensor_msgs::msg::PointCloud2::SharedPtr & msg, const double stamp_sec,
    const geometry_msgs::msg::PoseWithCovarianceStamped & prior)
  {
    pcl::PointCloud<PointT>::Ptr scan(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *scan);
    if (scan_voxel_leaf_size_m_ > 0.0) {
      pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
      pcl::VoxelGrid<PointT> voxel;
      voxel.setInputCloud(scan);
      const auto leaf = static_cast<float>(scan_voxel_leaf_size_m_);
      voxel.setLeafSize(leaf, leaf, leaf);
      voxel.filter(*filtered);
      scan = filtered;
    }
    if (static_cast<int>(scan->size()) < min_points_) {
      publish_debug(stamp_sec, "insufficient_points", scan->size(), false);
      return;
    }

    if (map_source_ == "service") {
      if (!update_map_if_needed(prior.pose.pose.position)) {
        publish_debug(stamp_sec, "map_unavailable:" + last_map_failure_reason_, scan->size(), false);
        return;
      }
    } else if (map_source_ == "pcd_tiles") {
      if (!update_pcd_tile_map_if_needed(prior.pose.pose.position)) {
        publish_debug(stamp_sec, "map_unavailable:" + last_map_failure_reason_, scan->size(), false);
        return;
      }
    } else if (!has_ready_topic_map()) {
      publish_debug(stamp_sec, "map_unavailable:" + last_map_failure_reason_, scan->size(), false);
      return;
    }

    std::vector<Candidate> candidates;
    {
      std::lock_guard<std::mutex> lock(ndt_mutex_);
      ndt_.setInputSource(scan);
      candidates = align_candidates(prior.pose.pose);
    }
    publish_payload(stamp_sec, scan->size(), candidates);
  }

  void on_loaded_map(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<PointT>::Ptr map_cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *map_cloud);
    if (map_cloud->empty()) {
      last_map_failure_reason_ = "empty_loaded_map_topic";
      return;
    }

    {
      std::lock_guard<std::mutex> lock(ndt_mutex_);
      ndt_.removeTarget(topic_map_cell_id_);
      ndt_.addTarget(map_cloud, topic_map_cell_id_);
      ndt_.createVoxelKdtree();
      has_map_ = true;
      last_map_position_.reset();
      last_map_failure_reason_.clear();
    }
    publish_debug(stamp_to_sec(msg->header.stamp), "loaded_map_topic_update", map_cloud->size(), true);
  }

  bool has_ready_topic_map()
  {
    std::lock_guard<std::mutex> lock(ndt_mutex_);
    if (has_map_) return true;
    last_map_failure_reason_ = "waiting_for_loaded_map_topic";
    return false;
  }

  bool load_tile_index_if_needed()
  {
    if (tile_index_loaded_) return !map_tiles_.empty();
    tile_index_loaded_ = true;
    map_tiles_.clear();
    if (map_directory_.empty()) {
      last_map_failure_reason_ = "empty_map_directory";
      return false;
    }
    std::error_code ec;
    if (!std::filesystem::is_directory(map_directory_, ec)) {
      last_map_failure_reason_ = "map_directory_not_found";
      return false;
    }
    for (const auto & entry : std::filesystem::directory_iterator(map_directory_, ec)) {
      if (ec) break;
      if (!entry.is_regular_file()) continue;
      const auto path = entry.path();
      if (path.extension() != ".pcd") continue;
      double x = 0.0;
      double y = 0.0;
      if (!parse_tile_origin_from_name(path.filename().string(), x, y)) continue;
      map_tiles_.push_back(MapTile{path.filename().string(), path.string(), x, y});
    }
    std::sort(map_tiles_.begin(), map_tiles_.end(), [](const MapTile & lhs, const MapTile & rhs) {
      return lhs.id < rhs.id;
    });
    if (map_tiles_.empty()) {
      last_map_failure_reason_ = "no_pcd_tiles_indexed";
      return false;
    }
    last_map_failure_reason_.clear();
    return true;
  }

  bool update_pcd_tile_map_if_needed(const geometry_msgs::msg::Point & position)
  {
    {
      std::lock_guard<std::mutex> lock(ndt_mutex_);
      if (last_map_position_.has_value()) {
        const double dx = position.x - last_map_position_->x;
        const double dy = position.y - last_map_position_->y;
        if (std::hypot(dx, dy) <= map_update_distance_m_ && has_map_) return true;
      }
    }

    if (!load_tile_index_if_needed()) return false;

    std::vector<std::pair<double, const MapTile *>> selected_tiles;
    selected_tiles.reserve(map_tiles_.size());
    const double select_radius =
      map_radius_m_ + std::max(0.0, map_tile_load_radius_margin_m_) +
      0.7072 * std::max(0.0, map_tile_resolution_m_);
    for (const auto & tile : map_tiles_) {
      const double center_x = tile.x + 0.5 * map_tile_resolution_m_;
      const double center_y = tile.y + 0.5 * map_tile_resolution_m_;
      const double distance = std::hypot(center_x - position.x, center_y - position.y);
      if (distance <= select_radius) {
        selected_tiles.emplace_back(distance, &tile);
      }
    }
    if (selected_tiles.empty()) {
      last_map_failure_reason_ = "no_tiles_in_radius";
      return false;
    }
    std::sort(selected_tiles.begin(), selected_tiles.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.first < rhs.first;
    });
    if (max_tiles_per_update_ > 0 && selected_tiles.size() > static_cast<std::size_t>(max_tiles_per_update_)) {
      selected_tiles.resize(static_cast<std::size_t>(max_tiles_per_update_));
    }

    std::unordered_set<std::string> desired_ids;
    desired_ids.reserve(selected_tiles.size());
    for (const auto & item : selected_tiles) {
      desired_ids.insert(item.second->id);
    }

    {
      std::lock_guard<std::mutex> lock(ndt_mutex_);
      for (auto it = loaded_tile_ids_.begin(); it != loaded_tile_ids_.end();) {
        if (desired_ids.count(*it) == 0U) {
          ndt_.removeTarget(*it);
          it = loaded_tile_ids_.erase(it);
        } else {
          ++it;
        }
      }

      for (const auto & item : selected_tiles) {
        const auto & tile = *item.second;
        if (loaded_tile_ids_.count(tile.id) != 0U) continue;
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        if (pcl::io::loadPCDFile<PointT>(tile.path, *cloud) != 0 || cloud->empty()) {
          continue;
        }
        if (map_voxel_leaf_size_m_ > 0.0) {
          pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
          pcl::VoxelGrid<PointT> voxel;
          voxel.setInputCloud(cloud);
          const auto leaf = static_cast<float>(map_voxel_leaf_size_m_);
          voxel.setLeafSize(leaf, leaf, leaf);
          voxel.filter(*filtered);
          cloud = filtered;
        }
        ndt_.addTarget(cloud, tile.id);
        loaded_tile_ids_.insert(tile.id);
      }

      ndt_.createVoxelKdtree();
      has_map_ = !loaded_tile_ids_.empty();
      last_map_position_ = position;
      last_map_failure_reason_ = has_map_ ? "" : "tile_load_failed";
      return has_map_;
    }
  }

  bool update_map_if_needed(const geometry_msgs::msg::Point & position)
  {
    {
      std::lock_guard<std::mutex> lock(ndt_mutex_);
      if (last_map_position_.has_value()) {
        const double dx = position.x - last_map_position_->x;
        const double dy = position.y - last_map_position_->y;
        if (std::hypot(dx, dy) <= map_update_distance_m_ && has_map_) return true;
      }
    }

    if (!map_client_) {
      last_map_failure_reason_ = "service_client_disabled";
      return false;
    }

    if (
      !map_client_->service_is_ready() &&
      !map_client_->wait_for_service(std::chrono::milliseconds(100))) {
      last_map_failure_reason_ = "service_not_ready";
      return false;
    }

    std::vector<std::string> cached_ids;
    {
      std::lock_guard<std::mutex> lock(ndt_mutex_);
      cached_ids = ndt_.getCurrentMapIDs();
    }

    auto request = std::make_shared<GetDifferentialPointCloudMap::Request>();
    request->area.center_x = static_cast<float>(position.x);
    request->area.center_y = static_cast<float>(position.y);
    request->area.radius = static_cast<float>(map_radius_m_);
    request->cached_ids = cached_ids;

    auto future = map_client_->async_send_request(request);
    if (future.wait_for(std::chrono::milliseconds(map_service_timeout_ms_)) != std::future_status::ready) {
      last_map_failure_reason_ = "service_timeout";
      std::lock_guard<std::mutex> lock(ndt_mutex_);
      return has_map_;
    }
    const auto response = future.get();
    if (response->new_pointcloud_with_ids.empty() && response->ids_to_remove.empty()) {
      std::lock_guard<std::mutex> lock(ndt_mutex_);
      last_map_position_ = position;
      last_map_failure_reason_ = has_map_ ? "" : "empty_initial_response";
      return has_map_;
    }

    {
      std::lock_guard<std::mutex> lock(ndt_mutex_);
      for (const auto & map : response->new_pointcloud_with_ids) {
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(map.pointcloud, *cloud);
        ndt_.addTarget(cloud, map.cell_id);
      }
      for (const auto & id : response->ids_to_remove) {
        ndt_.removeTarget(id);
      }
      ndt_.createVoxelKdtree();
      has_map_ = !ndt_.getCurrentMapIDs().empty();
      last_map_position_ = position;
      last_map_failure_reason_.clear();
      return has_map_;
    }
  }

  std::vector<Candidate> align_candidates(const geometry_msgs::msg::Pose & prior_pose)
  {
    std::vector<std::tuple<double, double, double, std::size_t>> offsets;
    offsets.reserve(offset_along_m_.size() * offset_cross_m_.size() * offset_yaw_deg_.size());
    std::size_t flat_index = 0;
    for (const double along : offset_along_m_) {
      for (const double cross : offset_cross_m_) {
        for (const double yaw_deg : offset_yaw_deg_) {
          offsets.emplace_back(along, cross, yaw_deg, flat_index++);
        }
      }
    }

    std::vector<Candidate> candidates;
    if (offsets.empty()) return candidates;
    const std::size_t max_candidates =
      max_candidates_per_scan_ > 0 ? static_cast<std::size_t>(max_candidates_per_scan_) :
                                     offsets.size();
    const std::size_t candidate_count = std::min(offsets.size(), max_candidates);
    const std::size_t start_index =
      candidate_count < offsets.size() ? offset_cycle_index_ % offsets.size() : 0U;
    if (candidate_count < offsets.size()) {
      offset_cycle_index_ = (offset_cycle_index_ + candidate_count) % offsets.size();
    }
    candidates.reserve(candidate_count);

    const double prior_yaw = yaw_from_pose(prior_pose);
    const double forward_x = std::cos(prior_yaw);
    const double forward_y = std::sin(prior_yaw);
    const double lateral_x = -std::sin(prior_yaw);
    const double lateral_y = std::cos(prior_yaw);

    for (std::size_t offset_i = 0; offset_i < candidate_count; ++offset_i) {
          const auto & offset = offsets[(start_index + offset_i) % offsets.size()];
          const double along = std::get<0>(offset);
          const double cross = std::get<1>(offset);
          const double yaw_deg = std::get<2>(offset);
          const std::size_t candidate_index = std::get<3>(offset);
          Candidate c;
          c.index = candidate_index;
          c.offset_along_m = along;
          c.offset_cross_m = cross;
          c.offset_yaw_deg = yaw_deg;
          c.initial_pose = offset_pose(prior_pose, along, cross, yaw_deg);
          pcl::PointCloud<PointT> output_cloud;
          ndt_.align(output_cloud, pose_to_matrix(c.initial_pose));
          const auto result = ndt_.getResult();
          c.result_pose = matrix_to_pose(result.pose);
          c.iteration_count = result.iteration_num;
          c.max_iterations = ndt_.getMaximumIterations();
          c.hit_max_iteration = c.max_iterations > 0 && c.iteration_count >= c.max_iterations;
          c.transform_probability = result.transform_probability;
          c.nvtl = result.nearest_voxel_transformation_likelihood;
          const double dx_initial = c.result_pose.position.x - c.initial_pose.position.x;
          const double dy_initial = c.result_pose.position.y - c.initial_pose.position.y;
          c.initial_to_result_distance_m = std::hypot(dx_initial, dy_initial);
          c.initial_to_result_yaw_deg =
            normalize_angle(yaw_from_pose(c.result_pose) - yaw_from_pose(c.initial_pose)) * 180.0 / M_PI;
          const double dx_prior = c.result_pose.position.x - prior_pose.position.x;
          const double dy_prior = c.result_pose.position.y - prior_pose.position.y;
          c.innovation_along_m = dx_prior * forward_x + dy_prior * forward_y;
          c.innovation_cross_m = dx_prior * lateral_x + dy_prior * lateral_y;
          c.innovation_yaw_deg = normalize_angle(yaw_from_pose(c.result_pose) - prior_yaw) * 180.0 / M_PI;
          c.converged =
            !c.hit_max_iteration && c.nvtl >= min_nvtl_ &&
            c.initial_to_result_distance_m <= max_initial_to_result_m_;
          if (!c.converged) {
            if (c.hit_max_iteration) {
              c.rejection_reason = "hit_max_iteration";
            } else if (c.nvtl < min_nvtl_) {
              c.rejection_reason = "low_nearest_voxel_transformation_likelihood";
            } else {
              c.rejection_reason = "large_initial_to_result";
            }
          }
          candidates.push_back(c);
    }
    return candidates;
  }

  std::pair<double, double> gnss_distance_penalty(
    const geometry_msgs::msg::Pose & pose, const double stamp_sec) const
  {
    if (!latest_gnss_.has_value()) {
      return {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
    }
    const auto & gnss = latest_gnss_.value();
    const double age = std::abs(stamp_sec - stamp_to_sec(gnss.header.stamp));
    if (age > gnss_max_age_sec_) {
      return {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
    }
    const double dx = pose.position.x - gnss.pose.pose.position.x;
    const double dy = pose.position.y - gnss.pose.pose.position.y;
    const double distance = std::hypot(dx, dy);
    const double sigma = std::max(0.1, gnss_sigma_m_);
    const double penalty = std::min(gnss_max_penalty_, distance * distance / (2.0 * sigma * sigma));
    return {distance, penalty};
  }

  bool main_ndt_health_degraded(const double stamp_sec) const
  {
    if (!health_trigger_enable_) return false;
    std::lock_guard<std::mutex> lock(main_health_mutex_);
    if (!latest_main_i2r_.has_value()) return false;
    const double i2r_age = std::abs(stamp_sec - latest_main_i2r_->stamp_sec);
    const bool i2r_fresh = i2r_age <= health_trigger_max_metric_age_sec_;
    if (
      i2r_fresh && latest_main_i2r_->value > health_trigger_i2r_m_) {
      return true;
    }
    if (latest_main_nvtl_.has_value()) {
      const double nvtl_age = std::abs(stamp_sec - latest_main_nvtl_->stamp_sec);
      if (
        i2r_fresh && nvtl_age <= health_trigger_max_metric_age_sec_ &&
        latest_main_nvtl_->value > 1.0e-6 && latest_main_i2r_->value > 0.25 &&
        latest_main_nvtl_->value < health_trigger_min_nvtl_) {
        return true;
      }
    }
    if (latest_main_iteration_.has_value() && health_trigger_max_iteration_count_ > 0) {
      const double iteration_age = std::abs(stamp_sec - latest_main_iteration_->stamp_sec);
      if (
        iteration_age <= health_trigger_max_metric_age_sec_ &&
        latest_main_iteration_->value >= health_trigger_max_iteration_count_ &&
        latest_main_i2r_->value > health_trigger_i2r_m_) {
        return true;
      }
    }
    return false;
  }

  void append_main_ndt_health_json(std::ostringstream & out, const double stamp_sec) const
  {
    out << ",\"main_ndt_health\":{";
    if (!health_trigger_enable_) {
      out << "\"available\":false,\"reason\":\"disabled\"}";
      return;
    }
    std::lock_guard<std::mutex> lock(main_health_mutex_);
    const bool available =
      latest_main_i2r_.has_value() && latest_main_nvtl_.has_value() &&
      latest_main_iteration_.has_value();
    out << "\"available\":" << (available ? "true" : "false");
    if (!available) {
      out << ",\"reason\":\"main_ndt_metric_missing\"}";
      return;
    }
    const double i2r_age = std::abs(stamp_sec - latest_main_i2r_->stamp_sec);
    const double nvtl_age = std::abs(stamp_sec - latest_main_nvtl_->stamp_sec);
    const double iteration_age = std::abs(stamp_sec - latest_main_iteration_->stamp_sec);
    const double metric_age = std::max({i2r_age, nvtl_age, iteration_age});
    const bool i2r_fresh = i2r_age <= health_trigger_max_metric_age_sec_;
    const bool triggered =
      (i2r_fresh && latest_main_i2r_->value > health_trigger_i2r_m_) ||
      (i2r_fresh && nvtl_age <= health_trigger_max_metric_age_sec_ &&
       latest_main_nvtl_->value > 1.0e-6 && latest_main_i2r_->value > 0.25 &&
       latest_main_nvtl_->value < health_trigger_min_nvtl_) ||
      (health_trigger_max_iteration_count_ > 0 &&
       iteration_age <= health_trigger_max_metric_age_sec_ &&
       latest_main_iteration_->value >= health_trigger_max_iteration_count_ &&
       latest_main_i2r_->value > health_trigger_i2r_m_);
    out << ",\"reason\":\"\""
        << ",\"metric_age_sec\":" << metric_age << ",\"pose_age_sec\":";
    if (latest_main_pose_stamp_sec_.has_value()) {
      out << std::abs(stamp_sec - latest_main_pose_stamp_sec_.value());
    } else {
      out << "null";
    }
    out << ",\"nearest_voxel_transformation_likelihood\":" << latest_main_nvtl_->value
        << ",\"initial_to_result_distance_m\":" << latest_main_i2r_->value
        << ",\"iteration_count\":" << latest_main_iteration_->value
        << ",\"max_iterations\":" << health_trigger_max_iteration_count_
        << ",\"health_triggered\":" << (triggered ? "true" : "false") << "}";
  }

  void publish_payload(
    const double stamp_sec, const std::size_t scan_size, const std::vector<Candidate> & candidates)
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(6);
    out << "{\"schema_version\":1,\"stamp_sec\":" << stamp_sec
        << ",\"reason\":\"runtime_candidate_observer\""
        << ",\"source\":\"independent_ndt_candidate_observer\""
        << ",\"controls_output\":false,\"controls_final_localization\":false"
        << ",\"uses_gt\":false,\"uses_future_frames\":false,\"uses_gnss_or_gt\":false"
        << ",\"uses_gnss_weak_prior\":" << (enable_gnss_weak_prior_ ? "true" : "false")
        << ",\"gnss_usage\":\"" << (enable_gnss_weak_prior_ ? "weak_penalty_only" : "not_used")
        << "\",\"candidate_count\":" << candidates.size()
        << ",\"has_selected_candidate\":true,\"selected_candidate_index\":0"
        << ",\"route_progress_m\":null,\"rejection_reason\":\"\""
        << ",\"observer_scan_point_count\":" << scan_size;
    append_main_ndt_health_json(out, stamp_sec);
    out << ",\"candidates\":[";
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      const auto & c = candidates[i];
      if (i > 0) out << ',';
      const auto gnss = gnss_distance_penalty(c.result_pose, stamp_sec);
      out << "{\"index\":" << c.index << ",\"source\":\"independent_ndt_alignment\""
          << ",\"initial_x\":" << c.initial_pose.position.x
          << ",\"initial_y\":" << c.initial_pose.position.y
          << ",\"initial_z\":" << c.initial_pose.position.z
          << ",\"initial_yaw_deg\":" << yaw_from_pose(c.initial_pose) * 180.0 / M_PI
          << ",\"result_x\":" << c.result_pose.position.x
          << ",\"result_y\":" << c.result_pose.position.y
          << ",\"result_z\":" << c.result_pose.position.z
          << ",\"result_yaw_deg\":" << yaw_from_pose(c.result_pose) * 180.0 / M_PI
          << ",\"offset_along_m\":" << c.offset_along_m
          << ",\"offset_cross_m\":" << c.offset_cross_m
          << ",\"offset_yaw_deg\":" << c.offset_yaw_deg
          << ",\"converged\":" << (c.converged ? "true" : "false")
          << ",\"iteration_count\":" << c.iteration_count
          << ",\"iteration_num\":" << c.iteration_count
          << ",\"max_iterations\":" << c.max_iterations
          << ",\"hit_max_iteration\":" << (c.hit_max_iteration ? "true" : "false")
          << ",\"transform_probability\":" << c.transform_probability
          << ",\"nearest_voxel_transformation_likelihood\":" << c.nvtl
          << ",\"score\":" << c.nvtl << ",\"total_score\":" << c.nvtl
          << ",\"initial_to_result_distance_m\":" << c.initial_to_result_distance_m
          << ",\"initial_to_result_yaw_deg\":" << c.initial_to_result_yaw_deg
          << ",\"innovation_along_m\":" << c.innovation_along_m
          << ",\"innovation_cross_m\":" << c.innovation_cross_m
          << ",\"innovation_yaw_deg\":" << c.innovation_yaw_deg
          << ",\"localizability_along_variance_m2\":0.0"
          << ",\"localizability_cross_variance_m2\":0.0"
          << ",\"covariance_condition_number\":1.0";
      if (std::isfinite(gnss.first)) {
        const double age = latest_gnss_.has_value()
                             ? std::abs(stamp_sec - stamp_to_sec(latest_gnss_->header.stamp))
                             : std::numeric_limits<double>::quiet_NaN();
        out << ",\"gnss_weak_prior_distance_m\":" << gnss.first
            << ",\"gnss_weak_prior_penalty\":" << gnss.second
            << ",\"gnss_weak_prior_age_sec\":" << age
            << ",\"gnss_weak_prior_gate_reason\":\"weak_penalty_only\"";
      } else {
        out << ",\"gnss_weak_prior_distance_m\":null"
            << ",\"gnss_weak_prior_penalty\":null"
            << ",\"gnss_weak_prior_age_sec\":null"
            << ",\"gnss_weak_prior_gate_reason\":\"no_gnss_weak_prior\"";
      }
      out << ",\"rejection_reason\":" << json_string(c.rejection_reason)
          << ",\"reject_reason\":" << json_string(c.rejection_reason)
          << ",\"selected_by_observer\":" << (c.index == 0 ? "true" : "false") << "}";
    }
    out << "]}";

    std_msgs::msg::String msg;
    msg.data = out.str();
    output_pub_->publish(msg);
    publish_debug(stamp_sec, "published", scan_size, true, candidates.size());
  }

  void publish_debug(
    const double stamp_sec, const std::string & reason, const std::size_t scan_size,
    const bool map_ready, const std::size_t candidate_count = 0)
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(6);
    std::size_t map_cell_count = 0;
    {
      std::lock_guard<std::mutex> lock(ndt_mutex_);
      map_cell_count = ndt_.getCurrentMapIDs().size();
    }

    out << "{\"stamp_sec\":" << stamp_sec << ",\"reason\":" << json_string(reason)
        << ",\"source\":\"independent_ndt_candidate_observer\""
        << ",\"controls_output\":false,\"controls_final_localization\":false"
        << ",\"scan_point_count\":" << scan_size
        << ",\"candidate_count\":" << candidate_count
        << ",\"map_ready\":" << (map_ready ? "true" : "false")
        << ",\"map_cell_count\":" << map_cell_count
        << ",\"health_triggered\":" << (main_ndt_health_degraded(stamp_sec) ? "true" : "false")
        << ",\"gnss_usage\":\"" << (enable_gnss_weak_prior_ ? "weak_penalty_only" : "not_used")
        << "\"}";
    std_msgs::msg::String msg;
    msg.data = out.str();
    debug_pub_->publish(msg);
  }

  std::string gnss_topic_;
  std::string output_topic_;
  std::string debug_topic_;
  std::string map_service_name_;
  std::string map_source_;
  std::string map_topic_;
  std::string map_directory_;
  const std::string topic_map_cell_id_{"loaded_map_topic"};
  double publish_min_period_sec_{};
  double map_radius_m_{};
  double map_update_distance_m_{};
  double map_tile_resolution_m_{};
  double map_tile_load_radius_margin_m_{};
  double gnss_max_age_sec_{};
  double gnss_sigma_m_{};
  double gnss_max_penalty_{};
  double min_nvtl_{};
  double max_initial_to_result_m_{};
  double map_voxel_leaf_size_m_{};
  double scan_voxel_leaf_size_m_{};
  double health_trigger_min_period_sec_{};
  double health_trigger_max_metric_age_sec_{};
  double health_trigger_i2r_m_{};
  double health_trigger_min_nvtl_{};
  int alignment_wall_delay_ms_{};
  int min_points_{};
  int map_service_timeout_ms_{};
  int max_tiles_per_update_{};
  int health_trigger_max_iteration_count_{};
  int max_candidates_per_scan_{};
  bool enable_alignment_{};
  bool enable_gnss_weak_prior_{};
  bool health_trigger_enable_{};
  bool has_map_{false};
  bool tile_index_loaded_{false};
  std::string last_map_failure_reason_{"not_called"};
  std::optional<double> last_publish_stamp_sec_;
  std::optional<geometry_msgs::msg::Point> last_map_position_;
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> latest_pose_;
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> latest_gnss_;
  std::vector<MapTile> map_tiles_;
  std::unordered_set<std::string> loaded_tile_ids_;
  std::vector<double> offset_along_m_;
  std::vector<double> offset_cross_m_;
  std::vector<double> offset_yaw_deg_;
  std::size_t offset_cycle_index_{0};
  NdtType ndt_;
  std::mutex ndt_mutex_;
  std::mutex pose_mutex_;
  mutable std::mutex main_health_mutex_;
  std::mutex delayed_timers_mutex_;
  std::vector<rclcpp::TimerBase::SharedPtr> delayed_timers_;
  std::optional<TimedValue<double>> latest_main_nvtl_;
  std::optional<TimedValue<double>> latest_main_i2r_;
  std::optional<TimedValue<int>> latest_main_iteration_;
  std::optional<double> latest_main_pose_stamp_sec_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr output_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::Client<GetDifferentialPointCloudMap>::SharedPtr map_client_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr main_pose_sub_;
  rclcpp::Subscription<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr main_nvtl_sub_;
  rclcpp::Subscription<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr main_i2r_sub_;
  rclcpp::Subscription<autoware_internal_debug_msgs::msg::Int32Stamped>::SharedPtr
    main_iteration_sub_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IndependentNdtCandidateObserver>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
