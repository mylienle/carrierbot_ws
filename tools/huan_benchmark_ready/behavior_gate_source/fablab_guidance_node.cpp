#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "obstacle_avoidance.hpp"
#include "avoidance_recovery.hpp"
#include "route_preview.hpp"
#include "behavior_gate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;
// A latched avoidance stop is released once d_obs exceeds d_brake by this much (m)
constexpr double kAvoidReleaseMargin = 0.3;
// ... for this many consecutive cycles (1 s at 10 Hz), so a detection flicker does not release it
constexpr int kAvoidReleaseCycles = 10;
// Largest move of the obstacle centre between cycles still counted as the same estimate (m)
constexpr double kAvoidCenterJump = 0.25;
// planAvoidance adds this buffer to the configured clearance.
constexpr double kAvoidPlanningBuffer = 0.10;

double normalizeAngle(double angle)
{
  angle = std::fmod(angle + kPi, 2.0 * kPi);
  if (angle < 0.0) {
    angle += 2.0 * kPi;
  }
  return angle - kPi;
}

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

std::string guidanceLogPath()
{
  std::ostringstream filename;
  const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm local_time{};
  localtime_r(&now, &local_time);
  filename << "guidance_" << std::put_time(&local_time, "%Y%m%d_%H%M%S") << ".csv";

  const char * prefixes = std::getenv("COLCON_PREFIX_PATH");
  if (prefixes != nullptr) {
    std::string prefix(prefixes);
    prefix = prefix.substr(0, prefix.find(':'));
    const std::string install_suffix = "/install";
    if (prefix.size() > install_suffix.size() &&
      prefix.compare(prefix.size() - install_suffix.size(), install_suffix.size(), install_suffix) == 0)
    {
      return prefix.substr(0, prefix.size() - install_suffix.size()) +
             "/src/carrierbot_datalog/data/" + filename.str();
    }
  }
  return "/tmp/" + filename.str();
}
}  // namespace

// ROS 2 port of robot_fablab_ws/src/guidance_node/src/guidance_node.cpp.
// It deliberately tracks the waypoint list directly instead of asking Nav2 to
// plan and complete one NavigateToPose action per waypoint.
class FablabGuidanceNode : public rclcpp::Node
{
public:
  FablabGuidanceNode()
  : Node("fablab_guidance_node")
  {
    linear_speed_ = declare_parameter<double>("linear_speed", 0.6);
    angular_speed_ = declare_parameter<double>("angular_speed", 0.5);
    goal_radius_ = declare_parameter<double>("goal_radius", 0.5);
    cycle_seconds_ = declare_parameter<double>("cycle", 0.1);
    max_linear_speed_ = declare_parameter<double>("linear_speed_max", 0.6);
    max_angular_speed_ = declare_parameter<double>("angular_speed_max", 0.8);
    kd_ = declare_parameter<double>("KD", 0.7);
    min_linear_speed_ = declare_parameter<double>("min_speed_linear", 0.3);
    direct_ = declare_parameter<double>("direct", 1.0);
    delta_min_ = declare_parameter<double>("delta_min", 0.5);
    delta_max_ = declare_parameter<double>("delta_max", 0.8);
    lookahead_gamma_ = declare_parameter<double>("lookahead_gamma", 4.0);
    corner_turn_threshold_ = declare_parameter<double>("corner_turn_threshold", 0.35);
    corner_advance_ = declare_parameter<double>("corner_advance", 0.0);
    corner_slowdown_distance_ = declare_parameter<double>("corner_slowdown_distance", 0.8);
    corner_max_speed_ = declare_parameter<double>("corner_max_speed", 0.35);
    turn_heading_threshold_ = declare_parameter<double>("turn_heading_threshold", 0.35);
    turn_heading_max_speed_ = declare_parameter<double>("turn_heading_max_speed", 0.18);
    // Final waypoint: stop closer than the intermediate switch radius, approaching slowly
    final_goal_radius_ = declare_parameter<double>("final_goal_radius", 0.15);
    final_slowdown_distance_ = declare_parameter<double>("final_slowdown_distance", 0.6);
    final_max_speed_ = declare_parameter<double>("final_max_speed", 0.2);
    start_alignment_threshold_ = declare_parameter<double>("start_alignment_threshold", 0.35);
    start_alignment_tolerance_ = declare_parameter<double>("start_alignment_tolerance", 0.20);
    start_alignment_angular_speed_ = declare_parameter<double>("start_alignment_angular_speed", 0.25);
    danger_distance_ = declare_parameter<double>("danger_distance", 0.6);
    safety_enabled_ = declare_parameter<bool>("safety_enabled", true);
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/amcl_pose");
    initial_pose_topic_ = declare_parameter<std::string>("initial_pose_topic", "/initialpose");
    waypoint_topic_ = declare_parameter<std::string>("waypoint_topic", "/fablab_waypoints");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    // Reactive avoidance of the paper (Section V); defaults follow Table I
    avoidance_enabled_ = declare_parameter<bool>("avoidance_enabled", true);
    avoid_params_.robot_radius = declare_parameter<double>("robot_radius", 0.52);
    avoid_params_.clearance = declare_parameter<double>("avoid_clearance", 0.1);
    avoid_params_.avoid_distance = declare_parameter<double>("avoid_distance", 2.5);
    avoid_params_.rejoin_distance = declare_parameter<double>("rejoin_distance", 1.5);
    avoid_params_.stop_min_clearance = declare_parameter<double>("stop_min_clearance", 0.3);
    avoid_params_.actuator_delay = declare_parameter<double>("actuator_delay", 0.17);
    avoid_params_.brake_decel = declare_parameter<double>("brake_decel", 0.6);
    avoid_params_.wall_margin = declare_parameter<double>("avoid_wall_margin", 0.08);
    avoid_params_.max_lateral = declare_parameter<double>("avoid_max_lateral", 2.5);
    avoid_params_.switch_radius = goal_radius_;
    sense_distance_ = declare_parameter<double>("sense_distance", 5.0);
    // Raw /scan also hits the robot's own frame; ignore returns closer than this to the robot centre
    avoid_self_radius_ = declare_parameter<double>("avoid_self_radius", 0.6);
    // -1 = pass the obstacle on the right (sigma of Eq. 21), +1 = left
    avoid_side_ = static_cast<int>(declare_parameter<int64_t>("avoid_side", -1));
    avoid_confirm_cycles_ = static_cast<int>(declare_parameter<int64_t>("avoid_confirm_cycles", 3));
    // Scan points this close to an occupied map cell are walls, not new obstacles
    map_ignore_radius_ = declare_parameter<double>("map_ignore_radius", 0.15);
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    guidance_log_path_ = declare_parameter<std::string>("guidance_log_path", guidanceLogPath());
    route_preview_enabled_ = declare_parameter<bool>("route_preview_enabled", false);
    behavior_gate_enabled_ = declare_parameter<bool>("behavior_gate_enabled", false);
    early_rejoin_enabled_ = declare_parameter<bool>("early_rejoin_enabled", false);
    final_approach_ramp_enabled_ = declare_parameter<bool>("final_approach_ramp_enabled", false);
    clear_cruise_enabled_ = declare_parameter<bool>("clear_cruise_enabled", false);
    yield_before_detour_ = declare_parameter<bool>("yield_before_detour", false);
    compact_standing_detour_ = declare_parameter<bool>("compact_standing_detour", false);
    standing_center_gap_ = declare_parameter<double>("standing_center_gap", 0.0);
    standing_post_pass_cruise_ = declare_parameter<bool>("standing_post_pass_cruise", false);
    if (early_rejoin_enabled_ && !behavior_gate_enabled_) {
      throw std::invalid_argument("Early rejoin requires the behavior gate");
    }
    if (final_approach_ramp_enabled_ &&
      (!early_rejoin_enabled_ || !(final_slowdown_distance_ > final_goal_radius_)))
    {
      throw std::invalid_argument("Final approach ramp requires early rejoin and a valid slowdown window");
    }
    if (clear_cruise_enabled_ && !behavior_gate_enabled_) {
      throw std::invalid_argument("Clear cruise requires the behavior gate");
    }
    if (yield_before_detour_ && !behavior_gate_enabled_) {
      throw std::invalid_argument("Yield before detour requires the behavior gate");
    }
    if (compact_standing_detour_ && !yield_before_detour_) {
      throw std::invalid_argument("Compact standing detour requires yield before detour");
    }
    if (standing_center_gap_ != 0.0 &&
      ((standing_center_gap_ != 1.50 && standing_center_gap_ != 1.65) ||
      compact_standing_detour_ || !yield_before_detour_))
    {
      throw std::invalid_argument("The 1.50 and 1.65 m standing centre gaps require yield before detour and their own profile");
    }
    if (standing_post_pass_cruise_ && standing_center_gap_ != 1.65) {
      throw std::invalid_argument("Post-pass cruise requires the 1.65 m standing centre gap");
    }
    if (behavior_gate_enabled_ && !route_preview_enabled_) {
      throw std::runtime_error("Behavior gate requires route telemetry");
    }
    route_preview_topic_ = declare_parameter<std::string>(
      "route_preview_topic", "/avoidance_preview/route_state");

    if (
      cycle_seconds_ <= 0.0 || goal_radius_ <= 0.0 ||
      delta_min_ <= 0.0 || delta_max_ < delta_min_ || lookahead_gamma_ < 0.0 ||
      corner_turn_threshold_ <= 0.0 || corner_advance_ < 0.0 ||
      corner_slowdown_distance_ <= 0.0 || corner_max_speed_ <= 0.0 ||
      turn_heading_threshold_ <= 0.0 || turn_heading_max_speed_ <= 0.0 ||
      final_goal_radius_ <= 0.0 || final_slowdown_distance_ <= 0.0 || final_max_speed_ <= 0.0 ||
      start_alignment_tolerance_ <= 0.0 ||
      start_alignment_threshold_ < start_alignment_tolerance_ ||
      start_alignment_angular_speed_ <= 0.0 ||
      avoid_params_.robot_radius <= 0.0 || avoid_params_.clearance < 0.0 ||
      avoid_params_.avoid_distance <= 0.0 || avoid_params_.rejoin_distance <= 0.0 ||
      avoid_params_.brake_decel <= 0.0 || sense_distance_ < avoid_params_.avoid_distance)
    {
      throw std::runtime_error("Invalid Fablab guidance parameters");
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    cmd_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    arrival_publisher_ = create_publisher<std_msgs::msg::Bool>("/fablab_arrival", 10);
    if (route_preview_enabled_) {
      route_preview_publisher_ = create_publisher<std_msgs::msg::String>(route_preview_topic_, 1);
      RCLCPP_INFO(get_logger(), "Read-only internal route telemetry: %s", route_preview_topic_.c_str());
    }
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic_, 10,
      std::bind(&FablabGuidanceNode::poseCallback, this, std::placeholders::_1));
    initial_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic_, 10,
      std::bind(&FablabGuidanceNode::poseCallback, this, std::placeholders::_1));
    waypoint_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      waypoint_topic_, 100,
      std::bind(&FablabGuidanceNode::waypointCallback, this, std::placeholders::_1));
    if (behavior_gate_enabled_) {
      behavior_subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/dynamic_behavior/directive", 1,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr message) {
          behavior_gate_.receive(message->data, get_clock()->now().seconds(), wallSeconds(), route_revision_);
        });
      RCLCPP_INFO(get_logger(), "Behavior gate enabled; commands held until fresh directives");
    }
    if (avoidance_enabled_) {
      map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, rclcpp::QoS(1).transient_local().reliable(),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr message) {map_ = message;});
    }
    if (safety_enabled_ || avoidance_enabled_) {
      scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, 10,
        std::bind(&FablabGuidanceNode::scanCallback, this, std::placeholders::_1));
    }

    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(cycle_seconds_)),
      std::bind(&FablabGuidanceNode::controlCallback, this));

    guidance_log_.open(guidance_log_path_);
    if (guidance_log_.is_open()) {
      guidance_log_ <<
        "time_s,segment,amcl_x,amcl_y,amcl_yaw,start_x,start_y,goal_x,goal_y,"
        "cross_track,long_track,lookahead,target_heading,heading_error,remaining_distance,"
        "linear_cmd,angular_cmd,safety_stop,finished,"
        "avoid_state,obstacle_x,obstacle_y,obstacle_r,obstacle_distance,recovery_count\n";
      RCLCPP_INFO(get_logger(), "Guidance CSV: %s", guidance_log_path_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Cannot open guidance CSV: %s", guidance_log_path_.c_str());
    }

    RCLCPP_INFO(
      get_logger(),
      "Fablab LOS-PD mode ready: direct waypoint tracking, goal_radius=%.2f m, cycle=%.2f s",
      goal_radius_, cycle_seconds_);
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    x_ = message->pose.pose.position.x;
    y_ = message->pose.pose.position.y;
    const auto & orientation = message->pose.pose.orientation;
    theta_ = std::atan2(
      2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
      1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
    const bool received_first_pose = !has_pose_;
    has_pose_ = true;

    if (received_first_pose && !pending_waypoints_.empty()) {
      startMissionAtCurrentPose();
      for (const auto & waypoint : pending_waypoints_) {
        appendWaypoint(waypoint.first, waypoint.second);
      }
      pending_waypoints_.clear();
    }
  }

  void waypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (arrival_published_ && (!waypoints_.empty() || !pending_waypoints_.empty())) {
      RCLCPP_WARN(get_logger(), "New mission received; resetting completed Fablab route");
      waypoints_.clear();
      pending_waypoints_.clear();
      current_segment_ = 0;
      arrival_published_ = false;
      finished_ = false;
      resetAvoidance();
    }

    const double waypoint_x = message->pose.position.x;
    const double waypoint_y = message->pose.position.y;
    if (!has_pose_) {
      pending_waypoints_.emplace_back(waypoint_x, waypoint_y);
      RCLCPP_WARN(
        get_logger(), "Queueing Fablab waypoint until /amcl_pose or /initialpose is available");
      return;
    }

    if (waypoints_.empty()) {
      startMissionAtCurrentPose();
    }
    appendWaypoint(waypoint_x, waypoint_y);
  }

  void startMissionAtCurrentPose()
  {
    waypoints_.emplace_back(x_, y_);
    current_segment_ = 0;
    finished_ = false;
    start_alignment_pending_ = true;
    start_alignment_active_ = false;
    integral_ = 0.0;
    RCLCPP_INFO(get_logger(), "Added Fablab start waypoint: (%.3f, %.3f)", x_, y_);
  }

  void appendWaypoint(double waypoint_x, double waypoint_y)
  {
    // The GUI currently includes its current pose as the first route element.
    // The original Fablab node adds that start point itself, so ignore only an
    // exact-near duplicate to prevent a zero-length first segment.
    if (waypoints_.size() == 1 &&
      std::hypot(waypoint_x - waypoints_.front().first, waypoint_y - waypoints_.front().second) < 0.05)
    {
      RCLCPP_INFO(get_logger(), "Ignoring duplicate route start waypoint");
      return;
    }

    waypoints_.emplace_back(waypoint_x, waypoint_y);
    RCLCPP_INFO(
      get_logger(), "Received Fablab waypoint %zu: (%.3f, %.3f)",
      waypoints_.size() - 1, waypoint_x, waypoint_y);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr message)
  {
    if (avoidance_enabled_) {
      storeScanPoints(*message);
    }
    if (!safety_enabled_) {
      return;
    }
    int danger_count = 0;
    for (std::size_t i = 0; i < message->ranges.size(); ++i) {
      const float range = message->ranges[i];
      const double angle = message->angle_min + i * message->angle_increment;
      if (std::abs(normalizeAngle(angle - kPi)) > kPi / 2.0) {
        continue;
      }
      if (std::isfinite(range) && range >= 0.5F && range <= danger_distance_) {
        ++danger_count;
        if (danger_count >= 30) {
          safety_stop_ = true;
          return;
        }
      }
    }
    safety_stop_ = false;
  }

  // Scan points in the map frame, in scan order, flagged when they are not on the static map
  void storeScanPoints(const sensor_msgs::msg::LaserScan & message)
  {
    if (!laser_offset_known_) {
      try {
        const auto transform = tf_buffer_->lookupTransform(
          "base_footprint", message.header.frame_id, tf2::TimePointZero);
        laser_x_ = transform.transform.translation.x;
        laser_y_ = transform.transform.translation.y;
        const auto & q = transform.transform.rotation;
        laser_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        laser_offset_known_ = true;
      } catch (const tf2::TransformException &) {
        return;
      }
    }
    scan_points_.clear();
    scan_point_is_new_.clear();
    const double cos_theta = std::cos(theta_);
    const double sin_theta = std::sin(theta_);
    for (std::size_t i = 0; i < message.ranges.size(); ++i) {
      const float range = message.ranges[i];
      if (!std::isfinite(range) || range < message.range_min || range > sense_distance_) {
        continue;
      }
      const double angle = laser_yaw_ + message.angle_min + i * message.angle_increment;
      const double base_x = laser_x_ + range * std::cos(angle);
      const double base_y = laser_y_ + range * std::sin(angle);
      if (std::hypot(base_x, base_y) < avoid_self_radius_) {
        continue;
      }
      const fablab_avoidance::Point point{
        x_ + base_x * cos_theta - base_y * sin_theta,
        y_ + base_x * sin_theta + base_y * cos_theta};
      scan_points_.push_back(point);
      scan_point_is_new_.push_back(!onStaticMap(point));
    }
    recovery_scan_stamp_ = rclcpp::Time(message.header.stamp).seconds();
    recovery_scan_received_ = std::chrono::steady_clock::now();
    recovery_scan_available_ = true;
  }

  bool onStaticMap(const fablab_avoidance::Point & point) const
  {
    if (!map_ || map_->info.resolution <= 0.0) {
      return false;
    }
    const auto & info = map_->info;
    const int cell_x = static_cast<int>(std::floor((point.x - info.origin.position.x) / info.resolution));
    const int cell_y = static_cast<int>(std::floor((point.y - info.origin.position.y) / info.resolution));
    const int reach = static_cast<int>(std::ceil(map_ignore_radius_ / info.resolution));
    for (int dy = -reach; dy <= reach; ++dy) {
      for (int dx = -reach; dx <= reach; ++dx) {
        const int x = cell_x + dx;
        const int y = cell_y + dy;
        if (x < 0 || y < 0 || x >= static_cast<int>(info.width) || y >= static_cast<int>(info.height)) {
          continue;
        }
        if (map_->data[static_cast<std::size_t>(y) * info.width + x] >= 50) {
          return true;
        }
      }
    }
    return false;
  }

  // Occupied static map cells within sense_distance of the robot, appended as wall points. They
  // come after the scan points, so the obstacle member flags (scan indices) never skip them.
  void appendMapWalls(
    const fablab_avoidance::Point & robot, std::vector<fablab_avoidance::Point> & points,
    bool require_known_space = false) const
  {
    if (!map_ || map_->info.resolution <= 0.0) {
      return;
    }
    const auto & info = map_->info;
    const double resolution = info.resolution;
    const int robot_x = static_cast<int>(std::floor((robot.x - info.origin.position.x) / resolution));
    const int robot_y = static_cast<int>(std::floor((robot.y - info.origin.position.y) / resolution));
    const int reach = static_cast<int>(std::ceil(sense_distance_ / resolution));
    for (int dy = -reach; dy <= reach; ++dy) {
      for (int dx = -reach; dx <= reach; ++dx) {
        const int x = robot_x + dx;
        const int y = robot_y + dy;
        if (std::hypot(dx, dy) > reach) {
          continue;
        }
        const bool inside = x >= 0 && y >= 0 && x < static_cast<int>(info.width) &&
          y < static_cast<int>(info.height);
        const int occupancy = inside ? map_->data[static_cast<std::size_t>(y) * info.width + x] : -1;
        if (occupancy < 50 && !(require_known_space && occupancy < 0)) {continue;}
        points.push_back({info.origin.position.x + (x + 0.5) * resolution,
            info.origin.position.y + (y + 0.5) * resolution});
      }
    }
  }

  void resetAvoidance()
  {
    avoid_stop_ = false;
    avoid_end_segment_ = 0;
    stationary_detour_ = false;
    avoid_confirm_count_ = 0;
    avoid_latched_ = false;
    avoid_release_count_ = 0;
    obstacle_ = fablab_avoidance::Obstacle();
    recovery_wait_cycles_ = 0;
    recovery_count_ = 0;
    early_rejoin_used_ = false;
    early_rejoin_window_.reset();
  }

  bool recoverBlockedDetour(bool early = false)
  {
    // A generic rejoin uses the ordinary scan clearance. Keep the checked
    // waypoints of a stationary-person bypass with its larger separation.
    if (stationary_detour_ || !recovery_scan_available_ || !map_ ||
      current_segment_ + 1 >= avoid_end_segment_ || avoid_end_segment_ >= waypoints_.size())
    {
      early_rejoin_window_.reset();
      return false;
    }
    const double now = get_clock()->now().seconds();
    const double scan_age = now - recovery_scan_stamp_;
    const double receipt_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - recovery_scan_received_).count();
    if (scan_age < -0.02 || scan_age > 0.5 || receipt_age > 0.5) {
      early_rejoin_window_.reset();
      return false;
    }
    if (early && recovery_scan_stamp_ == early_rejoin_window_.last_scan_stamp) {
      return false;
    }
    try {
      const auto transform = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
      const double tf_age = now - rclcpp::Time(transform.header.stamp).seconds();
      if (tf_age < -0.02 || tf_age > 0.5) {
        early_rejoin_window_.reset();
        return false;
      }
    } catch (const tf2::TransformException &) {
      early_rejoin_window_.reset();
      return false;
    }
    const fablab_avoidance::Point robot{x_, y_};
    const fablab_avoidance::Point goal{
      waypoints_[avoid_end_segment_].first, waypoints_[avoid_end_segment_].second};
    if (early && (scan_points_.size() < 30 ||
      fablab_avoidance::distance(robot, goal) > sense_distance_))
    {
      early_rejoin_window_.reset();
      return false;
    }
    std::vector<fablab_avoidance::Point> points = scan_points_;
    appendMapWalls(robot, points);
    if (early && std::any_of(points.begin(), points.end(), [&](const auto & point) {
      return fablab_avoidance::distance(point, robot) <
             avoid_params_.robot_radius + avoid_params_.clearance;
    })) {
      early_rejoin_window_.reset();
      return false;
    }
    const auto prefix = fablab_avoidance::planRejoin(
      points, robot, avoid_original_start_, goal, avoid_params_);
    double saved_distance = 0.0;
    if (early) {
      // Confirm the same direct connection over consecutive scans. The
      // projection fallback stays available for blocked-detour recovery.
      if (prefix.size() != 1) {
        early_rejoin_window_.reset();
        return false;
      }
      auto previous = robot;
      for (std::size_t i = current_segment_ + 1; i <= avoid_end_segment_; ++i) {
        const fablab_avoidance::Point next{waypoints_[i].first, waypoints_[i].second};
        saved_distance += fablab_avoidance::distance(previous, next);
        previous = next;
      }
      saved_distance -= fablab_avoidance::distance(robot, goal);
      if (!early_rejoin_window_.observe(recovery_scan_stamp_, saved_distance >= 0.5)) {
        return false;
      }
    }
    if (!fablab_avoidance::replaceDetour(waypoints_, current_segment_, avoid_end_segment_, prefix)) {
      return false;
    }
    avoid_end_segment_ = current_segment_ + prefix.size();
    avoid_latched_ = false;
    avoid_release_count_ = 0;
    avoid_confirm_count_ = 0;
    recovery_wait_cycles_ = 0;
    start_alignment_pending_ = true;
    start_alignment_active_ = false;
    linear_x_ = 0.0;
    angular_z_ = 0.0;
    last_linear_cmd_ = 0.0;
    previous_error_ = 0.0;
    filtered_derivative_ = 0.0;
    integral_ = 0.0;
    last_pid_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    ++recovery_count_;
    early_rejoin_window_.reset();
    if (early) {
      early_rejoin_used_ = true;
      RCLCPP_INFO(get_logger(), "EARLY_REJOIN saved %.2f m; original goal (%.2f, %.2f)",
        saved_distance, goal.x, goal.y);
    } else {
      RCLCPP_WARN(get_logger(),
        "Recovered blocked detour via %zu rejoin points; original goal (%.2f, %.2f) preserved",
        prefix.size() - 1, goal.x, goal.y);
    }
    return true;
  }

  // Detection, decision and stopping monitor of the paper (Eq. 19, 20). On a valid plan the
  // robot pose R, A1, A2 and B are inserted before the segment goal; the original waypoints are
  // followed again once the segment ending at that goal is active.
  void updateAvoidance(const fablab_avoidance::Point * stationary_blocker = nullptr)
  {
    avoid_stop_ = false;
    obstacle_ = fablab_avoidance::Obstacle();
    if (!avoidance_enabled_ || finished_ || start_alignment_pending_ ||
      current_segment_ + 1 >= waypoints_.size())
    {
      avoid_confirm_count_ = 0;
      early_rejoin_window_.reset();
      return;
    }
    const fablab_avoidance::Point robot{x_, y_};
    const fablab_avoidance::Point start{
      waypoints_[current_segment_].first, waypoints_[current_segment_].second};
    const fablab_avoidance::Point goal{
      waypoints_[current_segment_ + 1].first, waypoints_[current_segment_ + 1].second};
    // Only points ahead of the robot heading count: right after a turn onto a diagonal leg the
    // segment band also covers points beside or behind the robot, which then latch the stop forever
    std::vector<bool> is_ahead = scan_point_is_new_;
    const double heading_x = std::cos(theta_);
    const double heading_y = std::sin(theta_);
    for (std::size_t i = 0; i < scan_points_.size(); ++i) {
      if ((scan_points_[i].x - x_) * heading_x + (scan_points_[i].y - y_) * heading_y <= 0.0) {
        is_ahead[i] = false;
      }
    }
    obstacle_ = fablab_avoidance::detectObstacle(
      scan_points_, is_ahead, robot, start, goal, avoid_params_);
    if (stationary_blocker) {
      // A WAIT directive may prepare a new route, but never releases the wheels.
      // Require the current LiDAR obstacle to agree with the matched person.
      const double scan_age = get_clock()->now().seconds() - recovery_scan_stamp_;
      const double receipt_age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - recovery_scan_received_).count();
      if (!safety_stop_ && map_ && map_->info.resolution > 0.0 &&
        recovery_scan_available_ && scan_age >= -0.02 && scan_age <= 0.3 && receipt_age <= 0.6 &&
        scan_points_.size() >= 30 && obstacle_.found &&
        current_segment_ + 1 >= avoid_end_segment_ &&
        obstacle_.surface_distance < avoid_params_.avoid_distance &&
        fablab_avoidance::distance(obstacle_.center, *stationary_blocker) <= 0.35 &&
        fablab_avoidance::distance(robot, goal) <= sense_distance_ &&
        recovery_scan_stamp_ != stationary_plan_scan_stamp_)
      {
        stationary_plan_scan_stamp_ = recovery_scan_stamp_;
        insertDetour(robot, start, goal, true);
      }
      return;
    }
    const double brake_distance = fablab_avoidance::brakeDistance(last_linear_cmd_, avoid_params_);
    if (avoid_latched_) {
      early_rejoin_window_.reset();
      // Re-check the original mission leg while a changed obstacle keeps the
      // temporary detour stopped. Keep waiting unless a checked route exists.
      if (++recovery_wait_cycles_ >= kAvoidReleaseCycles) {
        recovery_wait_cycles_ = 0;
        if (recoverBlockedDetour()) {
          avoid_stop_ = true;
          return;
        }
      }
      // Once stopped, stay stopped: d_obs jitters around d_brake, and every creep forward brings
      // the obstacle closer to avoid_self_radius, where its points are dropped and it vanishes.
      // Release only after the obstacle is gone or clearly beyond d_brake for a while.
      if (obstacle_.found && obstacle_.surface_distance <= brake_distance + kAvoidReleaseMargin) {
        avoid_release_count_ = 0;
      } else {
        ++avoid_release_count_;
      }
      if (avoid_release_count_ < kAvoidReleaseCycles) {
        avoid_stop_ = true;
        return;
      }
      avoid_latched_ = false;
      avoid_release_count_ = 0;
      avoid_confirm_count_ = 0;
      recovery_wait_cycles_ = 0;
    }
    if (!obstacle_.found || obstacle_.surface_distance >= avoid_params_.avoid_distance) {
      avoid_confirm_count_ = 0;
      if (early_rejoin_enabled_ && !early_rejoin_used_ && !safety_stop_ &&
        behavior_result_.action == fablab_behavior::Action::BASELINE)
      {
        if (recoverBlockedDetour(true)) {
          avoid_stop_ = true;
        }
      } else {
        early_rejoin_window_.reset();
      }
      return;
    }
    early_rejoin_window_.reset();
    if (obstacle_.surface_distance <= brake_distance) {
      avoid_stop_ = true;
      avoid_latched_ = true;
      avoid_release_count_ = 0;
      recovery_wait_cycles_ = 0;
      return;
    }
    if (current_segment_ + 1 < avoid_end_segment_) {
      // Already following an avoidance pair: leave it to the stopping monitor
      return;
    }
    if (!fablab_behavior::mayInsertDetour(behavior_result_, yield_before_detour_)) {
      // The current leg may still be followed at the SLOW cap. The stop
      // monitor above remains active, and a new detour must be reconfirmed.
      avoid_confirm_count_ = 0;
      detour_deferred_by_yield_ = true;
      return;
    }
    // Plan only on an estimate that has settled: close to the obstacle the nearest point can
    // switch to another face or a neighbouring object and move the centre by more than 0.5 m
    if (avoid_confirm_count_ > 0 &&
      fablab_avoidance::distance(obstacle_.center, last_obstacle_center_) > kAvoidCenterJump)
    {
      avoid_confirm_count_ = 0;
    }
    last_obstacle_center_ = obstacle_.center;
    if (++avoid_confirm_count_ < avoid_confirm_cycles_) {
      return;
    }
    insertDetour(robot, start, goal, false);
  }

  void insertDetour(const fablab_avoidance::Point & robot, const fablab_avoidance::Point & start,
    const fablab_avoidance::Point & goal, bool stationary)
  {
    // The scan can miss a wall the legs pass (occluded, or a poor return at a distance), so the
    // legs are also checked against the occupied cells of the static map around the robot
    std::vector<fablab_avoidance::Point> points = scan_points_;
    appendMapWalls(robot, points, stationary);
    auto params = avoid_params_;
    auto obstacle = obstacle_;
    const double rejoin_clearance = std::max(params.clearance, compact_standing_detour_ ? 0.90 : 1.03);
    if (stationary) {
      params.wall_margin = std::max(params.wall_margin, 0.32);
      params.rejoin_distance = std::max(params.rejoin_distance, 2.30);
      obstacle.radius = std::max(obstacle.radius, 0.30);
      params.clearance = rejoin_clearance;
      if (standing_center_gap_ > 0.0) {
        params.clearance = std::max(avoid_params_.clearance,
          standing_center_gap_ - params.robot_radius - obstacle.radius - kAvoidPlanningBuffer);
      }
    }
    const auto inserted = fablab_avoidance::planAvoidance(
      obstacle, points, robot, start, goal, avoid_side_, params);
    if (inserted.empty()) {
      // No pair fits from here (e.g. the robot is still inside a narrow passage). The obstacle is
      // beyond d_brake, so keep following the original path and plan again next cycle; the
      // braking check above stops the robot if no pair is found before d_brake.
      return;
    }
    const auto position = waypoints_.begin() + static_cast<std::ptrdiff_t>(current_segment_ + 1);
    std::vector<std::pair<double, double>> pair;
    for (const auto & point : inserted) {
      pair.emplace_back(point.x, point.y);
    }
    waypoints_.insert(position, pair.begin(), pair.end());
    ++current_segment_;  // the active segment is now R -> A
    avoid_end_segment_ = current_segment_ + inserted.size();
    stationary_detour_ = stationary;
    if (stationary) {
      stationary_blocker_center_ = obstacle.center;
      stationary_pass_margin_ = params.robot_radius + obstacle.radius;
      const double cruise_clearance = standing_post_pass_cruise_ ? params.clearance : rejoin_clearance;
      stationary_rejoin_separation_ = stationary_pass_margin_ + cruise_clearance + kAvoidPlanningBuffer;
    }
    avoid_original_start_ = start;
    avoid_confirm_count_ = 0;
    RCLCPP_WARN(
      get_logger(), "Avoiding obstacle at (%.2f, %.2f) r=%.2f m, d=%.2f m via A=(%.2f, %.2f)",
      obstacle_.center.x, obstacle_.center.y, obstacle_.radius, obstacle_.surface_distance,
      inserted[1].x, inserted[1].y);
  }

  double pid(double error)
  {
    const rclcpp::Time now = get_clock()->now();
    if (behavior_reset_pid_) {
      previous_error_ = error;
      filtered_derivative_ = integral_ = 0.0;
      last_pid_time_ = now;
      behavior_reset_pid_ = false;
    }
    double dt = last_pid_time_.nanoseconds() == 0 ? 0.01 : (now - last_pid_time_).seconds();
    dt = std::max(dt, 0.001);
    const double derivative = (error - previous_error_) / dt;
    // The original PID filters its derivative before multiplying by KD.
    // Retaining this is important: without it this port turns substantially
    // harder than the old Robot Fablab controller at a segment transition.
    filtered_derivative_ = 0.2 * derivative + 0.8 * filtered_derivative_;
    // Preserve the Robot Fablab PD call convention: angular_speed is Kp and
    // KD is the derivative coefficient.
    previous_error_ = error;
    last_pid_time_ = now;
    if (std::abs(error) < 0.05) {
      integral_ = clamp(integral_ + 0.1 * cycle_seconds_ * error, -0.1, 0.1);
    }
    return angular_speed_ * error + kd_ * filtered_derivative_ + integral_;
  }

  void controlLos(
    const std::pair<double, double> & goal,
    const std::pair<double, double> & previous)
  {
    const double alpha = std::atan2(goal.second - previous.second, goal.first - previous.first);
    const double segment_length =
      (goal.first - previous.first) * std::cos(alpha) +
      (goal.second - previous.second) * std::sin(alpha);

    if (segment_length <= 1e-6) {
      remaining_distance_ = 0.0;
      linear_x_ = 0.0;
      angular_z_ = 0.0;
      return;
    }

    const double cross_track =
      (-(x_ - previous.first) * std::sin(alpha) +
      (y_ - previous.second) * std::cos(alpha)) * direct_;
    const double long_track =
      (x_ - previous.first) * std::cos(alpha) +
      (y_ - previous.second) * std::sin(alpha);
    const double lookahead =
      (delta_max_ - delta_min_) * std::exp(-lookahead_gamma_ * std::abs(cross_track)) + delta_min_;
    target_heading_ = normalizeAngle(alpha + std::atan(-cross_track / lookahead));
    heading_error_ = normalizeAngle(target_heading_ - theta_);
    start_x_ = previous.first;
    start_y_ = previous.second;
    goal_x_ = goal.first;
    goal_y_ = goal.second;
    cross_track_ = cross_track;
    long_track_ = long_track;
    lookahead_ = lookahead;

    double commanded_angular = clamp(pid(heading_error_), -max_angular_speed_, max_angular_speed_);
    commanded_angular = 0.2 * commanded_angular + 0.8 * angular_z_;
    angular_z_ = commanded_angular;

    remaining_distance_ = std::abs(segment_length - long_track);
    const double remaining_ratio = remaining_distance_ / segment_length;
    if (std::abs(heading_error_) > 0.1) {
      linear_x_ = clamp(
        max_linear_speed_ * std::exp(-3.0 * std::abs(heading_error_)),
        min_linear_speed_, max_linear_speed_);
    } else {
      linear_x_ = clamp(
        linear_speed_ * remaining_ratio, min_linear_speed_, max_linear_speed_);
      const bool standing_rejoin = stationary_detour_ && current_segment_ + 1 == avoid_end_segment_;
      if (final_approach_ramp_enabled_ && (early_rejoin_used_ || standing_rejoin) &&
        current_segment_ + 2 == waypoints_.size())
      {
        const double approach_speed = std::min({linear_speed_, max_linear_speed_, final_max_speed_});
        linear_x_ = fablab_behavior::finalApproachSpeed(behavior_result_, linear_x_,
          remaining_distance_, final_goal_radius_, final_slowdown_distance_,
          std::min(min_linear_speed_, approach_speed), approach_speed);
      }
      if (clear_cruise_enabled_ && !safety_stop_ && !avoid_stop_ && !obstacle_.found &&
        recovery_scan_available_ && !scan_points_.empty() && std::abs(cross_track) <= 0.10 &&
        long_track >= 0.0 && long_track < segment_length)
      {
        const double scan_age = get_clock()->now().seconds() - recovery_scan_stamp_;
        const double receipt_age = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - recovery_scan_received_).count();
        if (scan_age >= -0.02 && scan_age <= 0.3 && receipt_age <= 0.5) {
          const bool final_leg = current_segment_ + 2 == waypoints_.size();
          const double slowdown = final_leg ? final_slowdown_distance_ : corner_slowdown_distance_;
          const double speed = fablab_behavior::clearCruiseSpeed(behavior_result_, linear_x_,
            remaining_distance_, slowdown, std::min(linear_speed_, max_linear_speed_));
          clear_cruise_active_ = speed > linear_x_;
          linear_x_ = speed;
        }
      }
    }
    if (std::abs(heading_error_) >= turn_heading_threshold_) {
      linear_x_ = std::min(linear_x_, turn_heading_max_speed_);
    }
  }

  void updateRoute()
  {
    if (waypoints_.size() < 2) {
      linear_x_ = 0.0;
      angular_z_ = 0.0;
      return;
    }

    if (current_segment_ + 1 >= waypoints_.size()) {
      linear_x_ = 0.0;
      angular_z_ = 0.0;
      finished_ = true;
      return;
    }

    const auto & previous = waypoints_[current_segment_];
    const auto & goal = waypoints_[current_segment_ + 1];
    if (start_alignment_pending_ && std::hypot(goal.first - x_, goal.second - y_) > goal_radius_) {
      target_heading_ = std::atan2(goal.second - y_, goal.first - x_);
      heading_error_ = normalizeAngle(target_heading_ - theta_);
      start_x_ = previous.first;
      start_y_ = previous.second;
      goal_x_ = goal.first;
      goal_y_ = goal.second;
      remaining_distance_ = std::hypot(goal.first - x_, goal.second - y_);
      if (!start_alignment_active_ && std::abs(heading_error_) > start_alignment_threshold_) {
        start_alignment_active_ = true;
      }
      if (start_alignment_active_) {
        if (std::abs(heading_error_) <= start_alignment_tolerance_) {
          start_alignment_active_ = false;
          start_alignment_pending_ = false;
        } else {
          linear_x_ = 0.0;
          angular_z_ = clamp(
            angular_speed_ * heading_error_,
            -start_alignment_angular_speed_, start_alignment_angular_speed_);
          // Keep the D term fresh: a +-pi heading jump at route start must not be
          // replayed by LOS after aligning (it turned the robot the wrong way)
          previous_error_ = heading_error_;
          filtered_derivative_ = 0.0;
          last_pid_time_ = get_clock()->now();
          return;
        }
      } else {
        start_alignment_pending_ = false;
      }
    }

    controlLos(goal, previous);
    double reach_radius = goal_radius_;
    if (current_segment_ + 2 < waypoints_.size()) {
      const auto & current = waypoints_[current_segment_];
      const auto & goal = waypoints_[current_segment_ + 1];
      const auto & next = waypoints_[current_segment_ + 2];
      const double incoming = std::atan2(goal.second - current.second, goal.first - current.first);
      const double outgoing = std::atan2(next.second - goal.second, next.first - goal.first);
      if (std::abs(normalizeAngle(outgoing - incoming)) >= corner_turn_threshold_) {
        reach_radius += corner_advance_;
        if (remaining_distance_ <= corner_slowdown_distance_) {
          linear_x_ = std::min(linear_x_, corner_max_speed_);
        }
      }
    } else {
      reach_radius = final_goal_radius_;
      if (remaining_distance_ <= final_slowdown_distance_) {
        linear_x_ = std::min(linear_x_, final_max_speed_);
      }
    }
    if (remaining_distance_ <= reach_radius)
    {
      ++current_segment_;
      RCLCPP_INFO(get_logger(), "Reached Fablab waypoint %zu", current_segment_);
      if (current_segment_ + 1 >= waypoints_.size()) {
        linear_x_ = 0.0;
        angular_z_ = 0.0;
        finished_ = true;
        if (!arrival_published_) {
          std_msgs::msg::Bool arrival;
          arrival.data = true;
          arrival_publisher_->publish(arrival);
          arrival_published_ = true;
          RCLCPP_INFO(get_logger(), "Reached final Fablab waypoint");
        }
      }
    }
  }

  static double wallSeconds()
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  double standingDetourSpeedCap(double now, bool pose_tf_valid) const
  {
    if (!stationary_detour_ || current_segment_ >= avoid_end_segment_) {return 0.0;}
    if (current_segment_ + 1 != avoid_end_segment_ || avoid_end_segment_ >= waypoints_.size()) {
      return 0.06;
    }
    const fablab_avoidance::Point goal{
      waypoints_[avoid_end_segment_].first, waypoints_[avoid_end_segment_].second};
    if (!fablab_avoidance::standingRejoinHasClearance(
      {x_, y_}, stationary_blocker_center_, avoid_original_start_, goal,
      stationary_pass_margin_, stationary_rejoin_separation_))
    {
      return 0.06;
    }
    if (!standing_post_pass_cruise_) {return 0.0;}
    const double scan_age = now - recovery_scan_stamp_;
    const double receipt_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - recovery_scan_received_).count();
    const bool clear = pose_tf_valid && !safety_stop_ && !avoid_stop_ && !obstacle_.found &&
      recovery_scan_available_ && scan_points_.size() >= 30 &&
      scan_age >= -0.02 && scan_age <= 0.3 && receipt_age <= 0.5;
    return clear ? 0.08 : 0.06;
  }

  void refreshRouteRevision()
  {
    if (waypoints_ != revision_waypoints_ || current_segment_ != revision_segment_) {
      ++route_revision_;
      revision_waypoints_ = waypoints_;
      revision_segment_ = current_segment_;
    }
  }

  void controlCallback()
  {
    clear_cruise_active_ = false;
    detour_deferred_by_yield_ = false;
    if (!has_pose_) {
      if (behavior_gate_enabled_) {
        behavior_gate_.forceWait("no_robot_pose");
        behavior_result_ = behavior_gate_.evaluate(get_clock()->now().seconds(), wallSeconds(), route_revision_);
      }
      cmd_publisher_->publish(geometry_msgs::msg::Twist());
      publishRoutePreview(geometry_msgs::msg::Twist(), 0.0, false);
      return;
    }
    double route_pose_stamp = 0.0;
    bool route_pose_tf_valid = false;
    // /amcl_pose only updates ~2.5 Hz and jumps; use map->base_footprint TF
    // (AMCL + odometry) every cycle for a smooth pose, keep /amcl_pose if TF fails
    try {
      const auto transform = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
      x_ = transform.transform.translation.x;
      y_ = transform.transform.translation.y;
      const auto & q = transform.transform.rotation;
      theta_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      route_pose_stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9;
      const double pose_age = get_clock()->now().seconds() - route_pose_stamp;
      route_pose_tf_valid = pose_age >= -0.02 && pose_age <= 0.3;
    } catch (const tf2::TransformException &) {
    }
    refreshRouteRevision();
    if (behavior_gate_enabled_) {
      if (!route_pose_tf_valid) {behavior_gate_.forceWait("robot_pose_tf_unavailable");}
      behavior_result_ = behavior_gate_.evaluate(get_clock()->now().seconds(), wallSeconds(), route_revision_);
      if (behavior_result_.action == fablab_behavior::Action::WAIT) {
        early_rejoin_window_.reset();
        linear_x_ = angular_z_ = 0.0;
        if (behavior_was_waiting_ && behavior_result_.plan_while_waiting) {
          const fablab_avoidance::Point blocker{behavior_result_.blocker_x, behavior_result_.blocker_y};
          updateAvoidance(&blocker);
          refreshRouteRevision();
          behavior_result_ = behavior_gate_.evaluate(
            get_clock()->now().seconds(), wallSeconds(), route_revision_);
        }
      } else {
        if (behavior_was_waiting_) {
          behavior_reset_pid_ = true;
        }
        updateAvoidance();
        updateRoute();
        refreshRouteRevision();
        behavior_result_ = behavior_gate_.evaluate(get_clock()->now().seconds(), wallSeconds(), route_revision_);
      }
      behavior_was_waiting_ = behavior_result_.action == fablab_behavior::Action::WAIT;
    } else {
      updateAvoidance();
      updateRoute();
      refreshRouteRevision();
    }

    geometry_msgs::msg::Twist command;
    if (!finished_ && !safety_stop_ && !avoid_stop_) {
      // Driver wheel limit is 0.9 m/s: keep v + |w|*L/2 below it so turning is not clipped,
      // and v >= |w|*L/2 so the inner wheel never reverses (jerk at corners)
      const double half_turn = std::abs(angular_z_) * 0.57 / 2.0;
      command.linear.x = linear_x_ > 0.0 ?
        std::max(half_turn, std::min(linear_x_, 0.9 - half_turn)) : linear_x_;
      // Speed up at most 0.03 m/s per cycle (0.3 m/s^2) so steps like 0.3 -> 0.6 do not lurch;
      // slowing down stays immediate
      command.linear.x = std::min(command.linear.x, std::max(half_turn, last_linear_cmd_ + 0.03));
      command.angular.z = angular_z_;
    }
    const double standing_speed_cap = standingDetourSpeedCap(get_clock()->now().seconds(), route_pose_tf_valid);
    if (behavior_gate_enabled_) {
      auto motion_limit = behavior_result_;
      if (standing_speed_cap > 0.0) {
        motion_limit.speed_cap = std::min(motion_limit.speed_cap, standing_speed_cap);
      }
      fablab_behavior::limitCommand(motion_limit, command.linear.x, command.angular.z);
    }
    last_linear_cmd_ = command.linear.x;
    cmd_publisher_->publish(command);
    writeGuidanceLog(command);
    publishRoutePreview(command, route_pose_stamp, route_pose_tf_valid, standing_speed_cap);
  }

  void publishRoutePreview(
    const geometry_msgs::msg::Twist & command, double pose_stamp, bool pose_tf_valid,
    double standing_speed_cap = 0.0)
  {
    if (!route_preview_publisher_) {return;}
    fablab_route_preview::Snapshot snapshot;
    snapshot.stamp = get_clock()->now().seconds();
    snapshot.pose_tf_stamp = pose_stamp;
    snapshot.pose_tf_valid = pose_tf_valid;
    snapshot.sequence = ++route_preview_sequence_;
    snapshot.route_revision = route_revision_;
    snapshot.behavior_gate_enabled = behavior_gate_enabled_;
    snapshot.behavior_action = static_cast<int>(behavior_result_.action);
    snapshot.behavior_cap = behavior_result_.speed_cap;
    snapshot.behavior_reason = behavior_result_.reason;
    snapshot.directive_sequence = behavior_result_.sequence;
    snapshot.behavior_age_sim = behavior_result_.age_sim;
    snapshot.behavior_age_wall = behavior_result_.age_wall;
    snapshot.x = x_; snapshot.y = y_; snapshot.yaw = theta_;
    snapshot.has_pose = has_pose_;
    snapshot.finished = finished_;
    snapshot.segment = current_segment_;
    snapshot.waypoints = waypoints_;
    snapshot.speed = linear_speed_;
    snapshot.max_speed = max_linear_speed_;
    snapshot.linear_command = command.linear.x;
    snapshot.angular_command = command.angular.z;
    snapshot.safety_stop = safety_stop_;
    snapshot.avoidance_stop = avoid_stop_;
    snapshot.aligning = start_alignment_active_;
    snapshot.avoidance_state = avoid_stop_ ? 2 : (current_segment_ + 1 < avoid_end_segment_ ? 1 : 0);
    snapshot.early_rejoin_used = early_rejoin_used_;
    snapshot.clear_cruise_active = clear_cruise_active_ && command.linear.x > 0.0 &&
      behavior_result_.action == fablab_behavior::Action::BASELINE;
    snapshot.detour_deferred_by_yield = detour_deferred_by_yield_;
    snapshot.standing_detour_active = stationary_detour_ && current_segment_ < avoid_end_segment_;
    snapshot.standing_detour_speed_limited = standing_speed_cap == 0.06;
    snapshot.standing_detour_speed_cap = standing_speed_cap;
    const auto payload = fablab_route_preview::serialize(snapshot);
    if (payload.empty()) {return;}
    std_msgs::msg::String message;
    message.data = payload;
    route_preview_publisher_->publish(message);
  }

  void writeGuidanceLog(const geometry_msgs::msg::Twist & command)
  {
    if (!guidance_log_.is_open()) {
      return;
    }
    guidance_log_ << std::fixed << std::setprecision(6)
                  << get_clock()->now().seconds() << ',' << current_segment_ << ','
                  << x_ << ',' << y_ << ',' << theta_ << ','
                  << start_x_ << ',' << start_y_ << ',' << goal_x_ << ',' << goal_y_ << ','
                  << cross_track_ << ',' << long_track_ << ',' << lookahead_ << ','
                  << target_heading_ << ',' << heading_error_ << ',' << remaining_distance_ << ','
                  << command.linear.x << ',' << command.angular.z << ','
                  << safety_stop_ << ',' << finished_ << ','
                  << (avoid_stop_ ? 2 : (current_segment_ + 1 < avoid_end_segment_ ? 1 : 0)) << ','
                  << (obstacle_.found ? obstacle_.center.x : 0.0) << ','
                  << (obstacle_.found ? obstacle_.center.y : 0.0) << ','
                  << obstacle_.radius << ',' << obstacle_.surface_distance << ','
                  << recovery_count_ << '\n';
    guidance_log_.flush();
  }

  double linear_speed_{0.6};
  double angular_speed_{0.5};
  double goal_radius_{0.5};
  double cycle_seconds_{0.1};
  double max_linear_speed_{0.6};
  double max_angular_speed_{0.8};
  double kd_{0.7};
  double min_linear_speed_{0.3};
  double direct_{1.0};
  double delta_min_{0.5};
  double delta_max_{0.8};
  double lookahead_gamma_{0.2};
  double corner_turn_threshold_{0.35};
  double corner_advance_{0.0};
  double corner_slowdown_distance_{0.8};
  double corner_max_speed_{0.35};
  double turn_heading_threshold_{0.35};
  double turn_heading_max_speed_{0.18};
  double final_goal_radius_{0.15};
  double final_slowdown_distance_{0.6};
  double final_max_speed_{0.2};
  double start_alignment_threshold_{0.35};
  double start_alignment_tolerance_{0.20};
  double start_alignment_angular_speed_{0.25};
  double danger_distance_{0.6};
  bool safety_enabled_{true};
  bool safety_stop_{false};
  bool avoidance_enabled_{true};
  bool avoid_stop_{false};
  fablab_avoidance::Params avoid_params_;
  fablab_avoidance::Obstacle obstacle_;
  double sense_distance_{5.0};
  double avoid_self_radius_{0.6};
  int avoid_side_{-1};
  int avoid_confirm_cycles_{3};
  int avoid_confirm_count_{0};
  bool avoid_latched_{false};
  int avoid_release_count_{0};
  fablab_avoidance::Point last_obstacle_center_;
  std::size_t avoid_end_segment_{0};
  fablab_avoidance::Point avoid_original_start_;
  int recovery_wait_cycles_{0};
  int recovery_count_{0};
  bool early_rejoin_enabled_{false}, early_rejoin_used_{false};
  bool final_approach_ramp_enabled_{false};
  bool clear_cruise_enabled_{false}, clear_cruise_active_{false};
  bool yield_before_detour_{false}, detour_deferred_by_yield_{false};
  bool compact_standing_detour_{false};
  bool standing_post_pass_cruise_{false};
  double standing_center_gap_{0.0};
  bool stationary_detour_{false};
  fablab_avoidance::Point stationary_blocker_center_;
  double stationary_pass_margin_{0.0}, stationary_rejoin_separation_{0.0};
  double stationary_plan_scan_stamp_{-1.0};
  fablab_avoidance::EarlyRejoinWindow early_rejoin_window_;
  bool recovery_scan_available_{false};
  double recovery_scan_stamp_{0.0};
  std::chrono::steady_clock::time_point recovery_scan_received_;
  double map_ignore_radius_{0.15};
  bool laser_offset_known_{false};
  double laser_x_{0.0};
  double laser_y_{0.0};
  double laser_yaw_{0.0};
  std::vector<fablab_avoidance::Point> scan_points_;
  std::vector<bool> scan_point_is_new_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  double last_linear_cmd_{0.0};
  bool has_pose_{false};
  bool finished_{false};
  bool start_alignment_pending_{false};
  bool start_alignment_active_{false};
  bool arrival_published_{false};
  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};
  double linear_x_{0.0};
  double angular_z_{0.0};
  double remaining_distance_{0.0};
  double heading_error_{0.0};
  double previous_error_{0.0};
  double filtered_derivative_{0.0};
  double integral_{0.0};
  double start_x_{0.0};
  double start_y_{0.0};
  double goal_x_{0.0};
  double goal_y_{0.0};
  double cross_track_{0.0};
  double long_track_{0.0};
  double lookahead_{0.0};
  double target_heading_{0.0};
  rclcpp::Time last_pid_time_{0, 0, RCL_ROS_TIME};
  std::size_t current_segment_{0};
  std::string pose_topic_;
  std::string initial_pose_topic_;
  std::string waypoint_topic_;
  std::string scan_topic_;
  std::string map_topic_;
  std::string guidance_log_path_;
  std::vector<std::pair<double, double>> waypoints_;
  std::vector<std::pair<double, double>> pending_waypoints_;

  bool behavior_gate_enabled_{false}, behavior_was_waiting_{true}, behavior_reset_pid_{false};
  fablab_behavior::Gate behavior_gate_;
  fablab_behavior::Result behavior_result_;
  std::uint64_t route_revision_{0};
  std::size_t revision_segment_{0};
  std::vector<std::pair<double, double>> revision_waypoints_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr behavior_subscription_;
  bool route_preview_enabled_{false};
  std::string route_preview_topic_;
  std::uint64_t route_preview_sequence_{0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr route_preview_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arrival_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::ofstream guidance_log_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FablabGuidanceNode>());
  rclcpp::shutdown();
  return 0;
}
