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
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;

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
    avoid_params_.clearance = declare_parameter<double>("avoid_clearance", 0.25);
    avoid_params_.avoid_distance = declare_parameter<double>("avoid_distance", 2.5);
    avoid_params_.rejoin_distance = declare_parameter<double>("rejoin_distance", 2.0);
    avoid_params_.stop_min_clearance = declare_parameter<double>("stop_min_clearance", 0.3);
    avoid_params_.actuator_delay = declare_parameter<double>("actuator_delay", 0.17);
    avoid_params_.brake_decel = declare_parameter<double>("brake_decel", 0.6);
    avoid_params_.wall_margin = declare_parameter<double>("avoid_wall_margin", 0.05);
    avoid_params_.max_lateral = declare_parameter<double>("avoid_max_lateral", 2.5);
    avoid_params_.switch_radius = goal_radius_;
    sense_distance_ = declare_parameter<double>("sense_distance", 5.0);
    // Raw /scan also hits the robot's own frame; ignore returns closer than this, like laser_filter
    avoid_min_range_ = declare_parameter<double>("avoid_min_range", 0.5);
    // -1 = pass the obstacle on the right (sigma of Eq. 21), +1 = left
    avoid_side_ = static_cast<int>(declare_parameter<int64_t>("avoid_side", -1));
    avoid_confirm_cycles_ = static_cast<int>(declare_parameter<int64_t>("avoid_confirm_cycles", 3));
    // Scan points this close to an occupied map cell are walls, not new obstacles
    map_ignore_radius_ = declare_parameter<double>("map_ignore_radius", 0.15);
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    guidance_log_path_ = declare_parameter<std::string>("guidance_log_path", guidanceLogPath());

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
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic_, 10,
      std::bind(&FablabGuidanceNode::poseCallback, this, std::placeholders::_1));
    initial_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic_, 10,
      std::bind(&FablabGuidanceNode::poseCallback, this, std::placeholders::_1));
    waypoint_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      waypoint_topic_, 100,
      std::bind(&FablabGuidanceNode::waypointCallback, this, std::placeholders::_1));
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
        "avoid_state,obstacle_x,obstacle_y,obstacle_r,obstacle_distance\n";
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
      if (!std::isfinite(range) || range < std::max<double>(message.range_min, avoid_min_range_) ||
        range > sense_distance_)
      {
        continue;
      }
      const double angle = laser_yaw_ + message.angle_min + i * message.angle_increment;
      const double base_x = laser_x_ + range * std::cos(angle);
      const double base_y = laser_y_ + range * std::sin(angle);
      const fablab_avoidance::Point point{
        x_ + base_x * cos_theta - base_y * sin_theta,
        y_ + base_x * sin_theta + base_y * cos_theta};
      scan_points_.push_back(point);
      scan_point_is_new_.push_back(!onStaticMap(point));
    }
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

  void resetAvoidance()
  {
    avoid_stop_ = false;
    avoid_end_segment_ = 0;
    avoid_confirm_count_ = 0;
    obstacle_ = fablab_avoidance::Obstacle();
  }

  // Detection, decision and stopping monitor of the paper (Eq. 19, 20). On a valid plan the
  // robot pose R, A and B are inserted before the segment goal; the original waypoints are
  // followed again once the segment ending at that goal is active.
  void updateAvoidance()
  {
    avoid_stop_ = false;
    obstacle_ = fablab_avoidance::Obstacle();
    if (!avoidance_enabled_ || finished_ || start_alignment_pending_ ||
      current_segment_ + 1 >= waypoints_.size())
    {
      avoid_confirm_count_ = 0;
      return;
    }
    const fablab_avoidance::Point robot{x_, y_};
    const fablab_avoidance::Point start{
      waypoints_[current_segment_].first, waypoints_[current_segment_].second};
    const fablab_avoidance::Point goal{
      waypoints_[current_segment_ + 1].first, waypoints_[current_segment_ + 1].second};
    obstacle_ = fablab_avoidance::detectObstacle(
      scan_points_, scan_point_is_new_, robot, start, goal, avoid_params_);
    if (!obstacle_.found || obstacle_.surface_distance >= avoid_params_.avoid_distance) {
      avoid_confirm_count_ = 0;
      return;
    }
    if (obstacle_.surface_distance <=
      fablab_avoidance::brakeDistance(last_linear_cmd_, avoid_params_))
    {
      avoid_stop_ = true;
      return;
    }
    if (current_segment_ + 1 < avoid_end_segment_) {
      // Already following an avoidance pair: leave it to the stopping monitor
      return;
    }
    if (++avoid_confirm_count_ < avoid_confirm_cycles_) {
      return;
    }
    const auto inserted = fablab_avoidance::planAvoidance(
      obstacle_, scan_points_, robot, start, goal, avoid_side_, avoid_params_);
    if (inserted.empty()) {
      avoid_stop_ = true;
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
    avoid_confirm_count_ = 0;
    RCLCPP_WARN(
      get_logger(), "Avoiding obstacle at (%.2f, %.2f) r=%.2f m, d=%.2f m via A=(%.2f, %.2f)",
      obstacle_.center.x, obstacle_.center.y, obstacle_.radius, obstacle_.surface_distance,
      inserted[1].x, inserted[1].y);
  }

  double pid(double error)
  {
    const rclcpp::Time now = get_clock()->now();
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

  void controlCallback()
  {
    if (!has_pose_) {
      cmd_publisher_->publish(geometry_msgs::msg::Twist());
      return;
    }
    // /amcl_pose only updates ~2.5 Hz and jumps; use map->base_footprint TF
    // (AMCL + odometry) every cycle for a smooth pose, keep /amcl_pose if TF fails
    try {
      const auto transform = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
      x_ = transform.transform.translation.x;
      y_ = transform.transform.translation.y;
      const auto & q = transform.transform.rotation;
      theta_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    } catch (const tf2::TransformException &) {
    }
    updateAvoidance();
    updateRoute();

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
    last_linear_cmd_ = command.linear.x;
    cmd_publisher_->publish(command);
    writeGuidanceLog(command);
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
                  << obstacle_.radius << ',' << obstacle_.surface_distance << '\n';
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
  double avoid_min_range_{0.5};
  int avoid_side_{-1};
  int avoid_confirm_cycles_{3};
  int avoid_confirm_count_{0};
  std::size_t avoid_end_segment_{0};
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
