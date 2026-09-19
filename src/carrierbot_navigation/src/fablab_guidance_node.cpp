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
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

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
    corner_turn_threshold_ = declare_parameter<double>("corner_turn_threshold", 0.35);
    corner_advance_ = declare_parameter<double>("corner_advance", 0.0);
    corner_slowdown_distance_ = declare_parameter<double>("corner_slowdown_distance", 0.8);
    corner_max_speed_ = declare_parameter<double>("corner_max_speed", 0.35);
    turn_heading_threshold_ = declare_parameter<double>("turn_heading_threshold", 0.7);
    turn_heading_max_speed_ = declare_parameter<double>("turn_heading_max_speed", 0.18);
    start_alignment_threshold_ = declare_parameter<double>("start_alignment_threshold", 0.35);
    start_alignment_tolerance_ = declare_parameter<double>("start_alignment_tolerance", 0.20);
    start_alignment_angular_speed_ = declare_parameter<double>("start_alignment_angular_speed", 0.25);
    danger_distance_ = declare_parameter<double>("danger_distance", 0.6);
    safety_enabled_ = declare_parameter<bool>("safety_enabled", true);
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/amcl_pose");
    initial_pose_topic_ = declare_parameter<std::string>("initial_pose_topic", "/initialpose");
    waypoint_topic_ = declare_parameter<std::string>("waypoint_topic", "/fablab_waypoints");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    guidance_log_path_ = declare_parameter<std::string>("guidance_log_path", guidanceLogPath());

    if (
      cycle_seconds_ <= 0.0 || goal_radius_ <= 0.0 ||
      delta_min_ <= 0.0 || delta_max_ < delta_min_ ||
      corner_turn_threshold_ <= 0.0 || corner_advance_ < 0.0 ||
      corner_slowdown_distance_ <= 0.0 || corner_max_speed_ <= 0.0 ||
      turn_heading_threshold_ <= 0.0 || turn_heading_max_speed_ <= 0.0 ||
      start_alignment_tolerance_ <= 0.0 ||
      start_alignment_threshold_ < start_alignment_tolerance_ ||
      start_alignment_angular_speed_ <= 0.0)
    {
      throw std::runtime_error("Invalid Fablab guidance parameters");
    }

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
    if (safety_enabled_) {
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
        "linear_cmd,angular_cmd,safety_stop,finished\n";
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
    int danger_count = 0;
    for (const float range : message->ranges) {
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
    return angular_speed_ * error + kd_ * filtered_derivative_;
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
      (delta_max_ - delta_min_) * std::exp(-0.7 * cross_track * cross_track) + delta_min_;
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
    updateRoute();

    geometry_msgs::msg::Twist command;
    if (!finished_ && !safety_stop_) {
      command.linear.x = linear_x_;
      command.angular.z = angular_z_;
    }
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
                  << safety_stop_ << ',' << finished_ << '\n';
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
  double corner_turn_threshold_{0.35};
  double corner_advance_{0.0};
  double corner_slowdown_distance_{0.8};
  double corner_max_speed_{0.35};
  double turn_heading_threshold_{0.7};
  double turn_heading_max_speed_{0.18};
  double start_alignment_threshold_{0.35};
  double start_alignment_tolerance_{0.20};
  double start_alignment_angular_speed_{0.25};
  double danger_distance_{0.6};
  bool safety_enabled_{true};
  bool safety_stop_{false};
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
  std::string guidance_log_path_;
  std::vector<std::pair<double, double>> waypoints_;
  std::vector<std::pair<double, double>> pending_waypoints_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arrival_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::ofstream guidance_log_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FablabGuidanceNode>());
  rclcpp::shutdown();
  return 0;
}
