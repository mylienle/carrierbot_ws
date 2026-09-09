#include "carrierbot_navigation/los_pd_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace carrierbot_navigation
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kMinDt = 0.001;
constexpr double kMinSegmentLength = 1e-4;
}

double LosPdController::clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

double LosPdController::normalizeAngle(double angle)
{
  angle = std::fmod(angle + kPi, 2.0 * kPi);
  if (angle < 0.0) {
    angle += 2.0 * kPi;
  }
  return angle - kPi;
}

void LosPdController::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  std::string name,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  plugin_name_ = std::move(name);
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  node->declare_parameter(plugin_name_ + ".linear_speed", linear_speed_);
  node->declare_parameter(plugin_name_ + ".min_linear_speed", min_linear_speed_);
  node->declare_parameter(plugin_name_ + ".max_angular_speed", max_angular_speed_);
  node->declare_parameter(plugin_name_ + ".kp", kp_);
  node->declare_parameter(plugin_name_ + ".ki", ki_);
  node->declare_parameter(plugin_name_ + ".kd", kd_);
  node->declare_parameter(plugin_name_ + ".integral_limit", integral_limit_);
  node->declare_parameter(plugin_name_ + ".wheel_separation", wheel_separation_);
  node->declare_parameter(plugin_name_ + ".lookahead_min", lookahead_min_);
  node->declare_parameter(plugin_name_ + ".lookahead_max", lookahead_max_);
  node->declare_parameter(plugin_name_ + ".lookahead_decay", lookahead_decay_);
  node->declare_parameter(
    plugin_name_ + ".target_heading_filter_alpha", target_heading_filter_alpha_);
  node->declare_parameter(
    plugin_name_ + ".derivative_filter_alpha", derivative_filter_alpha_);
  node->declare_parameter(plugin_name_ + ".heading_slowdown_angle", heading_slowdown_angle_);
  node->declare_parameter(plugin_name_ + ".initial_rotate_angle", initial_rotate_angle_);
  node->declare_parameter(plugin_name_ + ".approach_distance", approach_distance_);
  node->declare_parameter(plugin_name_ + ".path_direction", path_direction_);
  node->declare_parameter(
    plugin_name_ + ".use_final_goal_orientation", use_final_goal_orientation_);
  node->declare_parameter(plugin_name_ + ".transform_tolerance", 0.1);

  node->get_parameter(plugin_name_ + ".linear_speed", linear_speed_);
  node->get_parameter(plugin_name_ + ".min_linear_speed", min_linear_speed_);
  node->get_parameter(plugin_name_ + ".max_angular_speed", max_angular_speed_);
  node->get_parameter(plugin_name_ + ".kp", kp_);
  node->get_parameter(plugin_name_ + ".ki", ki_);
  node->get_parameter(plugin_name_ + ".kd", kd_);
  node->get_parameter(plugin_name_ + ".integral_limit", integral_limit_);
  node->get_parameter(plugin_name_ + ".wheel_separation", wheel_separation_);
  node->get_parameter(plugin_name_ + ".lookahead_min", lookahead_min_);
  node->get_parameter(plugin_name_ + ".lookahead_max", lookahead_max_);
  node->get_parameter(plugin_name_ + ".lookahead_decay", lookahead_decay_);
  node->get_parameter(
    plugin_name_ + ".target_heading_filter_alpha", target_heading_filter_alpha_);
  node->get_parameter(
    plugin_name_ + ".derivative_filter_alpha", derivative_filter_alpha_);
  node->get_parameter(plugin_name_ + ".heading_slowdown_angle", heading_slowdown_angle_);
  node->get_parameter(plugin_name_ + ".initial_rotate_angle", initial_rotate_angle_);
  node->get_parameter(plugin_name_ + ".approach_distance", approach_distance_);
  node->get_parameter(plugin_name_ + ".path_direction", path_direction_);
  node->get_parameter(
    plugin_name_ + ".use_final_goal_orientation", use_final_goal_orientation_);

  double transform_tolerance = 0.1;
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);

  if (lookahead_min_ <= 0.0 || lookahead_max_ < lookahead_min_) {
    throw nav2_core::PlannerException("LOS lookahead limits are invalid");
  }
  if (linear_speed_ <= 0.0 || max_angular_speed_ <= 0.0) {
    throw nav2_core::PlannerException("LOS velocity limits must be positive");
  }

  if (integral_limit_ < 0.0) {
    throw nav2_core::PlannerException("LOS integral_limit must be non-negative");
  }

  resetControllerState();
  RCLCPP_INFO(
    logger_,
    "Configured custom LOS+PID controller: Kp=%.3f Ki=%.3f Kd=%.3f v=%.3f m/s omega_max=%.3f rad/s, "
    "lookahead=[%.2f, %.2f] m",
    kp_, ki_, kd_, linear_speed_, max_angular_speed_, lookahead_min_, lookahead_max_);
}

void LosPdController::cleanup()
{
  global_plan_.poses.clear();
  has_last_goal_ = false;
  resetControllerState();
}

void LosPdController::activate()
{
  RCLCPP_INFO(logger_, "Activated custom LOS+PID controller");
}

void LosPdController::deactivate()
{
  has_last_goal_ = false;
  resetControllerState();
  RCLCPP_INFO(logger_, "Deactivated custom LOS+PID controller");
}

void LosPdController::setPlan(const nav_msgs::msg::Path & path)
{
  const bool same_goal = isSameGoalAsCurrentPlan(path);
  global_plan_ = path;
  if (!path.poses.empty()) {
    last_goal_pose_ = path.poses.back().pose;
    has_last_goal_ = true;
  }
  if (!same_goal) {
    resetControllerState();
  }
}

void LosPdController::resetControllerState()
{
  has_controller_state_ = false;
  previous_error_ = 0.0;
  integral_error_ = 0.0;
  filtered_derivative_ = 0.0;
  previous_target_heading_ = 0.0;
  previous_control_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

bool LosPdController::isSameGoalAsCurrentPlan(const nav_msgs::msg::Path & new_path) const
{
  constexpr double kSameGoalPositionToleranceM = 0.05;
  constexpr double kSameGoalYawToleranceRad = 0.05;

  if (!has_last_goal_ || new_path.poses.empty()) {
    return false;
  }

  const auto & new_goal = new_path.poses.back().pose;
  const double dx = new_goal.position.x - last_goal_pose_.position.x;
  const double dy = new_goal.position.y - last_goal_pose_.position.y;
  if (std::hypot(dx, dy) > kSameGoalPositionToleranceM) {
    return false;
  }

  const double yaw_diff = normalizeAngle(
    tf2::getYaw(new_goal.orientation) - tf2::getYaw(last_goal_pose_.orientation));
  return std::fabs(yaw_diff) <= kSameGoalYawToleranceRad;
}

bool LosPdController::transformPose(
  const std::string & target_frame,
  const geometry_msgs::msg::PoseStamped & input,
  geometry_msgs::msg::PoseStamped & output) const
{
  if (input.header.frame_id == target_frame) {
    output = input;
    return true;
  }

  try {
    tf_->transform(input, output, target_frame, transform_tolerance_);
    output.header.frame_id = target_frame;
    return true;
  } catch (const tf2::TransformException & exception) {
    RCLCPP_ERROR(logger_, "LOS+PD TF transform failed: %s", exception.what());
    return false;
  }
}

geometry_msgs::msg::PoseStamped LosPdController::planPoseInBaseFrame(
  std::size_t index,
  const rclcpp::Time & stamp) const
{
  geometry_msgs::msg::PoseStamped input;
  input.header = global_plan_.header;
  input.header.stamp = stamp;
  input.pose = global_plan_.poses.at(index).pose;

  geometry_msgs::msg::PoseStamped output;
  if (!transformPose(costmap_ros_->getBaseFrameID(), input, output)) {
    throw nav2_core::PlannerException("Unable to transform LOS path point into base frame");
  }
  return output;
}

geometry_msgs::msg::TwistStamped LosPdController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /* velocity */)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("LOS+PD received an empty path");
  }

  geometry_msgs::msg::PoseStamped robot_in_plan;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_in_plan)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into LOS path frame");
  }

  std::size_t closest_index = 0;
  double closest_distance_squared = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < global_plan_.poses.size(); ++index) {
    const auto & point = global_plan_.poses[index].pose.position;
    const double dx = point.x - robot_in_plan.pose.position.x;
    const double dy = point.y - robot_in_plan.pose.position.y;
    const double distance_squared = dx * dx + dy * dy;
    if (distance_squared < closest_distance_squared) {
      closest_distance_squared = distance_squared;
      closest_index = index;
    }
  }

  const rclcpp::Time stamp(pose.header.stamp);
  const auto goal_in_base = planPoseInBaseFrame(global_plan_.poses.size() - 1, stamp);
  const double remaining_distance = std::hypot(
    goal_in_base.pose.position.x, goal_in_base.pose.position.y);

  double heading_error = 0.0;
  bool final_orientation_mode = global_plan_.poses.size() == 1 ||
    (use_final_goal_orientation_ && closest_index == global_plan_.poses.size() - 1);

  if (final_orientation_mode) {
    heading_error = normalizeAngle(tf2::getYaw(goal_in_base.pose.orientation));
  } else {
    std::size_t segment_start = closest_index;
    if (segment_start + 1 >= global_plan_.poses.size()) {
      segment_start = global_plan_.poses.size() - 2;
    }

    std::size_t segment_end = segment_start + 1;
    while (segment_end < global_plan_.poses.size()) {
      const auto & start = global_plan_.poses[segment_start].pose.position;
      const auto & end = global_plan_.poses[segment_end].pose.position;
      if (std::hypot(end.x - start.x, end.y - start.y) >= kMinSegmentLength) {
        break;
      }
      ++segment_end;
    }
    if (segment_end >= global_plan_.poses.size()) {
      // A degenerate path has no valid LOS segment. Stop instead of driving on stale data.
      final_orientation_mode = true;
      heading_error = use_final_goal_orientation_ ?
        normalizeAngle(tf2::getYaw(goal_in_base.pose.orientation)) : 0.0;
    } else {
      const auto segment_start_in_base = planPoseInBaseFrame(segment_start, stamp);
      const auto segment_end_in_base = planPoseInBaseFrame(segment_end, stamp);
      const double alpha = std::atan2(
        segment_end_in_base.pose.position.y - segment_start_in_base.pose.position.y,
        segment_end_in_base.pose.position.x - segment_start_in_base.pose.position.x);

      // Robot is at (0, 0) in base_link. This is the Robot 1 LOS cross-track equation.
      const double cross_track = (
        segment_start_in_base.pose.position.x * std::sin(alpha) -
        segment_start_in_base.pose.position.y * std::cos(alpha)) * path_direction_;
      const double lookahead = (lookahead_max_ - lookahead_min_) *
        std::exp(-lookahead_decay_ * std::fabs(cross_track)) + lookahead_min_;
      const double raw_target_heading = normalizeAngle(
        alpha + std::atan(-cross_track / lookahead));

  if (!has_controller_state_) {
        previous_target_heading_ = raw_target_heading;
      }
      const double target_heading = normalizeAngle(
        previous_target_heading_ + target_heading_filter_alpha_ *
        normalizeAngle(raw_target_heading - previous_target_heading_));
      previous_target_heading_ = target_heading;
      heading_error = target_heading;
    }
  }

  const rclcpp::Time now = clock_->now();
  double dt = kMinDt;
  if (has_controller_state_) {
    dt = std::max(kMinDt, (now - previous_control_time_).seconds());
  }
  const double derivative = has_controller_state_ ? (heading_error - previous_error_) / dt : 0.0;
  filtered_derivative_ = derivative_filter_alpha_ * derivative +
    (1.0 - derivative_filter_alpha_) * filtered_derivative_;
  previous_error_ = heading_error;
  previous_control_time_ = now;
  has_controller_state_ = true;

  // Clamping anti-windup, equivalent to the Simulink PID block's clamping mode.
  // Do not integrate while the saturated command would be driven farther into a limit.
  const double omega_before_integration =
    kp_ * heading_error + ki_ * integral_error_ + kd_ * filtered_derivative_;
  const bool drives_positive_saturation =
    omega_before_integration >= max_angular_speed_ && heading_error > 0.0;
  const bool drives_negative_saturation =
    omega_before_integration <= -max_angular_speed_ && heading_error < 0.0;
  if (!drives_positive_saturation && !drives_negative_saturation && ki_ != 0.0) {
    integral_error_ = clamp(
      integral_error_ + heading_error * dt, -integral_limit_, integral_limit_);
  }

  const double omega_raw =
    kp_ * heading_error + ki_ * integral_error_ + kd_ * filtered_derivative_;
  const double angular_z = clamp(omega_raw, -max_angular_speed_, max_angular_speed_);

  double linear_x = 0.0;
  if (!final_orientation_mode) {
    const double approach_ratio = clamp(remaining_distance / approach_distance_, 0.0, 1.0);
    linear_x = std::max(min_linear_speed_, linear_speed_ * approach_ratio);

    if (std::fabs(heading_error) > heading_slowdown_angle_) {
      linear_x = std::max(
        min_linear_speed_, linear_speed_ * std::exp(-3.0 * std::fabs(heading_error)));
    }
    if (closest_index == 0 && std::fabs(heading_error) > initial_rotate_angle_) {
      linear_x = 0.0;
    }

    // The same differential-drive curvature constraint used by Robot 1 guidance.
    if (std::fabs(linear_x) > 1e-6) {
      const double curvature = angular_z / std::fabs(linear_x);
      const double curvature_limited_speed = linear_speed_ /
        (1.0 + (wheel_separation_ / 2.0) * std::fabs(curvature));
      linear_x = std::min(linear_x, curvature_limited_speed);
    }
  }

  geometry_msgs::msg::TwistStamped command;
  command.header = pose.header;
  command.twist.linear.x = linear_x;
  command.twist.angular.z = angular_z;

  RCLCPP_DEBUG(
    logger_,
    "LOS+PID: segment=%zu goal_distance=%.3f heading_error=%.3f integral=%.3f v=%.3f omega=%.3f",
    closest_index, remaining_distance, heading_error, integral_error_, linear_x, angular_z);
  return command;
}

}  // namespace carrierbot_navigation

PLUGINLIB_EXPORT_CLASS(carrierbot_navigation::LosPdController, nav2_core::Controller)
