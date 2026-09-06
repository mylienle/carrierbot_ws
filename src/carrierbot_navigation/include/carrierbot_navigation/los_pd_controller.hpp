#ifndef CARRIERBOT_NAVIGATION__LOS_PD_CONTROLLER_HPP_
#define CARRIERBOT_NAVIGATION__LOS_PD_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/time.h"

namespace carrierbot_navigation
{

/// Custom adaptive line-of-sight guidance with PID heading control.
class LosPdController : public nav2_core::Controller
{
public:
  LosPdController() = default;
  ~LosPdController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;

private:
  static double clamp(double value, double min_value, double max_value);
  static double normalizeAngle(double angle);
  void resetControllerState();
  bool transformPose(
    const std::string & target_frame,
    const geometry_msgs::msg::PoseStamped & input,
    geometry_msgs::msg::PoseStamped & output) const;
  geometry_msgs::msg::PoseStamped planPoseInBaseFrame(
    std::size_t index,
    const rclcpp::Time & stamp) const;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("carrierbot_los_pd_controller")};
  std::string plugin_name_;
  nav_msgs::msg::Path global_plan_;
  tf2::Duration transform_tolerance_{tf2::durationFromSec(0.1)};

  // Robot 1 LOS + PID defaults. They remain ROS parameters for paper experiments.
  double linear_speed_{0.8};
  double min_linear_speed_{0.3};
  double max_angular_speed_{0.5};
  double kp_{1.0};
  double ki_{0.05};
  double kd_{0.7};
  // Clamp the accumulated heading error to prevent integral windup.
  double integral_limit_{1.0};
  double wheel_separation_{0.57};
  double lookahead_min_{0.5};
  double lookahead_max_{0.8};
  double lookahead_decay_{0.7};
  double target_heading_filter_alpha_{0.8};
  double derivative_filter_alpha_{0.2};
  double heading_slowdown_angle_{0.5};
  double initial_rotate_angle_{0.5};
  double approach_distance_{1.0};
  double path_direction_{1.0};
  bool use_final_goal_orientation_{true};

  bool has_controller_state_{false};
  double previous_error_{0.0};
  double integral_error_{0.0};
  double filtered_derivative_{0.0};
  double previous_target_heading_{0.0};
  rclcpp::Time previous_control_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace carrierbot_navigation

#endif  // CARRIERBOT_NAVIGATION__LOS_PD_CONTROLLER_HPP_
