#pragma once
// Diagnostic serialization only. It cannot alter waypoints or publish commands.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace fablab_route_preview
{
struct Snapshot
{
  double stamp{0.0}, pose_tf_stamp{0.0};
  double x{0.0}, y{0.0}, yaw{0.0};
  double speed{0.0}, max_speed{0.0}, linear_command{0.0}, angular_command{0.0};
  std::uint64_t sequence{0};
  std::size_t segment{0};
  std::uint64_t route_revision{0}, directive_sequence{0};
  bool behavior_gate_enabled{false};
  int behavior_action{0};
  double behavior_cap{0.0}, behavior_age_sim{0.0}, behavior_age_wall{0.0};
  const char * behavior_reason{"disabled"};
  bool has_pose{false}, pose_tf_valid{false}, finished{false};
  bool safety_stop{false}, avoidance_stop{false}, aligning{false};
  bool early_rejoin_used{false};
  bool clear_cruise_active{false};
  bool detour_deferred_by_yield{false};
  bool standing_detour_active{false};
  bool standing_detour_speed_limited{false};
  double standing_detour_speed_cap{0.0};
  int avoidance_state{0};
  std::vector<std::pair<double, double>> waypoints;
};

inline std::string serialize(const Snapshot & value)
{
  for (double number : {value.stamp, value.pose_tf_stamp, value.x, value.y, value.yaw,
      value.speed, value.max_speed, value.linear_command, value.angular_command,
      value.behavior_cap, value.behavior_age_sim, value.behavior_age_wall, value.standing_detour_speed_cap}) {
    if (!std::isfinite(number)) {return {};}
  }
  if (value.waypoints.size() > 2000) {return {};}
  for (const auto & point : value.waypoints) {
    if (!std::isfinite(point.first) || !std::isfinite(point.second)) {return {};}
  }
  const bool active = value.has_pose && value.pose_tf_valid && !value.finished &&
    value.waypoints.size() >= 2 && value.segment < value.waypoints.size() - 1;
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(15) << std::boolalpha
      << "{\"schema_version\":1,\"source\":\"fablab_internal_waypoints_v1\","
      << "\"frame_id\":\"map\",\"stamp\":" << value.stamp
      << ",\"sequence\":" << value.sequence << ",\"current_segment\":" << value.segment
      << ",\"route_revision\":" << value.route_revision
      << ",\"behavior_gate\":{\"enabled\":" << value.behavior_gate_enabled
      << ",\"action\":" << value.behavior_action
      << ",\"speed_cap_mps\":" << value.behavior_cap
      << ",\"reason\":\"" << value.behavior_reason << "\""
      << ",\"directive_sequence\":" << value.directive_sequence
      << ",\"age_sim_s\":" << value.behavior_age_sim
      << ",\"age_wall_s\":" << value.behavior_age_wall << '}' 
      << ",\"active\":" << active << ",\"finished\":" << value.finished
      << ",\"pose_tf_valid\":" << value.pose_tf_valid
      << ",\"pose_tf_stamp\":" << value.pose_tf_stamp
      << ",\"robot_pose_map\":[" << value.x << ',' << value.y << ',' << value.yaw << ']'
      << ",\"linear_speed_mps\":" << value.speed
      << ",\"linear_speed_max_mps\":" << value.max_speed
      << ",\"command_preview\":{\"linear_x\":" << value.linear_command
      << ",\"angular_z\":" << value.angular_command << '}'
      << ",\"safety_stop\":" << value.safety_stop
      << ",\"avoidance_stop\":" << value.avoidance_stop
      << ",\"aligning\":" << value.aligning
      << ",\"avoidance_state\":" << value.avoidance_state
      << ",\"early_rejoin_used\":" << value.early_rejoin_used
      << ",\"clear_cruise_active\":" << value.clear_cruise_active
      << ",\"detour_deferred_by_yield\":" << value.detour_deferred_by_yield
      << ",\"standing_detour_active\":" << value.standing_detour_active
      << ",\"standing_detour_speed_limited\":" << value.standing_detour_speed_limited
      << ",\"standing_detour_speed_cap_mps\":" << value.standing_detour_speed_cap
      << ",\"waypoints_map\":[";
  for (std::size_t i = 0; i < value.waypoints.size(); ++i) {
    if (i > 0) {out << ',';}
    out << '[' << value.waypoints[i].first << ',' << value.waypoints[i].second << ']';
  }
  out << "],\"diagnostic_only\":true}";
  return out.str();
}
}  // namespace fablab_route_preview
