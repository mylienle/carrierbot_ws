#pragma once

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "obstacle_avoidance.hpp"

namespace fablab_avoidance
{

inline bool standingRejoinHasClearance(
  const Point & robot, const Point & blocker, const Point & original_start,
  const Point & original_goal, double pass_margin, double required_separation)
{
  const double dx = original_goal.x - original_start.x;
  const double dy = original_goal.y - original_start.y;
  const double length = std::hypot(dx, dy);
  if (!(length > 1e-6 && pass_margin > 0.0 && required_separation > 0.0)) {
    return false;
  }
  const double passed = ((robot.x - blocker.x) * dx + (robot.y - blocker.y) * dy) / length;
  return passed >= pass_margin &&
         segmentDistance(blocker, robot, original_goal) >= required_separation;
}

struct EarlyRejoinWindow
{
  double first_clear_stamp{-1.0};
  double last_scan_stamp{-1.0};

  void reset()
  {
    first_clear_stamp = last_scan_stamp = -1.0;
  }

  bool observe(double stamp, bool clear)
  {
    if (!clear || !std::isfinite(stamp) || stamp < 0.0) {
      reset();
      return false;
    }
    if (stamp == last_scan_stamp) {
      return false;
    }
    if (last_scan_stamp < 0.0 || stamp < last_scan_stamp || stamp - last_scan_stamp > 0.25) {
      first_clear_stamp = stamp;
    }
    last_scan_stamp = stamp;
    return stamp - first_clear_stamp >= 0.5;
  }
};

// A close initial point can be left behind only if the new segment never gets
// closer to it. Subsequent legs must satisfy the full configured clearance.
// The final safety_guard independently checks every resulting velocity.
inline bool rejoinSegmentClear(
  const std::vector<Point> & points, const Point & start, const Point & goal,
  const Params & params, bool allow_escape)
{
  const double required = params.robot_radius + params.clearance;
  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      return false;
    }
    const double initial = distance(point, start);
    const double nearest = segmentDistance(point, start, goal);
    if (allow_escape && initial < required) {
      if (initial <= params.robot_radius + 0.02 || nearest < initial - 1e-6) {
        return false;
      }
    } else if (nearest < required) {
      return false;
    }
  }
  return true;
}

inline bool rejoinRouteClear(
  const std::vector<Point> & points, const std::vector<Point> & route,
  const Params & params)
{
  if (points.empty() || route.size() < 2) {
    return false;
  }
  for (std::size_t i = 0; i + 1 < route.size(); ++i) {
    if (!rejoinSegmentClear(points, route[i], route[i + 1], params, i == 0)) {
      return false;
    }
    if (i == 0) {
      continue;
    }
    const auto & previous = route[i - 1];
    const auto & corner = route[i];
    const auto & next = route[i + 1];
    const double incoming = distance(previous, corner);
    const double outgoing = distance(corner, next);
    if (incoming < 1e-6 || outgoing < 1e-6) {
      return false;
    }
    const double before = std::min(params.switch_radius, incoming);
    const double after = std::min(params.switch_radius, outgoing);
    const Point leave{
      corner.x + (previous.x - corner.x) * before / incoming,
      corner.y + (previous.y - corner.y) * before / incoming};
    const Point enter{
      corner.x + (next.x - corner.x) * after / outgoing,
      corner.y + (next.y - corner.y) * after / outgoing};
    if (!rejoinSegmentClear(points, leave, enter, params, false)) {
      return false;
    }
  }
  return true;
}

// Return a prefix excluding the original goal, matching planAvoidance's format.
// Prefer direct rejoining; otherwise test the perpendicular projection onto the
// original mission leg, including the corner shortcut made by the controller.
inline std::vector<Point> planRejoin(
  const std::vector<Point> & points, const Point & robot,
  const Point & original_start, const Point & original_goal, const Params & params)
{
  if (distance(original_start, original_goal) < 1e-6 ||
    distance(robot, original_goal) < 1e-6)
  {
    return {};
  }
  if (rejoinRouteClear(points, {robot, original_goal}, params)) {
    return {robot};
  }
  const double dx = original_goal.x - original_start.x;
  const double dy = original_goal.y - original_start.y;
  const double along = std::max(0.0, std::min(1.0,
    ((robot.x - original_start.x) * dx + (robot.y - original_start.y) * dy) /
    (dx * dx + dy * dy)));
  const Point projection{original_start.x + along * dx, original_start.y + along * dy};
  if (distance(robot, projection) <= params.switch_radius ||
    distance(projection, original_goal) <= params.switch_radius)
  {
    return {};
  }
  if (rejoinRouteClear(points, {robot, projection, original_goal}, params)) {
    return {robot, projection};
  }
  return {};
}

// Remove only the remaining temporary detour. Keep its original goal and all
// later mission waypoints in their existing order.
inline bool replaceDetour(
  std::vector<std::pair<double, double>> & waypoints, std::size_t current,
  std::size_t original_goal, const std::vector<Point> & prefix)
{
  if (prefix.empty() || current >= waypoints.size() ||
    original_goal <= current || original_goal >= waypoints.size())
  {
    return false;
  }
  std::vector<std::pair<double, double>> updated;
  for (std::size_t i = 1; i < prefix.size(); ++i) {
    updated.emplace_back(prefix[i].x, prefix[i].y);
  }
  waypoints.erase(waypoints.begin() + static_cast<std::ptrdiff_t>(current + 1),
    waypoints.begin() + static_cast<std::ptrdiff_t>(original_goal));
  waypoints[current] = {prefix.front().x, prefix.front().y};
  waypoints.insert(waypoints.begin() + static_cast<std::ptrdiff_t>(current + 1),
    updated.begin(), updated.end());
  return true;
}

}  // namespace fablab_avoidance
