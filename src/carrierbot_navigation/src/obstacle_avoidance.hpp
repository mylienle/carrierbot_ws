// Reactive LiDAR obstacle detection and braking of the paper (Section V, Eq. 16-20), kept free
// of ROS so it can be unit tested on its own. The detour itself is planned in smooth_avoidance.hpp.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace fablab_avoidance
{

struct Point
{
  double x{0.0};
  double y{0.0};
};

struct Params
{
  double robot_radius{0.52};     // r_robot
  double clearance{0.1};         // c_clear
  double avoid_distance{2.5};    // d_avoid, Eq. 19
  double stop_min_clearance{0.3};  // s_min, Eq. 20
  double actuator_delay{0.17};   // T_delay, Eq. 20
  double brake_decel{0.6};       // a_brake, Eq. 20
  // Free space kept between robot body and walls/other scan points. 0.2 rejected every pair
  // beside the static object near WI on B1, so the robot braked instead of avoiding.
  double wall_margin{0.08};
  double cluster_gap{0.15};      // neighbouring scan points closer than this belong to one obstacle
  double max_lateral{2.5};       // largest lateral offset of a detour from the obstacle centre
  double switch_radius{0.5};     // the guidance switches segment this far before a waypoint
};

struct Obstacle
{
  bool found{false};
  Point center;               // c_o
  double radius{0.0};         // R_o
  double surface_distance{0.0};  // d_obs, Eq. 16
  std::vector<bool> member;   // scan points that belong to this obstacle
};

inline double distance(const Point & a, const Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

// d_brake = r_robot + s_min + v T_delay + v^2 / (2 a_brake), Eq. 20
inline double brakeDistance(double speed, const Params & params)
{
  const double v = std::abs(speed);
  return params.robot_radius + params.stop_min_clearance + v * params.actuator_delay +
         v * v / (2.0 * params.brake_decel);
}

// Nearest new obstacle that blocks the active segment ahead of the robot. A scan point blocks
// the segment when its lateral offset is smaller than r_robot + c_clear and it lies between
// the robot and the segment goal (walls behind a corner waypoint are not obstacles).
// points are in the map frame and in scan order; is_new is false for points on the static map.
// The LiDAR only sees the near face, so the obstacle is approximated by the circle centred
// on the midpoint of the cluster ends that encloses every cluster point.
inline Obstacle detectObstacle(
  const std::vector<Point> & points, const std::vector<bool> & is_new,
  const Point & robot, const Point & start, const Point & goal, const Params & params)
{
  Obstacle obstacle;
  const double segment_length = distance(start, goal);
  if (segment_length < 1e-6 || points.empty()) {
    return obstacle;
  }
  const double ux = (goal.x - start.x) / segment_length;
  const double uy = (goal.y - start.y) / segment_length;
  const double robot_s = (robot.x - start.x) * ux + (robot.y - start.y) * uy;
  const double band = params.robot_radius + params.clearance;

  std::size_t nearest = points.size();
  double nearest_distance = 0.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (!is_new[i]) {
      continue;
    }
    const double s = (points[i].x - start.x) * ux + (points[i].y - start.y) * uy;
    const double lateral = -(points[i].x - start.x) * uy + (points[i].y - start.y) * ux;
    if (s <= robot_s || s >= segment_length || std::abs(lateral) >= band) {
      continue;
    }
    const double d = distance(points[i], robot);
    if (nearest == points.size() || d < nearest_distance) {
      nearest = i;
      nearest_distance = d;
    }
  }
  if (nearest == points.size()) {
    return obstacle;
  }

  // Grow the cluster both ways in scan order. The scan is a full turn and the LiDAR is mounted
  // backwards, so straight ahead is where the scan wraps: neighbours are taken modulo n.
  const std::size_t n = points.size();
  obstacle.member.assign(n, false);
  obstacle.member[nearest] = true;
  std::size_t first = nearest;
  std::size_t last = nearest;
  for (std::size_t count = 1; count < n; ++count) {
    const std::size_t previous = (first + n - 1) % n;
    if (obstacle.member[previous] || !is_new[previous] ||
      distance(points[previous], points[first]) >= params.cluster_gap)
    {
      break;
    }
    obstacle.member[previous] = true;
    first = previous;
  }
  for (std::size_t count = 1; count < n; ++count) {
    const std::size_t next = (last + 1) % n;
    if (obstacle.member[next] || !is_new[next] ||
      distance(points[next], points[last]) >= params.cluster_gap)
    {
      break;
    }
    obstacle.member[next] = true;
    last = next;
  }

  // The LiDAR only sees the near face. Like the circle of the paper, assume the obstacle is as
  // deep as it is wide: the centre lies half the visible width behind the nearest point.
  const double half_width = 0.5 * distance(points[first], points[last]);
  obstacle.found = true;
  obstacle.center = {
    points[nearest].x + half_width * (points[nearest].x - robot.x) / nearest_distance,
    points[nearest].y + half_width * (points[nearest].y - robot.y) / nearest_distance};
  obstacle.radius = half_width;
  obstacle.surface_distance = nearest_distance;
  for (std::size_t i = 0; i < n; ++i) {
    if (obstacle.member[i]) {
      obstacle.radius = std::max(obstacle.radius, distance(points[i], obstacle.center));
      obstacle.surface_distance = std::min(obstacle.surface_distance, distance(points[i], robot));
    }
  }
  return obstacle;
}

}  // namespace fablab_avoidance
