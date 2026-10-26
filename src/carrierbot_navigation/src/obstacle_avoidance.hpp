// Reactive LiDAR obstacle avoidance of the paper (Section V, Eq. 16-22), kept free of ROS
// so the geometry can be unit tested on its own.
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
  double clearance{0.25};        // c_clear
  double avoid_distance{2.5};    // d_avoid, Eq. 19
  double rejoin_distance{2.0};   // D_r, Eq. 21
  double stop_min_clearance{0.3};  // s_min, Eq. 20
  double actuator_delay{0.17};   // T_delay, Eq. 20
  double brake_decel{0.6};       // a_brake, Eq. 20
  // Free space kept between robot body and walls/other scan points. 0.2 rejected every pair
  // beside the static object near WI on B1, so the robot braked instead of avoiding.
  double wall_margin{0.08};
  double cluster_gap{0.15};      // neighbouring scan points closer than this belong to one obstacle
  double max_lateral{2.5};       // largest d_perp tried before the plan is rejected
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

// min over lambda in [0,1] of |(1-lambda) q1 + lambda q2 - p|, Eq. 22
inline double segmentDistance(const Point & p, const Point & q1, const Point & q2)
{
  const double dx = q2.x - q1.x;
  const double dy = q2.y - q1.y;
  const double length_sq = dx * dx + dy * dy;
  double lambda = 0.0;
  if (length_sq > 1e-12) {
    lambda = std::max(0.0, std::min(((p.x - q1.x) * dx + (p.y - q1.y) * dy) / length_sq, 1.0));
  }
  return std::hypot(q1.x + lambda * dx - p.x, q1.y + lambda * dy - p.y);
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

// Waypoints R (robot), A and, when it falls before the goal, B of Eq. 21. Every transition
// segment must stay R_safe (Eq. 18) away from the obstacle centre (Eq. 22) and keep the robot
// body clear of all other scan points. The preferred side sigma is tried first, then the other
// one; an empty result means the pair is rejected and the robot has to stop instead.
inline std::vector<Point> planAvoidance(
  const Obstacle & obstacle, const std::vector<Point> & points,
  const Point & robot, const Point & start, const Point & goal, int preferred_side,
  const Params & params)
{
  const double segment_length = distance(start, goal);
  if (!obstacle.found || segment_length < 1e-6) {
    return {};
  }
  const double ux = (goal.x - start.x) / segment_length;
  const double uy = (goal.y - start.y) / segment_length;
  const double r_safe = obstacle.radius + params.robot_radius + params.clearance;
  // Small extra margin so scan noise on the circle edge does not re-trigger detection
  const double required = r_safe + 0.1;

  // B is placed on the original path, D_r past the projection of the obstacle centre
  const double center_s = (obstacle.center.x - start.x) * ux + (obstacle.center.y - start.y) * uy;
  const bool use_b = center_s + params.rejoin_distance < segment_length;
  const Point b{start.x + (center_s + params.rejoin_distance) * ux,
    start.y + (center_s + params.rejoin_distance) * uy};

  auto clear = [&](const Point & q1, const Point & q2) {
      if (segmentDistance(obstacle.center, q1, q2) < required) {
        return false;
      }
      for (std::size_t i = 0; i < points.size(); ++i) {
        if (i < obstacle.member.size() && obstacle.member[i]) {
          continue;
        }
        if (segmentDistance(points[i], q1, q2) < params.robot_radius + params.wall_margin) {
          return false;
        }
      }
      return true;
    };

  // The robot leaves for the next segment switch_radius before a waypoint, so it cuts the
  // corner: the chord between the points switch_radius before and after the corner is checked too.
  auto clear_corner = [&](const Point & p, const Point & corner, const Point & q) {
      const double in = distance(p, corner);
      const double out = distance(corner, q);
      if (in < 1e-6 || out < 1e-6) {
        return true;
      }
      const double r_in = std::min(params.switch_radius, in);
      const double r_out = std::min(params.switch_radius, out);
      const Point leave{corner.x + (p.x - corner.x) * r_in / in, corner.y + (p.y - corner.y) * r_in / in};
      const Point enter{corner.x + (q.x - corner.x) * r_out / out,
        corner.y + (q.y - corner.y) * r_out / out};
      return clear(leave, enter);
    };

  const int sides[2] = {preferred_side >= 0 ? 1 : -1, preferred_side >= 0 ? -1 : 1};
  for (const int sigma : sides) {
    for (double lateral = required; lateral <= params.max_lateral + 1e-9; lateral += 0.05) {
      // A = c_o + sigma d_perp (-sin pi_p, cos pi_p), Eq. 21
      const Point a{obstacle.center.x - sigma * lateral * uy,
        obstacle.center.y + sigma * lateral * ux};
      if (!clear(robot, a)) {
        continue;
      }
      const bool valid = use_b ?
        (clear(a, b) && clear(b, goal) && clear_corner(robot, a, b) && clear_corner(a, b, goal)) :
        (clear(a, goal) && clear_corner(robot, a, goal));
      if (valid) {
        return use_b ? std::vector<Point>{robot, a, b} : std::vector<Point>{robot, a};
      }
    }
  }
  return {};
}

}  // namespace fablab_avoidance
