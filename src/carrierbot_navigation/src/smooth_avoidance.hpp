// Smooth sampling-based detour planner of the fablab guidance (obstacle_avoidance.hpp detects the
// obstacle). Candidates are smooth lateral-offset curves around the obstacle, scored by a cost that
// weighs curvature first, then clearance and extra length; each side is refined with the
// MPPI-style exponentially weighted average of its candidates and the cheaper refined side wins.
// The curve is handed to the
// unchanged LOS + PD tracker as a few evenly spaced waypoints. Kept free of ROS so it can be
// unit tested on its own.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <random>
#include <vector>

#include "obstacle_avoidance.hpp"

namespace fablab_avoidance
{

struct SmoothParams
{
  int samples{48};              // random candidates per side in the first round
  int refine_samples{24};       // candidates drawn around the weighted mean of the winning side
  double temperature{1.0};      // lambda of the MPPI weights exp(-(J - J_min) / lambda)
  double waypoint_spacing{1.0};  // arc length between inserted waypoints (m), > switch_radius
  double max_turn{0.35};        // heading change at a waypoint that is free of turn cost (rad)
  double hard_turn{0.7};        // larger heading changes at a waypoint reject the candidate (rad)
  double ramp_min{1.5};         // shortest ramp from the path to the full offset and back (m)
  double ramp_max{4.0};         // longest ramp (m)
  double hold_extra{1.5};       // the full-offset leg spans at most r_safe + this past the centre (m)
  double clearance_decay{0.3};  // c0 of the clearance cost exp(-c / c0) (m)
  double w_curvature{4.0};      // integral of curvature^2 along the curve
  double w_peak_curvature{2.0};  // peak curvature^2
  double w_clearance{2.0};      // integral of exp(-c / c0) along the tracked legs
  double w_length{1.0};         // extra length over the straight path (m)
  double w_turn{20.0};          // sum of (|turn| - max_turn)^2 over waypoints above max_turn
  double w_side_switch{1.0};    // passing on the other side than the previous detour
  double grid_resolution{0.05};  // local distance grid cell (m)
  double distance_cap{1.5};     // distances beyond this are treated as free space (m)
  unsigned seed{1};             // fixed seed: the same scan always gives the same plan
};

struct SmoothPlan
{
  std::vector<Point> route;     // robot pose, then the waypoints to insert; empty when rejected
  int side{0};                  // +1 passes the obstacle on the left, -1 on the right
  double cost{0.0};
  double max_curvature{0.0};    // 1/m, of the planned curve
  int valid_candidates{0};
};

namespace smooth_detail
{

// Quintic smoothstep: zero slope and zero curvature at both ends, so the offset curve has
// continuous curvature where it leaves and rejoins the path
inline double step(double t)
{
  t = std::max(0.0, std::min(t, 1.0));
  return t * t * t * (10.0 + t * (-15.0 + 6.0 * t));
}
inline double stepSlope(double t)
{
  if (t <= 0.0 || t >= 1.0) {
    return 0.0;
  }
  return 30.0 * t * t * (1.0 - t) * (1.0 - t);
}
inline double stepCurve(double t)
{
  if (t <= 0.0 || t >= 1.0) {
    return 0.0;
  }
  return 60.0 * t * (1.0 - t) * (1.0 - 2.0 * t);
}

// Path frame of the active segment: s along start -> goal, d to the left of it
struct Frame
{
  Point start;
  double ux{1.0};
  double uy{0.0};
  double length{0.0};

  double along(const Point & p) const {return (p.x - start.x) * ux + (p.y - start.y) * uy;}
  double lateral(const Point & p) const {return -(p.x - start.x) * uy + (p.y - start.y) * ux;}
  Point at(double s, double d) const {return {start.x + s * ux - d * uy, start.y + s * uy + d * ux};}
};

// Distance to the nearest scan or wall point on a grid aligned with the segment, filled by a
// brushfire that propagates the nearest source point. Outside the grid, or beyond the cap, the
// space counts as free.
class DistanceGrid
{
public:
  DistanceGrid(
    const Frame & frame, double s_min, double s_max, double d_min, double d_max,
    double resolution, double cap, const std::vector<Point> & points)
  : frame_(frame), s0_(s_min), d0_(d_min), resolution_(resolution), cap_(cap), points_(points)
  {
    ns_ = std::max(1, static_cast<int>(std::ceil((s_max - s_min) / resolution)));
    nd_ = std::max(1, static_cast<int>(std::ceil((d_max - d_min) / resolution)));
    source_.assign(static_cast<std::size_t>(ns_) * nd_, -1);
    distance_.assign(source_.size(), std::numeric_limits<double>::infinity());
    std::queue<int> open;
    for (std::size_t i = 0; i < points_.size(); ++i) {
      const int cell = cellOf(points_[i]);
      if (cell < 0) {
        continue;
      }
      const double d = distance(centre(cell), points_[i]);
      if (d < distance_[cell]) {
        if (source_[cell] < 0) {
          open.push(cell);
        }
        source_[cell] = static_cast<int>(i);
        distance_[cell] = d;
      }
    }
    while (!open.empty()) {
      const int cell = open.front();
      open.pop();
      const int cs = cell / nd_;
      const int cd = cell % nd_;
      const Point & source = points_[static_cast<std::size_t>(source_[cell])];
      for (int ds = -1; ds <= 1; ++ds) {
        for (int dd = -1; dd <= 1; ++dd) {
          const int s = cs + ds;
          const int d = cd + dd;
          if ((ds == 0 && dd == 0) || s < 0 || d < 0 || s >= ns_ || d >= nd_) {
            continue;
          }
          const int next = s * nd_ + d;
          const double candidate = fablab_avoidance::distance(centre(next), source);
          if (candidate < distance_[next] && candidate <= cap_ + resolution_) {
            distance_[next] = candidate;
            source_[next] = source_[cell];
            open.push(next);
          }
        }
      }
    }
  }

  // Distance from p to the nearest point, capped
  double clearance(const Point & p) const
  {
    const int cell = cellOf(p);
    if (cell < 0 || source_[cell] < 0) {
      return cap_;
    }
    return std::min(cap_, fablab_avoidance::distance(p, points_[static_cast<std::size_t>(source_[cell])]));
  }

private:
  int cellOf(const Point & p) const
  {
    const int s = static_cast<int>(std::floor((frame_.along(p) - s0_) / resolution_));
    const int d = static_cast<int>(std::floor((frame_.lateral(p) - d0_) / resolution_));
    if (s < 0 || d < 0 || s >= ns_ || d >= nd_) {
      return -1;
    }
    return s * nd_ + d;
  }
  Point centre(int cell) const
  {
    return frame_.at(s0_ + (cell / nd_ + 0.5) * resolution_, d0_ + (cell % nd_ + 0.5) * resolution_);
  }

  Frame frame_;
  double s0_;
  double d0_;
  double resolution_;
  double cap_;
  const std::vector<Point> & points_;
  int ns_{1};
  int nd_{1};
  std::vector<int> source_;
  std::vector<double> distance_;
};

// One candidate: pass on side, at lateral offset (from the obstacle centre), keeping the full
// offset for hold either side of the centre, with ramps ramp_in / ramp_out
struct Candidate
{
  int side{1};
  double offset{0.0};
  double hold{0.0};
  double ramp_in{2.0};
  double ramp_out{2.0};
};

struct Evaluation
{
  bool valid{false};
  double cost{0.0};
  double max_curvature{0.0};
  std::vector<Point> route;
};

}  // namespace smooth_detail

// Plans the detour for the obstacle that blocks start -> goal. points are all scan points and
// static map walls near the robot (map frame). preferred_side is the side of the previous detour
// (or the configured default); passing on the other side costs w_side_switch.
inline SmoothPlan planSmoothAvoidance(
  const Obstacle & obstacle, const std::vector<Point> & points,
  const Point & robot, const Point & start, const Point & goal, int preferred_side,
  const Params & params, const SmoothParams & smooth)
{
  using smooth_detail::Candidate;
  using smooth_detail::Evaluation;
  SmoothPlan plan;
  const double segment_length = distance(start, goal);
  if (!obstacle.found || segment_length < 1e-6) {
    return plan;
  }
  smooth_detail::Frame frame;
  frame.start = start;
  frame.ux = (goal.x - start.x) / segment_length;
  frame.uy = (goal.y - start.y) / segment_length;
  frame.length = segment_length;

  const double r_safe = obstacle.radius + params.robot_radius + params.clearance;
  // Small extra margin so scan noise on the circle edge does not re-trigger detection
  const double required = r_safe + 0.1;
  const double robot_s = frame.along(robot);
  const double robot_d = frame.lateral(robot);
  const double center_s = frame.along(obstacle.center);
  const double center_d = frame.lateral(obstacle.center);
  const double hold_max = r_safe + smooth.hold_extra;

  const double cap = smooth.distance_cap;
  const double s_lo = robot_s - cap - 0.5;
  const double s_hi = std::min(segment_length, center_s + hold_max + smooth.ramp_max) + 2.0 + cap;
  const double d_extent =
    std::max(std::abs(robot_d), std::abs(center_d) + params.max_lateral) + cap + 0.5;
  const smooth_detail::DistanceGrid grid(
    frame, s_lo, s_hi, -d_extent, d_extent, smooth.grid_resolution, cap, points);

  const double sample_step = smooth.grid_resolution;
  // Clearance cost and hard checks along one straight leg the LOS tracker follows
  auto check_leg = [&](const Point & a, const Point & b, double & clearance_cost) {
      const double length = distance(a, b);
      const int n = std::max(1, static_cast<int>(std::ceil(length / sample_step)));
      for (int i = 0; i <= n; ++i) {
        const double t = static_cast<double>(i) / n;
        const Point p{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
        // Outside the grid the rest of the rejoin leg is the original path, left to detection
        if (frame.along(p) > s_hi - cap) {
          break;
        }
        const double circle = distance(p, obstacle.center) - required;
        const double body = grid.clearance(p) - params.robot_radius - params.wall_margin;
        if (circle < 0.0 || body < 0.0) {
          return false;
        }
        const double c = std::min(distance(p, obstacle.center) - r_safe, grid.clearance(p) - params.robot_radius);
        clearance_cost += std::exp(-std::max(c, 0.0) / smooth.clearance_decay) * (length / n);
      }
      return true;
    };

  auto evaluate = [&](const Candidate & c) {
      Evaluation e;
      const double target_d = center_d + c.side * c.offset;
      if (std::abs(target_d) > std::abs(center_d) + params.max_lateral + 1e-9) {
        return e;
      }
      const double hold_start = center_s - c.hold;
      const double hold_end = center_s + c.hold;
      // The ramp in starts at the robot at the latest; too little room makes the turn too sharp
      const double in_start = std::max(robot_s, hold_start - c.ramp_in);
      const double ramp_in = hold_start - in_start;
      if (ramp_in < smooth.ramp_min * 0.5) {
        return e;
      }
      // Rejoin before the segment goal; when it does not fit the curve ends at the goal itself
      double out_end = hold_end + c.ramp_out;
      const bool ends_at_goal = out_end >= segment_length - 1e-6;
      if (ends_at_goal) {
        out_end = segment_length;
      }
      const double ramp_out = out_end - hold_end;
      if (ramp_out < smooth.ramp_min * 0.5) {
        return e;
      }
      if (ends_at_goal && std::abs(target_d) > 1e-6 && ramp_out < smooth.ramp_min) {
        return e;
      }

      // Offset profile d(s) with its first and second derivatives
      auto profile = [&](double s, double & d1, double & d2) {
          if (s < in_start) {
            d1 = 0.0;
            d2 = 0.0;
            return robot_d;
          }
          if (s < hold_start) {
            const double t = (s - in_start) / ramp_in;
            const double delta = target_d - robot_d;
            d1 = delta * smooth_detail::stepSlope(t) / ramp_in;
            d2 = delta * smooth_detail::stepCurve(t) / (ramp_in * ramp_in);
            return robot_d + delta * smooth_detail::step(t);
          }
          if (s < hold_end) {
            d1 = 0.0;
            d2 = 0.0;
            return target_d;
          }
          const double t = (s - hold_end) / ramp_out;
          d1 = -target_d * smooth_detail::stepSlope(t) / ramp_out;
          d2 = -target_d * smooth_detail::stepCurve(t) / (ramp_out * ramp_out);
          return target_d * (1.0 - smooth_detail::step(t));
        };

      // Dense curve: curvature cost, and waypoints every waypoint_spacing of arc length
      double curvature_integral = 0.0;
      double max_curvature = 0.0;
      double arc = 0.0;
      double next_mark = smooth.waypoint_spacing;
      std::vector<Point> route{robot};
      double d1 = 0.0;
      double d2 = 0.0;
      Point previous = frame.at(robot_s, profile(robot_s, d1, d2));
      for (double s = robot_s + sample_step; ; s += sample_step) {
        const bool last = s >= out_end;
        if (last) {
          s = out_end;
        }
        const double d = profile(s, d1, d2);
        const Point p = frame.at(s, d);
        const double ds = distance(previous, p);
        const double kappa = std::abs(d2) / std::pow(1.0 + d1 * d1, 1.5);
        curvature_integral += kappa * kappa * ds;
        max_curvature = std::max(max_curvature, kappa);
        arc += ds;
        previous = p;
        if (last) {
          // A mark this close to the rejoin point is replaced by the rejoin point itself
          if (route.size() > 1 && distance(route.back(), p) < 0.5 * smooth.waypoint_spacing) {
            route.pop_back();
          }
          if (!ends_at_goal) {
            route.push_back(p);
          }
          break;
        }
        if (arc >= next_mark) {
          route.push_back(p);
          next_mark += smooth.waypoint_spacing;
        }
      }
      if (route.size() < 2) {
        return e;
      }

      // The LOS tracker drives the legs, and leaves for the next leg switch_radius before each
      // waypoint: the legs and those corner chords must keep the body clear
      std::vector<Point> legs = route;
      legs.push_back(goal);
      double clearance_cost = 0.0;
      double turn_cost = 0.0;
      double length = 0.0;
      for (std::size_t i = 0; i + 1 < legs.size(); ++i) {
        length += distance(legs[i], legs[i + 1]);
        if (!check_leg(legs[i], legs[i + 1], clearance_cost)) {
          return e;
        }
        if (i == 0) {
          continue;
        }
        const Point & p = legs[i - 1];
        const Point & corner = legs[i];
        const Point & q = legs[i + 1];
        const double in = distance(p, corner);
        const double out = distance(corner, q);
        if (in < 1e-6 || out < 1e-6) {
          continue;
        }
        double turn = std::atan2(q.y - corner.y, q.x - corner.x) -
          std::atan2(corner.y - p.y, corner.x - p.x);
        turn = std::abs(std::atan2(std::sin(turn), std::cos(turn)));
        // Every corner here is an inserted waypoint (the goal itself is never a corner of legs)
        if (turn > smooth.hard_turn) {
          return e;
        }
        turn_cost += std::pow(std::max(0.0, turn - smooth.max_turn), 2);
        const double r_in = std::min(params.switch_radius, in);
        const double r_out = std::min(params.switch_radius, out);
        const Point leave{corner.x + (p.x - corner.x) * r_in / in, corner.y + (p.y - corner.y) * r_in / in};
        const Point enter{corner.x + (q.x - corner.x) * r_out / out,
          corner.y + (q.y - corner.y) * r_out / out};
        double unused = 0.0;
        if (!check_leg(leave, enter, unused)) {
          return e;
        }
      }
      const double extra_length = std::max(0.0, length - distance(robot, goal));

      e.valid = true;
      e.max_curvature = max_curvature;
      e.cost = smooth.w_curvature * curvature_integral +
        smooth.w_peak_curvature * max_curvature * max_curvature +
        smooth.w_clearance * clearance_cost + smooth.w_length * extra_length +
        smooth.w_turn * turn_cost +
        (preferred_side != 0 && c.side != preferred_side ? smooth.w_side_switch : 0.0);
      e.route = std::move(route);
      return e;
    };

  std::mt19937 random(smooth.seed);
  const double offset_max = params.max_lateral;
  if (offset_max < required) {
    return plan;
  }
  auto clip = [&](Candidate c) {
      c.offset = std::max(required, std::min(c.offset, offset_max));
      c.hold = std::max(0.0, std::min(c.hold, hold_max));
      c.ramp_in = std::max(smooth.ramp_min, std::min(c.ramp_in, smooth.ramp_max));
      c.ramp_out = std::max(smooth.ramp_min, std::min(c.ramp_out, smooth.ramp_max));
      return c;
    };

  struct Scored
  {
    Candidate candidate;
    Evaluation evaluation;
  };
  std::array<std::vector<Scored>, 2> scored;  // [0] right (-1), [1] left (+1)
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  for (const int side : {-1, 1}) {
    auto & bucket = scored[side > 0 ? 1 : 0];
    for (int i = 0; i < smooth.samples; ++i) {
      Candidate c;
      c.side = side;
      // Favour small offsets: the square pulls most samples towards the required clearance
      const double u = unit(random);
      c.offset = required + u * u * (offset_max - required);
      c.hold = unit(random) * hold_max;
      c.ramp_in = smooth.ramp_min + unit(random) * (smooth.ramp_max - smooth.ramp_min);
      c.ramp_out = smooth.ramp_min + unit(random) * (smooth.ramp_max - smooth.ramp_min);
      Evaluation e = evaluate(c);
      if (e.valid) {
        bucket.push_back({c, std::move(e)});
      }
    }
  }

  // Each side is refined on its own and the sides are compared on their refined paths, so a side
  // is not dropped because its first random draw was unlucky. The MPPI-style weighted mean is
  // taken within one side only: averaging a left and a right detour would drive straight through
  // the obstacle.
  std::normal_distribution<double> normal(0.0, 1.0);
  auto refine_side = [&](std::vector<Scored> & bucket, int side) {
      Evaluation chosen;
      if (bucket.empty()) {
        return chosen;
      }
      auto weighted_mean = [&](Candidate & mean, Candidate & spread) {
          double minimum = std::numeric_limits<double>::infinity();
          for (const auto & s : bucket) {
            minimum = std::min(minimum, s.evaluation.cost);
          }
          double total = 0.0;
          mean = Candidate{};
          mean.side = side;
          mean.offset = mean.hold = mean.ramp_in = mean.ramp_out = 0.0;
          std::vector<double> weights;
          for (const auto & s : bucket) {
            const double w = std::exp(-(s.evaluation.cost - minimum) / smooth.temperature);
            weights.push_back(w);
            total += w;
            mean.offset += w * s.candidate.offset;
            mean.hold += w * s.candidate.hold;
            mean.ramp_in += w * s.candidate.ramp_in;
            mean.ramp_out += w * s.candidate.ramp_out;
          }
          mean.offset /= total;
          mean.hold /= total;
          mean.ramp_in /= total;
          mean.ramp_out /= total;
          spread = Candidate{};
          spread.offset = spread.hold = spread.ramp_in = spread.ramp_out = 0.0;
          for (std::size_t i = 0; i < bucket.size(); ++i) {
            const auto & c = bucket[i].candidate;
            const double w = weights[i] / total;
            spread.offset += w * std::pow(c.offset - mean.offset, 2);
            spread.hold += w * std::pow(c.hold - mean.hold, 2);
            spread.ramp_in += w * std::pow(c.ramp_in - mean.ramp_in, 2);
            spread.ramp_out += w * std::pow(c.ramp_out - mean.ramp_out, 2);
          }
          spread.offset = std::max(0.05, std::sqrt(spread.offset));
          spread.hold = std::max(0.05, std::sqrt(spread.hold));
          spread.ramp_in = std::max(0.1, std::sqrt(spread.ramp_in));
          spread.ramp_out = std::max(0.1, std::sqrt(spread.ramp_out));
        };

      Candidate mean;
      Candidate spread;
      weighted_mean(mean, spread);
      for (int i = 0; i < smooth.refine_samples; ++i) {
        Candidate c = mean;
        c.offset += spread.offset * normal(random);
        c.hold += spread.hold * normal(random);
        c.ramp_in += spread.ramp_in * normal(random);
        c.ramp_out += spread.ramp_out * normal(random);
        c = clip(c);
        Evaluation e = evaluate(c);
        if (e.valid) {
          bucket.push_back({c, std::move(e)});
        }
      }
      weighted_mean(mean, spread);

      // The weighted mean is used when it is valid and not clearly worse than the best sample
      const Scored * best = &bucket.front();
      for (const auto & s : bucket) {
        if (s.evaluation.cost < best->evaluation.cost) {
          best = &s;
        }
      }
      chosen = evaluate(clip(mean));
      if (!chosen.valid || chosen.cost > best->evaluation.cost + smooth.temperature) {
        chosen = best->evaluation;
      }
      return chosen;
    };

  const Evaluation right = refine_side(scored[0], -1);
  const Evaluation left = refine_side(scored[1], 1);
  if (!right.valid && !left.valid) {
    return plan;
  }
  // Exact ties go to the preferred side (the side-switch cost already separates most cases)
  bool take_left;
  if (right.valid != left.valid) {
    take_left = left.valid;
  } else if (left.cost != right.cost) {
    take_left = left.cost < right.cost;
  } else {
    take_left = preferred_side > 0;
  }
  const Evaluation & chosen = take_left ? left : right;
  plan.route = chosen.route;
  plan.side = take_left ? 1 : -1;
  plan.cost = chosen.cost;
  plan.max_curvature = chosen.max_curvature;
  plan.valid_candidates = static_cast<int>(scored[0].size() + scored[1].size());
  return plan;
}

}  // namespace fablab_avoidance
