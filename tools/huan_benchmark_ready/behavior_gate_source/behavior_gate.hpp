#pragma once
// Protocol v1: [version, ROS stamp, sequence, action, speed cap, route revision].
// Protocol v2 appends a matched blocker centre in map coordinates to a WAIT only.
// Actions: 0 WAIT, 1 SLOW, 2 BASELINE (original LiDAR guidance remains in charge).
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace fablab_behavior
{
enum class Action {WAIT = 0, SLOW = 1, BASELINE = 2};
struct Result
{
  Action action{Action::WAIT};
  double speed_cap{0.0}, age_sim{0.0}, age_wall{0.0};
  std::uint64_t sequence{0};
  const char * reason{"no_directive"};
  bool plan_while_waiting{false};
  double blocker_x{0.0}, blocker_y{0.0};
};

class Gate
{
public:
  void forceWait(const char * reason)
  {
    valid_ = false;
    effective_ = target_ = Action::WAIT;
    effective_cap_ = 0.0;
    confirmations_ = 0;
    plan_while_waiting_ = false;
    reason_ = reason;
  }

  bool receive(const std::vector<double> & wire, double now, double wall,
    std::uint64_t revision)
  {
    const bool plan = wire.size() == 8 && wire[0] == 2.0 && wire[3] == 0;
    const bool regular = wire.size() == 6 && wire[0] == 1.0;
    if ((!regular && !plan) || !std::isfinite(now) || !std::isfinite(wall)) {
      forceWait("invalid_directive"); return false;
    }
    for (double value : wire) {
      if (!std::isfinite(value)) {forceWait("invalid_directive"); return false;}
    }
    const auto integer = [](double x) {return x >= 0 && x <= 9007199254740991.0 && std::floor(x) == x;};
    if (!integer(wire[2]) || wire[2] <= sequence_ ||
      !integer(wire[3]) || wire[3] > 2 || !integer(wire[5]) ||
      wire[5] != static_cast<double>(revision) ||
      now-wire[1] < -0.02 || now-wire[1] > 0.3 || wire[4] < 0 || wire[4] > 0.12 ||
      (wire[3] == 0 && wire[4] != 0) ||
      (wire[3] == 1 && (wire[4] <= 0 || wire[4] > 0.06)))
    {
      forceWait("invalid_or_old_directive"); return false;
    }
    const Action requested = static_cast<Action>(static_cast<int>(wire[3]));
    sequence_ = static_cast<std::uint64_t>(wire[2]);
    revision_ = revision;
    stamp_ = wire[1]; wall_ = wall; ever_ = valid_ = true;
    plan_while_waiting_ = plan;
    if (plan) {
      blocker_x_ = wire[6];
      blocker_y_ = wire[7];
    }
    const double cap = requested == Action::WAIT ? 0.0 : wire[4];
    if (requested == Action::WAIT) {
      effective_ = target_ = Action::WAIT;
      effective_cap_ = target_cap_ = 0.0; confirmations_ = 0;
    } else {
      if (requested != target_ || cap != target_cap_) {
        target_ = requested; target_cap_ = cap; since_ = now; confirmations_ = 1;
      } else {
        ++confirmations_;
      }
      if (static_cast<int>(requested) < static_cast<int>(effective_) ||
        (requested == effective_ && cap < effective_cap_))
      {
        effective_ = requested; effective_cap_ = cap;
      }
    }
    reason_ = "directive_received";
    return true;
  }

  Result evaluate(double now, double wall, std::uint64_t revision)
  {
    Result result;
    result.sequence = sequence_;
    if (ever_) {
      result.age_sim = std::max(0.0, now-stamp_);
      result.age_wall = std::max(0.0, wall-wall_);
    }
    if (!std::isfinite(now) || !std::isfinite(wall)) {forceWait("invalid_clock");}
    else if (valid_ && revision_ != revision) {forceWait("route_revision_changed");}
    else if (valid_ && (now-stamp_ < -0.02 || now-stamp_ > 0.3 || wall-wall_ < 0 || wall-wall_ > 0.6)) {
      forceWait("directive_timeout");
    }
    if (!valid_) {result.reason = reason_; return result;}
    if (confirmations_ >= 3 && now-since_ >= 0.2) {
      effective_ = target_; effective_cap_ = target_cap_;
    }
    result.action = effective_;
    result.speed_cap = effective_cap_;
    result.plan_while_waiting = plan_while_waiting_;
    result.blocker_x = blocker_x_;
    result.blocker_y = blocker_y_;
    result.reason = effective_ == Action::WAIT ?
      (target_ == Action::WAIT ? "directive_wait" : "release_hysteresis") :
      (effective_ == Action::SLOW ? "directive_slow" : "baseline_lidar_guidance");
    return result;
  }

private:
  bool valid_{false}, ever_{false};
  bool plan_while_waiting_{false};
  double blocker_x_{0.0}, blocker_y_{0.0};
  Action effective_{Action::WAIT}, target_{Action::WAIT};
  double effective_cap_{0.0}, target_cap_{0.0}, stamp_{0.0}, wall_{0.0}, since_{0.0};
  std::uint64_t sequence_{0}, revision_{0};
  int confirmations_{0};
  const char * reason_{"no_directive"};
};

inline double finalApproachSpeed(const Result & gate, double los_speed, double remaining,
  double goal_radius, double slowdown_distance, double minimum_speed, double approach_speed)
{
  if (gate.action != Action::BASELINE) {
    return los_speed;
  }
  if (!std::isfinite(remaining) || remaining <= goal_radius ||
    !(slowdown_distance > goal_radius) || !(approach_speed >= minimum_speed) ||
    !(minimum_speed > 0.0))
  {
    return 0.0;
  }
  // Anchor the taper to the final slowdown window, rather than the full leg length.
  const double fraction = std::min(1.0, (remaining - goal_radius) /
    (slowdown_distance - goal_radius));
  const double floor = minimum_speed + fraction * (approach_speed - minimum_speed);
  const double requested = std::max(los_speed, floor);
  return remaining <= slowdown_distance ? std::min(requested, approach_speed) : requested;
}

inline bool mayInsertDetour(const Result & gate, bool yield_before_detour)
{
  return !yield_before_detour || gate.action == Action::BASELINE;
}

inline double clearCruiseSpeed(const Result & gate, double los_speed, double remaining,
  double slowdown_distance, double configured_speed)
{
  if (gate.action != Action::BASELINE || !std::isfinite(remaining) || remaining <= slowdown_distance) {
    return los_speed;
  }
  // Blend back to the existing LOS request before entering the slowdown window.
  constexpr double transition_distance = 0.4;
  const double blend = std::min(1.0, (remaining - slowdown_distance) / transition_distance);
  const double cruise = std::min(configured_speed, gate.speed_cap);
  return los_speed + blend * std::max(0.0, cruise - los_speed);
}

inline void limitCommand(const Result & gate, double & linear, double & angular)
{
  if (!std::isfinite(linear) || !std::isfinite(angular) || gate.action == Action::WAIT) {
    linear = angular = 0.0; return;
  }
  // Cap both components together so a speed reduction preserves curvature.
  // The baseline cap is also a bound, never a request to accelerate.
  if (std::abs(linear) > gate.speed_cap && std::abs(linear) > 1e-12) {
    const double scale = gate.speed_cap/std::abs(linear);
    linear *= scale; angular *= scale;
  }
}
}  // namespace fablab_behavior
