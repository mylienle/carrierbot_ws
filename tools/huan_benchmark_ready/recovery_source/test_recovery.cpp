#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "avoidance_recovery.hpp"

using namespace fablab_avoidance;

int checks = 0;
void check(bool condition, const std::string & name)
{
  if (!condition) {
    throw std::runtime_error(name);
  }
  ++checks;
  std::cout << "PASS " << name << '\n';
}

int main()
{
  try {
    Params params;
    const Point robot{2.4756, -1.6290}, start{0.0, 0.0}, goal{5.0, 0.0};
    std::vector<Point> cylinder;
    for (int i = 0; i < 360; ++i) {
      const double angle = i * 3.14159265358979323846 / 180.0;
      cylinder.push_back({3.0 + 0.20 * std::cos(angle), -2.18975 + 0.20 * std::sin(angle)});
    }
    check(!rejoinRouteClear(cylinder, {robot, goal}, params),
      "observed turn-back geometry rejects direct diagonal approaching the cylinder");
    const auto rejoin = planRejoin(cylinder, robot, start, goal, params);
    check(rejoin.size() == 2 && std::abs(rejoin[1].x - robot.x) < 1e-9 &&
      std::abs(rejoin[1].y) < 1e-9, "turn-back geometry rejoins through original route projection");
    check(rejoinRouteClear(cylinder, {robot, rejoin[1], goal}, params),
      "recovery route and controller corner shortcut meet clearance checks");
    check(rejoinSegmentClear({{0.58, 0.0}}, {0.0, 0.0}, {-1.0, 0.0}, params, true),
      "close point allows only a non-approaching escape prefix");
    check(!rejoinSegmentClear({{0.58, 0.0}}, {0.0, 0.0}, {1.0, 0.0}, params, true),
      "escape prefix approaching a close point is rejected");
    check(!rejoinSegmentClear({{0.50, 0.0}}, {0.0, 0.0}, {-1.0, 0.0}, params, true),
      "point inside the robot envelope cannot authorize recovery");
    check(!rejoinSegmentClear({{0.8, 0.61}}, {0.0, 0.0}, {2.0, 0.0}, params, true),
      "initially clear point must retain full future clearance");
    check(!rejoinSegmentClear({{std::numeric_limits<double>::quiet_NaN(), 0.0}},
      {0.0, 0.0}, {1.0, 0.0}, params, true), "invalid geometry is rejected");
    check(planRejoin({}, robot, start, goal, params).empty(),
      "missing geometry keeps the vehicle waiting");
    check(planRejoin({{2.0, 0.0}}, {1.0, 0.0}, start, goal, params).empty(),
      "obstacle still blocking the original route prevents rejoining");
    auto walled = cylinder;
    for (int i = 0; i <= 100; ++i) {
      walled.push_back({i * 0.05, -1.0});
    }
    check(planRejoin(walled, robot, start, goal, params).empty(),
      "wall crossing both recovery choices keeps waiting");

    Params small;
    small.robot_radius = 0.1;
    small.clearance = 0.01;
    check(!rejoinRouteClear({{0.75, 0.25}}, {{0, 0}, {1, 0}, {1, 1}}, small),
      "waypoint switching chord is checked even when both legs are clear");

    std::vector<std::pair<double, double>> route{
      {0, 0}, {1.1, 0}, {2.14, -1.73}, {3.94, -1.73}, {4.54, 0}, {5, 0}, {6, 1}};
    check(replaceDetour(route, 2, 5, rejoin), "remaining temporary route is replaceable");
    check(route.size() == 6 && route[4] == std::make_pair(5.0, 0.0) &&
      route[5] == std::make_pair(6.0, 1.0), "original goal and later mission waypoints remain ordered");
    const auto saved = route;
    check(!replaceDetour(route, 2, 6, rejoin) && route == saved,
      "invalid route indices do not mutate the mission");
    check(!replaceDetour(route, 2, 4, {}) && route == saved,
      "no valid recovery leaves the previous mission intact");
    check(planRejoin(cylinder, robot, goal, goal, params).empty(),
      "degenerate original mission leg is rejected");

    const Point standing_person{3.20, -0.04}, standing_goal{5.4, 0.0};
    check(!standingRejoinHasClearance({2.9, -2.0}, standing_person, start, standing_goal, .82, 1.95),
      "standing bypass keeps slow speed until the robot has passed the person");
    check(!standingRejoinHasClearance({4.10, -2.04}, standing_person, start, standing_goal, .82, 1.95),
      "standing bypass keeps slow speed if the return would cut inside the planned separation");
    check(standingRejoinHasClearance({4.62, -1.68}, standing_person, start, standing_goal, .82, 1.95),
      "standing bypass can use normal guidance speed after a checked return");

    EarlyRejoinWindow window;
    check(!window.observe(10.0, true) && !window.observe(10.0, true) &&
      !window.observe(10.125, true) && !window.observe(10.25, true) &&
      !window.observe(10.375, true) && window.observe(10.5, true),
      "early rejoin requires half a second of distinct clear scans");
    check(!window.observe(10.625, false) && !window.observe(10.75, true) &&
      !window.observe(11.125, true) && !window.observe(11.25, true) &&
      !window.observe(10.0, true), "blocked, missing or reversed scans restart confirmation");
    std::cout << "ALL " << checks << " CHECKS PASSED\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}
