#include <stdexcept>
#include <cmath>
#include <iostream>
#include <limits>
#include "behavior_gate.hpp"

#define CHECK(condition) do { if (!(condition)) { throw std::runtime_error(#condition); } } while (false)

using fablab_behavior::Action;
using fablab_behavior::Gate;

void release(Gate & gate, int action = 2, double cap = 0.12)
{
  CHECK(gate.receive({1, 10, 1, double(action), cap, 4}, 10, 100, 4));
  CHECK(gate.evaluate(10, 100, 4).action == Action::WAIT);
  CHECK(gate.receive({1, 10.11, 2, double(action), cap, 4}, 10.11, 100.1, 4));
  CHECK(gate.receive({1, 10.22, 3, double(action), cap, 4}, 10.22, 100.2, 4));
  CHECK(gate.evaluate(10.22, 100.2, 4).action == static_cast<Action>(action));
}

int main()
{
  int tests = 0;
  {Gate g; CHECK(g.evaluate(10, 100, 4).action == Action::WAIT); ++tests;}
  {Gate g; release(g); ++tests;}
  {Gate g; release(g); CHECK(g.evaluate(10.6, 100.3, 4).action == Action::WAIT); ++tests;}
  {Gate g; release(g); CHECK(g.evaluate(10.22, 100.81, 4).action == Action::WAIT); ++tests;}
  {Gate g; release(g); CHECK(g.evaluate(10.22, 100.2, 5).action == Action::WAIT); ++tests;}
  {Gate g; release(g); CHECK(!g.receive({1, 10.22, 3, 2, .12, 4}, 10.22, 100.2, 4));
    CHECK(g.evaluate(10.22, 100.2, 4).action == Action::WAIT); ++tests;}
  {Gate g; CHECK(!g.receive({1, 11, 1, 2, .12, 4}, 10, 100, 4)); ++tests;}
  {Gate g; CHECK(!g.receive({1, 10, 1, 1, .2, 4}, 10, 100, 4)); ++tests;}
  {Gate g; CHECK(!g.receive({1, 10, 1, 1, .06, 5}, 10, 100, 4)); ++tests;}
  {Gate g; CHECK(!g.receive({1, 10, 1, 1, std::numeric_limits<double>::quiet_NaN(), 4}, 10, 100, 4)); ++tests;}
  {Gate g; release(g); CHECK(g.receive({1, 10.25, 4, 0, 0, 4}, 10.25, 100.3, 4));
    CHECK(g.evaluate(10.25, 100.3, 4).action == Action::WAIT); ++tests;}
  {Gate g; release(g); CHECK(g.receive({1, 10.25, 4, 1, .06, 4}, 10.25, 100.3, 4));
    auto r = g.evaluate(10.25, 100.3, 4); CHECK(r.action == Action::SLOW);
    double v = .12, w = .2; fablab_behavior::limitCommand(r, v, w);
    CHECK(std::abs(v-.06) < 1e-9 && std::abs(w-.1) < 1e-9); ++tests;}
  {Gate g; release(g, 1, .06); CHECK(g.receive({1, 10.25, 4, 2, .12, 4}, 10.25, 100.3, 4));
    CHECK(g.evaluate(10.25, 100.3, 4).action == Action::SLOW); ++tests;}
  {fablab_behavior::Result r; double v = .12, w = .2; fablab_behavior::limitCommand(r, v, w);
    CHECK(v == 0 && w == 0); ++tests;}
  {Gate g; release(g); const auto r = g.evaluate(10.22, 100.2, 4);
    double v = .03, w = .1; fablab_behavior::limitCommand(r, v, w);
    CHECK(v == .03 && w == .1); ++tests;}
  {Gate g; CHECK(!g.receive({1, 10, 1}, 10, 100, 4)); ++tests;}
  {Gate g; release(g); CHECK(g.evaluate(9.5, 100.3, 4).action == Action::WAIT); ++tests;}
  {Gate g;
    for (int i = 1; i <= 5; ++i) {CHECK(g.receive({1, 10, double(i), 2, .12, 4}, 10, 100+i*.1, 4));}
    CHECK(g.evaluate(10, 100.5, 4).action == Action::WAIT); ++tests;}
  {Gate g; release(g); CHECK(g.receive({1, 10.25, 4, 1, .03, 4}, 10.25, 100.3, 4));
    auto r = g.evaluate(10.25, 100.3, 4); double v = .12, w = .2;
    fablab_behavior::limitCommand(r, v, w); CHECK(std::abs(v-.03) < 1e-9); ++tests;}
  {Gate g; release(g); const auto r = g.evaluate(10.22, 100.2, 4);
    double v = std::numeric_limits<double>::quiet_NaN(), w = .2;
    fablab_behavior::limitCommand(r, v, w); CHECK(v == 0 && w == 0); ++tests;}
  {Gate g; release(g); const auto r = g.evaluate(10.22, 100.2, 4);
    const auto approach = [&](double los, double remaining) {
      return fablab_behavior::finalApproachSpeed(r, los, remaining, .15, .6, .03, .08);
    };
    CHECK(approach(.11, 2.6) == .11);
    CHECK(std::abs(approach(.03, .5) - .0688888888888889) < 1e-9);
    CHECK(approach(.10, .5) == .08);
    CHECK(approach(.03, .16) > .03 && approach(.03, .16) < .032);
    CHECK(approach(.03, .15) == 0.0);
    CHECK(approach(.03, std::numeric_limits<double>::quiet_NaN()) == 0.0);
    ++tests;}
  {Gate g; release(g, 1, .06); const auto r = g.evaluate(10.22, 100.2, 4);
    CHECK(fablab_behavior::finalApproachSpeed(r, .03, .5, .15, .6, .03, .08) == .03);
    ++tests;}
  {Gate g; release(g); const auto r = g.evaluate(10.22, 100.2, 4);
    const auto cruise = [&](double remaining) {
      return fablab_behavior::clearCruiseSpeed(r, .08, remaining, .6, .12);
    };
    CHECK(cruise(1.2) == .12);
    CHECK(std::abs(cruise(.8) - .10) < 1e-9);
    CHECK(cruise(.6) == .08 && cruise(.15) == .08);
    CHECK(cruise(std::numeric_limits<double>::quiet_NaN()) == .08);
    CHECK(fablab_behavior::clearCruiseSpeed(r, .06, 1.2, .6, .09) == .09);
    ++tests;}
  {Gate g; release(g, 2, .09); auto r = g.evaluate(10.22, 100.2, 4);
    CHECK(fablab_behavior::clearCruiseSpeed(r, .06, 1.2, .6, .12) == .09);
    CHECK(g.receive({1, 10.25, 4, 1, .06, 4}, 10.25, 100.3, 4));
    r = g.evaluate(10.25, 100.3, 4);
    CHECK(fablab_behavior::clearCruiseSpeed(r, .05, 1.2, .6, .12) == .05);
    CHECK(g.receive({1, 10.26, 5, 0, 0, 4}, 10.26, 100.31, 4));
    r = g.evaluate(10.26, 100.31, 4);
    double v = fablab_behavior::clearCruiseSpeed(r, .05, 1.2, .6, .12), w = .03;
    fablab_behavior::limitCommand(r, v, w);
    CHECK(v == 0.0 && w == 0.0);
    ++tests;}
  {Gate g;
    CHECK(!fablab_behavior::mayInsertDetour(g.evaluate(10, 100, 4), true));
    release(g, 1, .06);
    const auto slow = g.evaluate(10.22, 100.2, 4);
    CHECK(!fablab_behavior::mayInsertDetour(slow, true));
    CHECK(fablab_behavior::mayInsertDetour(slow, false));
    double v = .12, w = .2;
    fablab_behavior::limitCommand(slow, v, w);
    CHECK(std::abs(v-.06) < 1e-9 && std::abs(w-.1) < 1e-9);
    ++tests;}
  {Gate g; release(g);
    CHECK(fablab_behavior::mayInsertDetour(g.evaluate(10.22, 100.2, 4), true));
    ++tests;}
  {Gate g; release(g);
    CHECK(g.receive({2, 10.25, 4, 0, 0, 4, 3.2, -0.1}, 10.25, 100.3, 4));
    auto r = g.evaluate(10.25, 100.3, 4);
    CHECK(r.action == Action::WAIT && r.plan_while_waiting && r.blocker_x == 3.2 && r.blocker_y == -0.1);
    double v = .06, w = .1; fablab_behavior::limitCommand(r, v, w);
    CHECK(v == 0 && w == 0);
    r = g.evaluate(10.26, 100.31, 5);
    CHECK(r.action == Action::WAIT && !r.plan_while_waiting);
    CHECK(g.receive({1, 10.27, 5, 1, .06, 5}, 10.27, 100.32, 5));
    CHECK(g.evaluate(10.27, 100.32, 5).action == Action::WAIT);
    ++tests;}
  {Gate g;
    CHECK(!g.receive({2, 10, 1, 1, .06, 4, 3.2, 0}, 10, 100, 4));
    CHECK(!g.receive({2, 10, 1, 0, 0, 4, std::numeric_limits<double>::quiet_NaN(), 0}, 10, 100, 4));
    CHECK(g.receive({2, 10, 1, 0, 0, 4, 3.2, 0}, 10, 100, 4));
    CHECK(!g.evaluate(10.31, 100.2, 4).plan_while_waiting);
    ++tests;}
  std::cout << "PASS: " << tests << " behavior gate checks\n";
}
