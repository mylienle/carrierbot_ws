#include <stdexcept>
#include <iostream>
#include <limits>
#include <route_preview.hpp>

#define CHECK(condition) do { if (!(condition)) { throw std::runtime_error(#condition); } } while (false)

int main()
{
  fablab_route_preview::Snapshot snapshot;
  snapshot.stamp = snapshot.pose_tf_stamp = 10.0;
  snapshot.has_pose = snapshot.pose_tf_valid = true;
  snapshot.speed = snapshot.max_speed = 0.12;
  snapshot.sequence = 7;
  snapshot.segment = 1;
  snapshot.waypoints = {{0, 0}, {1, 0}, {1, 1}, {5, 1}};
  const auto serialized = fablab_route_preview::serialize(snapshot);
  CHECK(!serialized.empty());
  std::cout << serialized << '\n';
  snapshot.x = std::numeric_limits<double>::quiet_NaN();
  CHECK(fablab_route_preview::serialize(snapshot).empty());
  snapshot.x = 0;
  snapshot.waypoints.back().second = std::numeric_limits<double>::infinity();
  CHECK(fablab_route_preview::serialize(snapshot).empty());
  snapshot.waypoints = {{0, 0}};
  std::cout << fablab_route_preview::serialize(snapshot) << '\n';
}
