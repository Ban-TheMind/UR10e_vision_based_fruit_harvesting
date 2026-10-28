#include <cassert>
#include "pointcloud_readiness.hpp"

int main() {
  using pointcloud_readiness::Observation;
  Observation raw, filtered;
  assert(!pointcloud_readiness::check(raw, filtered, 3.0).empty());
  raw = {true, true, "", "camera_depth_optical_frame", 10000000000LL, 0.1, 0.1};
  filtered = raw;
  assert(pointcloud_readiness::check(raw, filtered, 3.0).empty());
  filtered.arrival_age = 3.1;
  assert(!pointcloud_readiness::check(raw, filtered, 3.0).empty());
  filtered = raw; filtered.stamp_age = -1.;
  assert(!pointcloud_readiness::check(raw, filtered, 3.0).empty());
  filtered = raw; filtered.frame = "wrong_frame";
  assert(!pointcloud_readiness::check(raw, filtered, 3.0).empty());
  filtered = raw; filtered.valid = false; filtered.reason = "empty";
  assert(!pointcloud_readiness::check(raw, filtered, 3.0).empty());
  filtered = raw; raw.stamp_age = 4.;
  assert(!pointcloud_readiness::check(raw, filtered, 3.0).empty());
  raw = filtered; raw.stamp_ns = 0;
  assert(!pointcloud_readiness::check(raw, filtered, 3.0).empty());
}
