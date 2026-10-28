#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <utility>

namespace pointcloud_readiness {
struct Observation {
  bool received = false;
  bool valid = false;
  std::string reason;
  std::string frame;
  int64_t stamp_ns = 0;
  double arrival_age = 0.0;
  double stamp_age = 0.0;
};

inline std::string check_observation(const Observation& cloud, const std::string& label,
                                     double max_age) {
    if (!cloud.received) return label + " not received (check topic, publisher QoS and updater/TF)";
    if (!cloud.valid) return label + " invalid: " + cloud.reason;
    if (cloud.frame.empty() || cloud.stamp_ns <= 0) return label + " has no frame or timestamp";
    if (!std::isfinite(cloud.arrival_age) || !std::isfinite(cloud.stamp_age) ||
        cloud.arrival_age > max_age || cloud.stamp_age > max_age || cloud.stamp_age < -0.25)
      return label + " stale or clock mismatch";
  return {};
}

inline std::string check(const Observation& raw, const Observation& filtered,
                         double max_age) {
  auto error = check_observation(raw, "raw point cloud", max_age);
  if (!error.empty()) return error;
  error = check_observation(filtered, "MoveIt filtered cloud", max_age);
  if (!error.empty()) return error;
  if (raw.frame != filtered.frame) return "raw/filtered point-cloud frames differ";
  if (std::abs(static_cast<double>(raw.stamp_ns - filtered.stamp_ns)) / 1e9 > max_age)
    return "MoveIt updater is not processing recent source clouds";
  return {};
}
}  // namespace pointcloud_readiness
