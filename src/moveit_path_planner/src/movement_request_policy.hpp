#pragma once

#include <cmath>
#include <map>
#include <string>
#include <vector>

namespace movement_request_policy {

enum class Command {
  Invalid,
  PlanCartesian,
  PlanJoint,
  ExecuteCartesian,
  ExecuteJoint,
};

inline Command parse_command(const std::string& command) {
  if (command == "plan_cartesian") return Command::PlanCartesian;
  if (command == "plan_joint") return Command::PlanJoint;
  if (command == "cartesian") return Command::ExecuteCartesian;
  if (command == "joint") return Command::ExecuteJoint;
  return Command::Invalid;
}

inline bool is_execution(Command command) {
  return command == Command::ExecuteCartesian || command == Command::ExecuteJoint;
}

inline bool is_cartesian(Command command) {
  return command == Command::PlanCartesian || command == Command::ExecuteCartesian;
}

inline bool valid_positions(const std::vector<double>& positions) {
  if (positions.size() != 6) return false;
  for (double value : positions) {
    if (!std::isfinite(value)) return false;
  }
  return true;
}

// UR joint order, in radians: base, shoulder, elbow, wrist 1, wrist 2, wrist 3.
inline std::map<std::string, double> joint_targets(const std::vector<double>& positions) {
  return {
    {"shoulder_pan_joint", positions[0]},
    {"shoulder_lift_joint", positions[1]},
    {"elbow_joint", positions[2]},
    {"wrist_1_joint", positions[3]},
    {"wrist_2_joint", positions[4]},
    {"wrist_3_joint", positions[5]},
  };
}

}  // namespace movement_request_policy
