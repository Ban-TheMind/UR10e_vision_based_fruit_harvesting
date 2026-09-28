#include <cassert>
#include <limits>
#include <vector>

#include "movement_request_policy.hpp"

using movement_request_policy::Command;

int main() {
  using namespace movement_request_policy;

  assert(parse_command("plan_cartesian") == Command::PlanCartesian);
  assert(parse_command("plan_joint") == Command::PlanJoint);
  assert(!is_execution(parse_command("plan_cartesian")));
  assert(!is_execution(parse_command("plan_joint")));
  assert(is_execution(parse_command("cartesian")));
  assert(is_execution(parse_command("joint")));
  assert(parse_command("move") == Command::Invalid);

  assert(!valid_positions({1.0, 2.0}));
  assert(!valid_positions({1.0, 2.0, 3.0, 4.0, 5.0,
                           std::numeric_limits<double>::quiet_NaN()}));
  assert(!valid_positions({1.0, 2.0, 3.0, 4.0, 5.0,
                           std::numeric_limits<double>::infinity()}));

  const std::vector<double> ur_joints{-1.9, 1.2, -2.0, -1.57, 0.0, 0.25};
  assert(valid_positions(ur_joints));
  const auto targets = joint_targets(ur_joints);
  assert(targets.at("shoulder_pan_joint") == -1.9);
  assert(targets.at("shoulder_lift_joint") == 1.2);
  assert(targets.at("elbow_joint") == -2.0);
  assert(targets.at("wrist_1_joint") == -1.57);
  assert(targets.at("wrist_2_joint") == 0.0);
  assert(targets.at("wrist_3_joint") == 0.25);
}
