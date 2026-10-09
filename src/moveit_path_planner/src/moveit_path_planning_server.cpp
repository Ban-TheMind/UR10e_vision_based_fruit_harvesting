#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <cmath>
#include <map>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"
#include "moveit_msgs/msg/orientation_constraint.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "custom_interface/srv/movement_request.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h> 

class MoveitPathPlanningServer {
public:
  MoveitPathPlanningServer(const rclcpp::Node::SharedPtr& node)
  : node_(node)
  {
    using namespace std::placeholders;

    RCLCPP_INFO(node_->get_logger(), "Starting MoveIt Path Planning Server...");

    // Initialize MoveGroupInterface with proper parameters
    // base_link -> tool0
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node_, 
      node_->declare_parameter<std::string>("planning_group", "ur_manipulator"),
      std::shared_ptr<tf2_ros::Buffer>(),
      rclcpp::Duration::from_seconds(5.0)
    );

    move_group_->setPoseReferenceFrame(node_->declare_parameter<std::string>("pose_reference_frame", "base_link"));
    // Configure planner parameters
    node_->declare_parameter("planning_time", 20.0);

    // IMPORTANT!!!!!!!!!!!!!!!!!
    // Refrain urself from loosing the tolerance, if the planning is slow, it's probably not the fault of the tolerance
    // Why?
    // If you increase it to 0.05, there will be a max goal displacement of 0.05m in all axis, that's a nightmare to tune
    node_->declare_parameter("goal_joint_tolerance", 0.001);
    node_->declare_parameter("goal_position_tolerance", 0.001);  
    node_->declare_parameter("goal_orientation_tolerance", 0.001);  

    node_->declare_parameter("execute_enabled", false);
    node_->declare_parameter("max_planning_attempts", 3);
    node_->declare_parameter("velocity_scaling", 0.1);
    node_->declare_parameter("acceleration_scaling", 0.1);
    node_->declare_parameter<std::string>("planner_id", "TRRTkConfigDefault");
    node_->declare_parameter<std::vector<std::string>>("joint_names", {
      "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint", "shoulder_pan_joint"});
    setupCollisionObjects();
    for (const auto &name : {"velocity_scaling", "acceleration_scaling"}) {
      const auto value = node_->get_parameter(name).as_double();
      if (!(value > 0.0 && value <= 1.0)) throw std::runtime_error("Scaling must be in (0, 1]");
    }
    move_group_->setMaxVelocityScalingFactor(node_->get_parameter("velocity_scaling").as_double());
    move_group_->setMaxAccelerationScalingFactor(node_->get_parameter("acceleration_scaling").as_double());

    // Apply parameters
    move_group_->setPlanningTime(node_->get_parameter("planning_time").as_double());
    move_group_->setGoalJointTolerance(node_->get_parameter("goal_joint_tolerance").as_double());
    move_group_->setGoalPositionTolerance(node_->get_parameter("goal_position_tolerance").as_double());
    move_group_->setGoalOrientationTolerance(node_->get_parameter("goal_orientation_tolerance").as_double());
    // move_group_->setPlannerId("RRTConnect");
    // move_group_->setPlannerId("BKPIECEkConfigDefault");
    move_group_->setPlannerId(node_->get_parameter("planner_id").as_string());

    service_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_ = node_->create_service<custom_interface::srv::MovementRequest>(
      "/moveit_path_plan",
      std::bind(&MoveitPathPlanningServer::handle_request, this, _1, _2),
      rmw_qos_profile_services_default, service_group_
    );
  }

	moveit_msgs::msg::Constraints set_constraint(const std::string& constraint_str) {
		moveit_msgs::msg::Constraints constraints;
	
		if (str_contains(constraint_str, ORIEN)) {
			RCLCPP_INFO(node_->get_logger(), "should not be triggered.");
			constraints.orientation_constraints.push_back(set_orientation_constraint(constraint_str));
		}

		// Joint Constraints

		// Elbow -> no less than 60
		//       -> no more than 150
		// Joint constraint for elbow (50° to 145°)

		if (str_contains(constraint_str, ELBOW)) {
			moveit_msgs::msg::JointConstraint elbow_constraint;
			elbow_constraint.joint_name = "elbow_joint";
			
			// Convert degrees to radians
			const double min_angle = 60.0 * M_PI / 180.0;   // 60° in radians (~1.0472)
			const double max_angle = 150.0 * M_PI / 180.0;  // 150° in radians (~2.6179)
			
			// Calculate midpoint (105° which is between 60° and 150°)
			const double midpoint = (min_angle + max_angle) / 2.0;  // ~1.8326 radians
			
			// Set constraints
			elbow_constraint.position = midpoint;
			elbow_constraint.tolerance_below = 1.0;  // ~0.7854 radians (45°)
			elbow_constraint.tolerance_above = max_angle - midpoint;  // ~0.7854 radians (45°)
			elbow_constraint.weight = 1.0;
			
			RCLCPP_INFO(node_->get_logger(), "elbow constraint implemented.");
			
			constraints.joint_constraints.push_back(elbow_constraint);
		}

		if (str_contains(constraint_str, WRIST_1)) {
			moveit_msgs::msg::JointConstraint wrist_constraint;
			wrist_constraint.joint_name = "wrist_1_joint";
			
			// Convert degrees to radians (-71° to -218°)
			const double min_angle = 218.0 * M_PI / 180.0;  // ≈ -3.8048 radians
			const double mid_angle = 144.5 * M_PI / 180.0;  // Midpoint (-144.5° ≈ -2.5220 rad)
			const double max_angle = 71.0 * M_PI / 180.0;   // ≈ -1.2392 radians
		
			// Configure constraint
			wrist_constraint.position = mid_angle;           // Center of range
			wrist_constraint.tolerance_below = 1.7;
			// wrist_constraint.tolerance_below = 0.01;
			// wrist_constraint.tolerance_above = 0.01;
			wrist_constraint.tolerance_above = 1.2828;
			wrist_constraint.weight = 1.0;
			
			constraints.joint_constraints.push_back(wrist_constraint);
		}

		return constraints;
	}

	// -1.389
	//- -3.691542764703268

	// actual value: 3.361864

	// -3.926776

	// -1.244382

	moveit_msgs::msg::OrientationConstraint set_orientation_constraint(const std::string& orien_constraint_str) {
		moveit_msgs::msg::OrientationConstraint orientation_constraint;

		// Common setup for all orientations
		orientation_constraint.header.frame_id = move_group_->getPlanningFrame();
		orientation_constraint.link_name = move_group_->getEndEffectorLink();
		orientation_constraint.absolute_x_axis_tolerance = 0.001;
		orientation_constraint.absolute_y_axis_tolerance = 0.001;
		orientation_constraint.absolute_z_axis_tolerance = 0.001;
		orientation_constraint.weight = 1;
	
		// Set orientation based on input parameter
		tf2::Quaternion q;
		
		if (str_contains(orien_constraint_str, DOWN)) {
			// Facing downward: Z-axis pointing down (Roll=0, Pitch=π, Yaw=0)
			q.setRPY(0, M_PI, 0);
		} 
		else if (str_contains(orien_constraint_str, LEFT)) {
			// Facing left: X-axis pointing left (Roll=0, Pitch=0, Yaw=π/2)
			q.setRPY(0, 0, M_PI_2);
		}
		else if (str_contains(orien_constraint_str, RIGHT)) {
			// Facing right: X-axis pointing right (Roll=0, Pitch=0, Yaw=-π/2)
			q.setRPY(0, 0, -M_PI_2);
		}
		else if (str_contains(orien_constraint_str, UP)) {
			// Facing upward: Z-axis pointing up (Roll=0, Pitch=0, Yaw=0)
			q.setRPY(0, 0, 0);
		}
	
		orientation_constraint.orientation = tf2::toMsg(q);

		return orientation_constraint;
	}

  void handle_request(
    const std::shared_ptr<custom_interface::srv::MovementRequest::Request> request,
    std::shared_ptr<custom_interface::srv::MovementRequest::Response> response)
{
    RCLCPP_INFO(node_->get_logger(), "Received MoveIt path planning request. Command: %s", request->command.c_str());

    if (request->positions.size() != 6 || !std::all_of(request->positions.begin(), request->positions.end(), [](double x) { return std::isfinite(x); })) {
        RCLCPP_ERROR(node_->get_logger(), "Expected 6 position elements, got %zu", request->positions.size());
        response->success = false;
        return;
    }

    // Clear previous targets and constraints
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();

    // Handle different command types
    bool target_set = false;
    if (request->command == "cartesian") {
        target_set = set_cartesian_target(request->positions);
    } 
    else if (request->command == "joint") {
        target_set = set_joint_target(request->positions);
    }
    else {
        RCLCPP_ERROR(node_->get_logger(), "Invalid command: %s", request->command.c_str());
        response->success = false;
        return;
    }

    if (!target_set) {
        response->success = false;
        return;

    }

    // Apply constraints if specified
    if (request->constraints_identifier != NONE) {
        move_group_->setPathConstraints(set_constraint(request->constraints_identifier));
    }

    // Common planning and execution logic
    response->success = plan_and_execute();
}

  void setupCollisionObjects() {
    const auto frame_id = node_->declare_parameter<std::string>("scene_frame", "world");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const std::vector<std::pair<std::string, std::vector<double>>> objects = {
      {"back_wall", {2.4, 0.04, 3.0, 0.70, -0.60, 0.5}},
      {"side_wall", {0.04, 2.4, 3.0, -0.55, 0.25, 0.8}},
      {"table", {3.0, 3.0, 0.01, 0.85, 0.25, 0.05}},
      {"ceiling", {2.4, 2.4, 0.04, 0.85, 0.25, 1.5}}};
    for (const auto &object : objects) {
      const auto values = node_->declare_parameter<std::vector<double>>(object.first, object.second);
      if (values.size() != 6 || !std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); }) || values[0] <= 0 || values[1] <= 0 || values[2] <= 0)
        throw std::runtime_error("Invalid scene box: " + object.first);
      if (!planning_scene_interface.applyCollisionObject(generateCollisionObject(
          values[0], values[1], values[2], values[3], values[4], values[5], frame_id, object.first)))
        throw std::runtime_error("Could not apply collision object: " + object.first);
    }
  }

  auto generateCollisionObject(float sx, float sy, float sz, float x, float y, float z, const std::string& frame_id, const std::string& id) -> moveit_msgs::msg::CollisionObject {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = id;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {sx, sy, sz};

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = x;
    box_pose.position.y = y;
    box_pose.position.z = z;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }

  bool str_contains(const std::string& str, const std::string& substr) {
    if (substr.empty()) return false;
    return str.find(substr) != std::string::npos;
  }

private:
  bool set_cartesian_target(const std::vector<double>& positions) {
    try {
        geometry_msgs::msg::Pose target_pose = create_pose_from_positions(positions);
        move_group_->setPoseTarget(target_pose);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set Cartesian target: %s", e.what());
        return false;
    }
  }

  bool set_joint_target(const std::vector<double>& positions) {
    try {
        auto joint_targets = create_joint_map_from_positions(positions);
        return move_group_->setJointValueTarget(joint_targets);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set joint target: %s", e.what());
        return false;
    }
  }

  geometry_msgs::msg::Pose create_pose_from_positions(const std::vector<double>& positions) {
    tf2::Quaternion q;
    q.setRPY(positions[3], positions[4], positions[5]);
    q.normalize();
    
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = positions[0];
    target_pose.position.y = positions[1];
    target_pose.position.z = positions[2];
    target_pose.orientation = tf2::toMsg(q);
    
    return target_pose;
  }

  std::map<std::string, double> create_joint_map_from_positions(const std::vector<double>& positions) {
    const auto names = node_->get_parameter("joint_names").as_string_array();
    if (names.size() != 6) throw std::runtime_error("joint_names must contain six names");
    std::map<std::string, double> result;
    for (size_t i = 0; i < names.size(); ++i) result[names[i]] = positions[i];
    if (result.size() != 6) throw std::runtime_error("joint_names must be unique");
    return result;
  }

  bool plan_and_execute() {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;
    int attempts = 0;
    const int max_attempts = node_->get_parameter("max_planning_attempts").as_int();
    if (max_attempts < 1 || max_attempts > 10) return false;
    move_group_->setStartStateToCurrentState();
    
    while (!success && attempts < max_attempts) {
        success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        attempts++;
        if (!success) {
            RCLCPP_WARN(node_->get_logger(), "Planning attempt %d failed, retrying...", attempts);

        }
    }
    
    if (success) {
        RCLCPP_INFO(node_->get_logger(), "Plan successful after %d attempts. Executing...", attempts);
        if (!node_->get_parameter("execute_enabled").as_bool()) {
            RCLCPP_INFO(node_->get_logger(), "Plan-only mode: trajectory not executed; movement success is false.");
            return false;
        }
        return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Planning failed after %d attempts.", max_attempts);
        return false;
    }
  }


  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Service<custom_interface::srv::MovementRequest>::SharedPtr service_;
  rclcpp::CallbackGroup::SharedPtr service_group_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;

  const int NO_CONSTRAINT = 0;
  const int ORIENTATION_CONSTRAINT_DOWN = 1;
  const int ORIENTATION_CONSTRAINT_LEFT = 2;
  const std::string DOWN = "DOWN";
  const std::string UP =  "UP";
  const std::string LEFT = "LEFT";
  const std::string RIGHT = "RIGHT";
  const std::string NONE = "NONE";
  const std::string VER_ELBOW = "LEFT_ELBOW";
  const std::string ELBOW = "ELBOW";
  const std::string ORIEN = "ORIEN";
  const std::string WRIST_1= "WRIST1";
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("moveit_path_planning_server");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  int result = 0;
  try {
    MoveitPathPlanningServer server(node);
    spinner.join();
  } catch (const std::exception &error) {
    RCLCPP_ERROR(node->get_logger(), "Planner startup failed: %s", error.what());
    executor.cancel();
    if (spinner.joinable()) spinner.join();
    result = 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
  return 0;
}

// birds-eye for camera horizontal
// ros2 service call /moveit_path_plan custom_interface/srv/MovementRequest "{positions: [0.822, 0.183, 0.856, 0.0, 3.14, 0.0]}"

// birds-eye for camera vertical
// ros2 service call /moveit_path_plan custom_interface/srv/MovementRequest "{positions: [0.64, 0.174, 1.041, -1.56, -0.0, -1.571]}"

// ros2 service call /moveit_path_plan custom_interface/srv/MovementRequest "{positions: [0.44, 0.174, 0.841, -1.56, -0.0, -1.571]}"
// can't push past 0.75

// ros2 service call /moveit_path_plan custom_interface/srv/MovementRequest "{positions: [0.44, 0.16, 0.83,-1.687, 0, -1.61]}"

// ros2 service call /moveit_path_plan custom_interface/srv/MovementRequest "{positions: [0.471,0.149, 1.044, -1.978, 0.058, -1.549]}"
// Object 1: X=1.248m, Y=-0.042m, Z=1.067m

// Home pose Joint
// ros2 service call /moveit_path_plan custom_interface/srv/MovementRequest "{command: 'joint', positions: [-1.3, 1.57, -1.83, -1.57, 0, 0], constraints_identifier: '0'}"