cmake_minimum_required(VERSION 3.10)
project(moveit_path_planner)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(custom_interface REQUIRED)

add_executable(moveit_path_planning_server
  src/moveit_path_planning_server.cpp
)

ament_target_dependencies(moveit_path_planning_server
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  sensor_msgs
  std_srvs
  octomap_msgs
  shape_msgs
  moveit_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  custom_interface
)

target_include_directories(moveit_path_planning_server PRIVATE ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(moveit_path_planning_server ${OCTOMAP_LIBRARIES} Threads::Threads)

install(TARGETS moveit_path_planning_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
