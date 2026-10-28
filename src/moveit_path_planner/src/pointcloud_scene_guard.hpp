#pragma once

#include <chrono>
#include <cstring>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "pointcloud_readiness.hpp"

// Observe sensor health and query the *actual move_group scene*. The observer
// has its own executor so a blocking planning-service callback cannot prevent
// TF, cloud callbacks or the GetPlanningScene response from being processed.
class PointCloudSceneGuard {
 public:
  explicit PointCloudSceneGuard(const rclcpp::Node::SharedPtr& owner) : owner_(owner) {
    enabled_ = parameter<bool>("pointcloud_avoidance", false);
    if (!enabled_) return;
    frame_ = parameter<std::string>("octomap_frame", "base_link");
    resolution_ = parameter<double>("octomap_resolution", 0.01);
    max_age_ = parameter<double>("pointcloud_max_age", 3.0);
    timeout_ = parameter<double>("scene_service_timeout", 2.0);
    raw_topic_ = parameter<std::string>("point_cloud_topic", "/camera/depth/color/points");
    const auto filtered_topic = parameter<std::string>("filtered_cloud_topic", "/harvesting/filtered_points");
    const auto service = parameter<std::string>("planning_scene_service", "/get_planning_scene");
    if (frame_.empty() || raw_topic_.empty() || filtered_topic.empty() ||
        raw_topic_ == filtered_topic || !std::isfinite(resolution_) || resolution_ <= 0 ||
        !std::isfinite(max_age_) || max_age_ <= 0 || !std::isfinite(timeout_) || timeout_ <= 0)
      throw std::invalid_argument("Invalid point-cloud guard configuration");
    auto options = rclcpp::NodeOptions().use_global_arguments(false);
    observer_ = std::make_shared<rclcpp::Node>("harvesting_pointcloud_guard", owner->get_namespace(), options);
    buffer_ = std::make_shared<tf2_ros::Buffer>(owner->get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, observer_, false);
    raw_sub_ = observer_->create_subscription<sensor_msgs::msg::PointCloud2>(
        raw_topic_, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { receive(*msg, raw_); });
    filtered_sub_ = observer_->create_subscription<sensor_msgs::msg::PointCloud2>(
        filtered_topic, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { receive(*msg, filtered_); });
    scene_client_ = observer_->create_client<moveit_msgs::srv::GetPlanningScene>(service);
    executor_.add_node(observer_);
    thread_ = std::thread([this]() { executor_.spin(); });
  }

  ~PointCloudSceneGuard() {
    executor_.cancel();
    if (thread_.joinable()) thread_.join();
  }

  bool enabled() const { return enabled_; }
  const std::string& reason() const { return last_error_; }

  bool ready() {
    if (!enabled_) return true;
    try {
      auto error = sensor_error();
      if (!error.empty()) return reject(error);
      const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(timeout_));
      if (!scene_client_->wait_for_service(timeout)) return reject("move_group GetPlanningScene service unavailable");
      auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
      request->components.components = moveit_msgs::msg::PlanningSceneComponents::OCTOMAP;
      // At most one outstanding request, even if a live service stops replying.
      if (pending_scene_.valid()) {
        if (pending_scene_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
          return reject("previous GetPlanningScene request is still pending");
        pending_scene_ = {};
      }
      pending_scene_ = scene_client_->async_send_request(request);
      if (pending_scene_.wait_for(timeout) != std::future_status::ready) return reject("GetPlanningScene timed out");
      const auto response = pending_scene_.get();
      pending_scene_ = {};
      if (!response) return reject("GetPlanningScene returned no response");
      const auto& map = response->scene.world.octomap;
      // Foxy does not stamp GetPlanningScene Octomaps: freshness is established
      // using raw and updater-filtered cloud headers/steady-clock receipt times.
      if (map.header.frame_id != frame_) return reject("scene map frame differs from configured octomap_frame");
      if (map.octomap.data.empty()) return reject("move_group scene contains no OctoMap data");
      if (!std::isfinite(map.octomap.resolution) ||
          std::abs(map.octomap.resolution - resolution_) > 1e-8)
        return reject("scene OctoMap resolution differs from configured updater");
      std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap_msgs::msgToMap(map.octomap));
      auto* tree = dynamic_cast<octomap::OcTree*>(abstract_tree.get());
      if (!tree) return reject("scene OctoMap cannot be decoded as OcTree");
      bool occupied = false;
      for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it) {
        if (tree->isNodeOccupied(*it)) { occupied = true; break; }
      }
      if (!occupied) return reject("scene OctoMap has no occupied cells; empty space is not mapping readiness");
      error = sensor_error();
      if (!error.empty()) return reject(error);
      last_error_.clear();
      return true;
    } catch (const std::exception& error) {
      return reject(std::string("map readiness check failed: ") + error.what());
    }
  }

 private:
  template <typename T> T parameter(const std::string& name, const T& value) {
    if (!owner_->has_parameter(name)) owner_->declare_parameter<T>(name, value);
    return owner_->get_parameter(name).get_value<T>();
  }

  struct Sample {
    pointcloud_readiness::Observation observation;
    std::chrono::steady_clock::time_point arrival;
  };

  void receive(const sensor_msgs::msg::PointCloud2& msg, Sample& destination) {
    Sample sample;
    sample.arrival = std::chrono::steady_clock::now();
    auto& value = sample.observation;
    value.received = true;
    value.frame = msg.header.frame_id;
    value.stamp_ns = static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL + msg.header.stamp.nanosec;
    value.reason = "empty, malformed, non-FLOAT32 XYZ, or no finite points";
    uint32_t offsets[3] = {};
    bool fields[3] = {};
    for (const auto& field : msg.fields) {
      for (int i = 0; i < 3; ++i) {
        if (field.name == std::string(1, "xyz"[i]) &&
            field.datatype == sensor_msgs::msg::PointField::FLOAT32 && field.count == 1 &&
            field.offset <= msg.point_step && msg.point_step - field.offset >= sizeof(float)) {
          fields[i] = true; offsets[i] = field.offset;
        }
      }
    }
    if (!msg.is_bigendian && fields[0] && fields[1] && fields[2] && msg.width && msg.height &&
        static_cast<uint64_t>(msg.row_step) >= static_cast<uint64_t>(msg.width) * msg.point_step &&
        msg.data.size() >= static_cast<uint64_t>(msg.row_step) * msg.height) {
      for (uint32_t row = 0; row < msg.height && !value.valid; ++row) {
        for (uint32_t col = 0; col < msg.width && !value.valid; ++col) {
          const auto* point = msg.data.data() + static_cast<size_t>(row) * msg.row_step + static_cast<size_t>(col) * msg.point_step;
          float xyz[3];
          for (int i = 0; i < 3; ++i) std::memcpy(&xyz[i], point + offsets[i], sizeof(float));
          value.valid = std::isfinite(xyz[0]) && std::isfinite(xyz[1]) && std::isfinite(xyz[2]);
        }
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    destination = std::move(sample);
  }

  std::string sensor_error() {
    Sample raw, filtered;
    { std::lock_guard<std::mutex> lock(mutex_); raw = raw_; filtered = filtered_; }
    const auto steady_now = std::chrono::steady_clock::now();
    const auto now_ns = owner_->get_clock()->now().nanoseconds();
    for (auto* sample : {&raw, &filtered}) {
      sample->observation.arrival_age = std::chrono::duration<double>(steady_now - sample->arrival).count();
      sample->observation.stamp_age = static_cast<double>(now_ns - sample->observation.stamp_ns) / 1e9;
    }
    auto error = pointcloud_readiness::check_observation(raw.observation, "raw point cloud", max_age_);
    if (!error.empty()) return error;
    if (!buffer_->canTransform(frame_, raw.observation.frame,
          rclcpp::Time(raw.observation.stamp_ns, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.)))
      return "TF unavailable from cloud frame to map frame at cloud timestamp";
    return pointcloud_readiness::check(raw.observation, filtered.observation, max_age_);
  }

  bool reject(const std::string& reason) {
    last_error_ = reason;
    RCLCPP_ERROR(owner_->get_logger(), "Point-cloud planning rejected: %s", reason.c_str());
    return false;
  }

  rclcpp::Node::SharedPtr owner_, observer_;
  bool enabled_ = false;
  std::string frame_, raw_topic_, last_error_;
  double resolution_ = 0., max_age_ = 0., timeout_ = 0.;
  std::mutex mutex_;
  Sample raw_, filtered_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_sub_, filtered_sub_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_client_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedFuture pending_scene_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread thread_;
};
