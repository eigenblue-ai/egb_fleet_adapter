// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors
// Python ref: fleet_adapter.py (672 lines)

#ifndef EGB_FLEET__FLEET_ADAPTER_HPP_
#define EGB_FLEET__FLEET_ADAPTER_HPP_

#include <atomic>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rmf_fleet_adapter/agv/Adapter.hpp>
#include <rmf_fleet_adapter/agv/Transformation.hpp>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

// Forward declarations for tf2_ros
namespace tf2_ros {
class Buffer;
class TransformListener;
} // namespace tf2_ros

namespace egb_fleet {

// Forward declarations
class RobotState;
class RestRobotDiscovery;

class FleetAdapter {
public:
  explicit FleetAdapter(const std::string &config_path);
  void start();
  void shutdown();

private:
  void initialize_fleet();
  void update_loop();
  void initialize_static_robots();
  void initialize_discovery();

  /// Action categories claimed by the configured plugins, deduplicated.
  std::vector<std::string> performable_plugin_actions() const;

  YAML::Node config_;
  std::string nav_graph_id_;
  std::string nav_graph_url_;
  std::string building_id_;
  double nav_graph_timeout_s_ = 30.0;
  std::shared_ptr<rmf_fleet_adapter::agv::Adapter> adapter_;
  std::shared_ptr<rmf_fleet_adapter::agv::FleetUpdateHandle> fleet_handle_;
  std::shared_ptr<const rmf_traffic::agv::Graph> graph_;

  // Robot management
  std::map<std::string, std::shared_ptr<RobotState>> robots_;
  std::map<std::string,
           std::shared_ptr<rmf_fleet_adapter::agv::RobotCommandHandle>>
      command_handles_;
  std::shared_ptr<RestRobotDiscovery> discovery_;

  // Shared resources
  void *zenoh_session_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::map<std::string, rmf_fleet_adapter::agv::Transformation>
      coordinate_transforms_;

  // Must outlive the executor — destruction order matters.
  rclcpp::TimerBase::SharedPtr update_timer_;

  std::atomic<bool> shutdown_requested_{false};
};

} // namespace egb_fleet

#endif // EGB_FLEET__FLEET_ADAPTER_HPP_
