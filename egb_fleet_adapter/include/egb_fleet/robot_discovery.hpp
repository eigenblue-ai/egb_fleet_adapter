// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors
// Python ref: rest_robot_discovery.py (418 lines)

#ifndef EGB_FLEET__ROBOT_DISCOVERY_HPP_
#define EGB_FLEET__ROBOT_DISCOVERY_HPP_

#include "egb_fleet/robot_state.hpp"
#include <chrono>
#include <map>
#include <memory>
#include <rmf_fleet_adapter/agv/Transformation.hpp>
#include <set>
#include <string>
#include <yaml-cpp/yaml.h>

namespace egb_fleet {

class RestRobotDiscovery {
public:
  RestRobotDiscovery(
      std::shared_ptr<rclcpp::Node> node, void *zenoh_session,
      std::shared_ptr<rmf_fleet_adapter::agv::FleetUpdateHandle> fleet_handle,
      std::shared_ptr<const rmf_traffic::agv::Graph> graph,
      const YAML::Node &config, const YAML::Node &discovery_config,
      const YAML::Node &robot_template_config, const YAML::Node &plugins_config,
      const std::map<std::string, rmf_fleet_adapter::agv::Transformation>
          &coordinate_transforms);

  std::map<std::string, std::shared_ptr<RobotState>> discover_robots();

private:
  std::set<std::string> scan_for_robots_via_rest();
  bool add_dynamic_robot(const std::string &robot_name);

  std::shared_ptr<rclcpp::Node> node_;
  void *zenoh_session_;
  std::shared_ptr<rmf_fleet_adapter::agv::FleetUpdateHandle> fleet_handle_;
  std::shared_ptr<const rmf_traffic::agv::Graph> graph_;
  std::set<std::string> discovered_robots_;
  std::map<std::string, std::shared_ptr<RobotState>> robots_;
  std::map<std::string,
           std::shared_ptr<rmf_fleet_adapter::agv::RobotCommandHandle>>
      command_handles_;

  // Discovery timing
  std::chrono::steady_clock::time_point last_discovery_time_;
  double discovery_frequency_{0.1}; // Hz, default 0.1 Hz = every 10 seconds

  // Configuration
  YAML::Node config_;
  YAML::Node discovery_config_;
  YAML::Node robot_template_config_;
  YAML::Node plugins_config_;
  std::string robot_name_pattern_;
  std::string rest_endpoint_;
  std::map<std::string, rmf_fleet_adapter::agv::Transformation>
      coordinate_transforms_;
};

} // namespace egb_fleet

#endif // EGB_FLEET__ROBOT_DISCOVERY_HPP_
