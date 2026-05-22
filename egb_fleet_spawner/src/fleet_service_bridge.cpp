// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#include <chrono>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>

#include "domain_bridge/domain_bridge.hpp"
#include "egb_fleet_msgs/srv/start_fleet_adapters.hpp"
#include "egb_fleet_msgs/srv/stop_fleet_adapters.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Read domain IDs from environment variables
  // FOREIGN_DOMAIN_ID: foreign domain (where client calls from)
  // ROS_DOMAIN_ID: OpenRMF/Fleet adapter domain (where service server runs)
  const char *foreign_domain_env = std::getenv("FOREIGN_DOMAIN_ID");
  const char *ros_domain_env = std::getenv("ROS_DOMAIN_ID");

  if (!foreign_domain_env || !ros_domain_env) {
    RCLCPP_ERROR(rclcpp::get_logger("fleet_service_bridge"),
                 "Missing required environment variables: "
                 "FOREIGN_DOMAIN_ID=%s, ROS_DOMAIN_ID=%s",
                 foreign_domain_env ? foreign_domain_env : "NOT SET",
                 ros_domain_env ? ros_domain_env : "NOT SET");
    return 1;
  }

  // Parse domain IDs
  size_t to_domain;   // foreign domain (where client needs to call)
  size_t from_domain; // OpenRMF/Fleet adapter domain (where service server is)

  try {
    to_domain = std::stoul(foreign_domain_env);
    from_domain = std::stoul(ros_domain_env);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("fleet_service_bridge"),
                 "Failed to parse domain IDs: %s", e.what());
    return 1;
  }

  // Create domain bridge
  auto options = domain_bridge::DomainBridgeOptions();
  options.mode(domain_bridge::DomainBridgeOptions::Mode::Normal);

  auto bridge = std::make_shared<domain_bridge::DomainBridge>(options);

  RCLCPP_INFO(rclcpp::get_logger("fleet_service_bridge"),
              "Bridging fleet adapter services from domain %zu to domain %zu",
              from_domain, to_domain);

  // Bridge fleet adapter services from domain 45 to domain 41
  // This makes services provided in domain 45 accessible to clients in domain
  // 41
  bridge->bridge_service<egb_fleet_msgs::srv::StartFleetAdapters>(
      "/egb_fleet/start_fleet_adapters", from_domain, to_domain);

  bridge->bridge_service<egb_fleet_msgs::srv::StopFleetAdapters>(
      "/egb_fleet/stop_fleet_adapters", from_domain, to_domain);

  RCLCPP_INFO(rclcpp::get_logger("fleet_service_bridge"),
              "Fleet adapter service bridges created successfully");

  // Create executor and add the bridge to it using its add_to_executor method
  rclcpp::executors::SingleThreadedExecutor executor;
  bridge->add_to_executor(executor);

  // Spin the executor
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
