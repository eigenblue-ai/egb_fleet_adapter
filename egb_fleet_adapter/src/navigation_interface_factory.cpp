// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors
// Licensed under the Apache License, Version 2.0

#include "egb_fleet/navigation_interface_factory.hpp"
#include "egb_fleet/navigation_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace egb_fleet {

NavigationInterfaceFactory::AdapterType
NavigationInterfaceFactory::parse_type(const std::string &type_str) {
  if (type_str == "ros2" || type_str == "Ros2" || type_str == "ROS2") {
    // ROS2 adapter is deprecated, using MPC adapter instead
    return AdapterType::MPC;
  }
  // Default to MPC adapter
  return AdapterType::MPC;
}

std::unique_ptr<NavigationInterface> NavigationInterfaceFactory::create(
    AdapterType type, const std::string &robot_name,
    std::shared_ptr<rclcpp::Node> node, void *zenoh_session,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer, double max_linear_velocity) {
  (void)type;
  (void)max_linear_velocity;

  RCLCPP_INFO(node->get_logger(),
              "Creating navigation controller for robot: %s",
              robot_name.c_str());
  return std::make_unique<NavigationController>(robot_name, node, zenoh_session,
                                                tf_buffer);
}

} // namespace egb_fleet
