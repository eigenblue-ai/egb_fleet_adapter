// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors
// Licensed under the Apache License, Version 2.0

#ifndef EGB_FLEET__NAVIGATION_INTERFACE_FACTORY_HPP_
#define EGB_FLEET__NAVIGATION_INTERFACE_FACTORY_HPP_

#include "egb_fleet/navigation_interface.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_ros/buffer.h>

namespace egb_fleet {

/**
 * Factory for creating NavigationInterface instances
 *
 * Selects between different adapter implementations based on configuration:
 * - ROS2: Original ROS2-based navigation stack
 * - MPC: MPC controller-based local planner with minimum-time via-points
 */
class NavigationInterfaceFactory {
public:
  enum class AdapterType { ROS2, MPC };

  /**
   * Create a navigation controller instance
   * @param type Type of adapter to create
   * @param robot_name Name of the robot
   * @param node ROS2 node for subscriptions/clients
   * @param zenoh_session Zenoh session for cross-domain communication
   * @param tf_buffer TF2 buffer for coordinate transforms
   * @param max_linear_velocity Maximum linear velocity for the robot (m/s)
   * @return Unique pointer to the created navigation controller
   */
  static std::unique_ptr<NavigationInterface>
  create(AdapterType type, const std::string &robot_name,
         std::shared_ptr<rclcpp::Node> node, void *zenoh_session,
         std::shared_ptr<tf2_ros::Buffer> tf_buffer,
         double max_linear_velocity = 2.5);

  /**
   * Parse adapter type from configuration string
   * @param type_str Configuration string ("ros2" or "mpc")
   * @return Parsed AdapterType enum (defaults to ROS2 if unrecognized)
   */
  static AdapterType parse_type(const std::string &type_str);
};

} // namespace egb_fleet

#endif // EGB_FLEET__NAVIGATION_INTERFACE_FACTORY_HPP_
