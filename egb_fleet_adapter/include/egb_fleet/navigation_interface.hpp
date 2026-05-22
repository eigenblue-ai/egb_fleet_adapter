// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors
// Licensed under the Apache License, Version 2.0

#ifndef EGB_FLEET__NAVIGATION_INTERFACE_HPP_
#define EGB_FLEET__NAVIGATION_INTERFACE_HPP_

#include <array>
#include <egb_fleet_msgs/msg/waypoint.hpp>
#include <functional>
#include <memory>
#include <optional>
#include <rmf_traffic/agv/Planner.hpp>
#include <vector>

namespace egb_fleet {

/**
 * Abstract interface for robot navigation control adapters.
 * Implementations include Ros2Adapter (existing) and MpcAdapter (new).
 * This enables a plugin architecture where different navigation controllers
 * can be swapped via configuration.
 */
class NavigationInterface {
public:
  virtual ~NavigationInterface() = default;

  /**
   * Execute a navigation path
   * @param waypoints Waypoints in the internal format (already converted and in
   * robot coordinates)
   * @param completion_callback Called when navigation completes or fails
   * @param arrival_estimator_callback Called periodically with
   * (target_waypoint_idx, eta_seconds) This callback is used by RMF to track
   * task progress and estimate arrival times
   */
  virtual void
  execute_path(const std::vector<egb_fleet_msgs::msg::Waypoint> &waypoints,
               std::function<void()> completion_callback,
               std::function<void(size_t, double)> arrival_estimator_callback =
                   nullptr) = 0;

  /**
   * Cancel ongoing navigation
   */
  virtual void cancel_navigation() = 0;

  /**
   * Check if robot is currently navigating
   * @return true if navigation is in progress
   */
  virtual bool is_navigating() const = 0;

  /**
   * Get current goal ID (if any)
   * @return The UUID of the current navigation goal, or empty optional if no
   * goal active
   */
  virtual std::optional<std::array<uint8_t, 16>>
  get_current_goal_id() const = 0;

protected:
  std::function<void()> on_navigation_failed_callback_;

  /**
   * Set callback to be invoked when navigation fails
   * This is called by implementations when the controller reports failure,
   * allowing the fleet adapter to request replanning from RMF
   */
  void set_navigation_failed_callback(std::function<void()> callback) {
    on_navigation_failed_callback_ = callback;
  }

  friend class Ros2RobotAdapter; // Allow setting callback
};

} // namespace egb_fleet

#endif // EGB_FLEET__NAVIGATION_INTERFACE_HPP_
