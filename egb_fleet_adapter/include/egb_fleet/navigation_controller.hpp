// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors
// Licensed under the Apache License, Version 2.0

#ifndef EGB_FLEET__NAVIGATION_CONTROLLER_HPP_
#define EGB_FLEET__NAVIGATION_CONTROLLER_HPP_

#include "egb_fleet/navigation_interface.hpp"
#include "egb_fleet/navigation_session.hpp"
#include <atomic>
#include <egb_fleet_msgs/action/follow_waypoints.hpp>
#include <egb_fleet_msgs/msg/lane.hpp>
#include <egb_fleet_msgs/msg/waypoint.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_ros/buffer.h>

namespace egb_fleet {

/**
 * Navigation control adapter for the local planner.
 *
 * Communicates with the the controller via Zenoh (FollowWaypoints action).
 * All per-plan state lives in NavigationSession — this class owns no mutable
 * navigation state itself, only a weak_ptr to the current session.
 *
 * Threading: Zenoh callbacks (feedback/status) run on Zenoh threads.
 * They acquire session->mutex to snapshot callbacks, then invoke outside lock.
 */
class NavigationController : public NavigationInterface {
public:
  NavigationController(const std::string &robot_name,
                       std::shared_ptr<rclcpp::Node> node, void *zenoh_session,
                       std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  ~NavigationController() override;

  void execute_path(const std::vector<egb_fleet_msgs::msg::Waypoint> &waypoints,
                    std::function<void()> completion_callback,
                    std::function<void(size_t, double)>
                        arrival_estimator_callback = nullptr) override;

  void cancel_navigation() override;
  bool is_navigating() const override;
  std::optional<std::array<uint8_t, 16>> get_current_goal_id() const override;

  /// Set the session to use. Called by PathHandle::follow_new_path before
  /// execute_path.
  void set_session(std::shared_ptr<NavigationSession> session);

private:
  bool send_follow_path_goal(
      const std::array<uint8_t, 16> &goal_id,
      const std::vector<egb_fleet_msgs::msg::Waypoint> &waypoints,
      const std::vector<egb_fleet_msgs::msg::Lane> &lanes,
      const std::string &order_id);

  std::vector<egb_fleet_msgs::msg::Lane>
  generate_lanes(size_t waypoint_count,
                 const std::vector<double> &speed_limits);
  void setup_zenoh_subscribers();
  void feedback_callback(const std::vector<uint8_t> &payload);
  void status_callback(const std::vector<uint8_t> &payload);

  std::string robot_name_;
  std::shared_ptr<rclcpp::Node> node_;
  void *zenoh_session_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // Weak reference to the current session (owned by PathHandle).
  // Protected by session_wp_mutex_ for the weak_ptr copy only.
  mutable std::mutex session_wp_mutex_;
  std::weak_ptr<NavigationSession> session_wp_;

  // Zenoh subscribers (opaque handles) - created once in constructor
  void *feedback_subscriber_{nullptr};
  void *status_subscriber_{nullptr};
};

} // namespace egb_fleet

#endif // EGB_FLEET__NAVIGATION_CONTROLLER_HPP_
