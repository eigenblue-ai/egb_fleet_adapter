// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#ifndef EGB_FLEET__FULL_CONTROL_HANDLE_HPP_
#define EGB_FLEET__FULL_CONTROL_HANDLE_HPP_

#include <Eigen/Core>
#include <atomic>
#include <egb_fleet_msgs/msg/waypoint.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rmf_fleet_adapter/agv/RobotCommandHandle.hpp>
#include <rmf_fleet_adapter/agv/Transformation.hpp>
#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/agv/Planner.hpp>

namespace egb_fleet {

class RobotHandle;
class NavigationInterface;
struct NavigationSession;
namespace action {
class RobotActionContext;
class RobotActionFactory;
} // namespace action

class PathHandle : public rmf_fleet_adapter::agv::RobotCommandHandle {
public:
  PathHandle(const std::string &robot_name,
             std::shared_ptr<NavigationInterface> control_adapter,
             std::shared_ptr<rclcpp::Node> node,
             const std::map<std::string, rmf_fleet_adapter::agv::Transformation>
                 &coordinate_transforms,
             std::shared_ptr<const rmf_traffic::agv::Graph> graph);

  void follow_new_path(
      const std::vector<rmf_traffic::agv::Plan::Waypoint> &waypoints,
      ArrivalEstimator next_arrival_estimator,
      RequestCompleted path_finished_callback) override;

  void stop() override;

  void dock(const std::string &dock_name,
            RequestCompleted docking_finished_callback) override;

  // Update position and nearby lanes (mirrors RobotUpdateHandle signature)
  void update_position(const Eigen::Vector3d &position,
                       const std::vector<std::size_t> &lanes);

  // Update position when robot is on a waypoint (for docking)
  void update_position(std::size_t waypoint_index, double orientation);

  // Navigation lane info from current plan (for lane-aware position updates)
  struct NavigationLaneInfo {
    std::vector<std::size_t> lanes;
    std::optional<std::size_t> graph_index;
  };
  std::optional<NavigationLaneInfo> get_navigation_lanes() const;

  /// Get the current active session (may be null). Used by
  /// NavigationController.
  std::shared_ptr<NavigationSession> current_session() const;

  /// Drain buffered Zenoh feedback and forward to RMF arrival estimator.
  /// MUST be called from the timer/update thread (same executor as RMF).
  void drain_pending_feedback();

private:
  std::string robot_name_;
  std::shared_ptr<NavigationInterface> control_adapter_;
  std::shared_ptr<rclcpp::Node> node_;
  std::map<std::string, rmf_fleet_adapter::agv::Transformation>
      coordinate_transforms_;
  std::shared_ptr<const rmf_traffic::agv::Graph> graph_;

  // Cached robot position and nearby lanes for docking operations
  // Protected by position_mutex_ (written by update timer, read by dock on RMF
  // thread)
  mutable std::mutex position_mutex_;
  Eigen::Vector3d current_position_;
  std::vector<std::size_t> nearby_lanes_;

  // The active navigation session. Atomically swapped on each follow_new_path.
  // Protected by session_mutex_ for the pointer swap; session internals have
  // their own mutex.
  mutable std::mutex session_mutex_;
  std::shared_ptr<NavigationSession> session_;

  // Apply coordinate transformation to a 2D position
  Eigen::Vector2d transform_position(const Eigen::Vector2d &rmf_position) const;

  // Convert a single waypoint directly to waypoints (with coordinate transform)
  egb_fleet_msgs::msg::Waypoint
  waypoint_to_msg(const rmf_traffic::agv::Plan::Waypoint &waypoint,
                  size_t waypoint_index, size_t total_waypoints) const;

  // Find the best approach lane to a dock
  std::optional<size_t>
  find_best_approach_lane(const rmf_traffic::agv::Graph::Lane &docking_lane,
                          size_t dock_waypoint_index) const;
};

} // namespace egb_fleet

#endif // EGB_FLEET__FULL_CONTROL_HANDLE_HPP_
