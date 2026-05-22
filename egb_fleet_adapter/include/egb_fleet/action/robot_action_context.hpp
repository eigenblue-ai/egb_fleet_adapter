// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#ifndef EGB_FLEET__ACTION__ROBOT_ACTION_CONTEXT_HPP_
#define EGB_FLEET__ACTION__ROBOT_ACTION_CONTEXT_HPP_

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <rmf_fleet_adapter/agv/RobotUpdateHandle.hpp>
#include <rmf_fleet_adapter/agv/Transformation.hpp>
#include <rmf_traffic/agv/Graph.hpp>
#include <yaml-cpp/yaml.h>

namespace egb_fleet {
namespace action {

/**
 * @brief Context object providing robot state and communication access to
 * actions
 *
 * This struct encapsulates all the information and capabilities that an action
 * needs to interact with the robot and the RMF system. It provides callbacks
 * for querying robot state and access to ROS2 and Zenoh communication channels.
 */
struct RobotActionContext {
  using RobotUpdateHandle = rmf_fleet_adapter::agv::RobotUpdateHandle;

  /// ROS2 node for logging and communication
  std::shared_ptr<rclcpp::Node> node;

  /// Name of the robot
  std::string robot_name;

  /// RMF update handle for reporting robot state and progress
  std::shared_ptr<RobotUpdateHandle> update_handle;

  /// Action-specific configuration from YAML
  YAML::Node action_config;

  /// Callback to get the robot's battery state of charge (0.0-1.0)
  std::function<double()> get_battery_soc;

  /// Callback to get the name of the map the robot is on
  std::function<std::string()> get_map_name;

  /// Callback to get the robot's current pose [x, y, yaw]
  std::function<std::optional<Eigen::Vector3d>()> get_pose;

  /// Pointer to Zenoh session for communication (void* to avoid dependency
  /// here) Cast to zenoh::Session* when needed
  void *zenoh_session;

  /// Navigation graph for path planning and undocking operations
  std::shared_ptr<const rmf_traffic::agv::Graph> traffic_graph;

  /// Coordinate transformations between RMF and robot coordinate systems (map
  /// -> transform)
  std::shared_ptr<
      const std::map<std::string, rmf_fleet_adapter::agv::Transformation>>
      coordinate_transforms;

  /// Callback to execute multi-waypoint navigation
  /// Parameters: vector of poses, completion callback
  std::function<void(const std::vector<geometry_msgs::msg::PoseStamped> &,
                     std::function<void()>)>
      execute_navigation;

  /// Callback to cancel ongoing navigation
  std::function<void()> cancel_navigation;

  /**
   * @brief Constructor with all required parameters
   */
  RobotActionContext(
      std::shared_ptr<rclcpp::Node> node_, const std::string &robot_name_,
      std::shared_ptr<RobotUpdateHandle> update_handle_,
      const YAML::Node &action_config_,
      std::function<double()> get_battery_soc_,
      std::function<std::string()> get_map_name_,
      std::function<std::optional<Eigen::Vector3d>()> get_pose_,
      void *zenoh_session_ = nullptr,
      std::shared_ptr<const rmf_traffic::agv::Graph> traffic_graph_ = nullptr,
      std::shared_ptr<
          const std::map<std::string, rmf_fleet_adapter::agv::Transformation>>
          coordinate_transforms_ = nullptr,
      std::function<void(const std::vector<geometry_msgs::msg::PoseStamped> &,
                         std::function<void()>)>
          execute_navigation_ = nullptr,
      std::function<void()> cancel_navigation_ = nullptr)
      : node(node_), robot_name(robot_name_), update_handle(update_handle_),
        action_config(action_config_), get_battery_soc(get_battery_soc_),
        get_map_name(get_map_name_), get_pose(get_pose_),
        zenoh_session(zenoh_session_), traffic_graph(traffic_graph_),
        coordinate_transforms(coordinate_transforms_),
        execute_navigation(execute_navigation_),
        cancel_navigation(cancel_navigation_) {}
};

} // namespace action
} // namespace egb_fleet

#endif // EGB_FLEET__ACTION__ROBOT_ACTION_CONTEXT_HPP_
