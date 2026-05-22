// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#ifndef EGB_FLEET__ROBOT_STATE_HPP_
#define EGB_FLEET__ROBOT_STATE_HPP_

#include "egb_fleet/action/robot_action_factory.hpp"
#include "egb_fleet/robot_handle.hpp"
#include "egb_fleet/tf_handler.hpp"
#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <pluginlib/class_loader.hpp>
#include <rmf_fleet_adapter/agv/Transformation.hpp>
#include <rmf_task/events/SimpleEventState.hpp>
#include <rmf_traffic/agv/Graph.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <yaml-cpp/yaml.h>

namespace egb_fleet {

class NavigationInterface;

/**
 * @brief Robot adapter
 *
 * Provides robot state information (pose, battery, map) for the fleet adapter.
 * Does NOT handle navigation - that's delegated to NavigationController.
 */
class RobotState : public RobotHandle {
public:
  /**
   * @brief Constructor
   * @param name Robot name
   * @param robot_config Robot-specific configuration
   * @param plugins_config Plugins configuration
   * @param node ROS2 node for communication
   * @param zenoh_session Zenoh session for cross-domain communication
   * @param fleet_handle Handle to the fleet for robot management
   * @param tf_buffer TF2 buffer for coordinate transforms
   * @param linear_velocity Maximum linear velocity
   */
  RobotState(const std::string &name, const YAML::Node &robot_config,
             const YAML::Node &plugins_config,
             std::shared_ptr<rclcpp::Node> node, void *zenoh_session,
             std::shared_ptr<FleetUpdateHandle> fleet_handle,
             std::shared_ptr<tf2_ros::Buffer> tf_buffer, double linear_velocity,
             std::shared_ptr<rmf_fleet_adapter::agv::RobotCommandHandle>
                 command_handle = nullptr);

  ~RobotState() override;

  double get_battery_soc() const override;
  std::string get_map_name() const override;
  std::optional<Eigen::Vector3d> get_pose() const override;

  void execute_action(const std::string &category,
                      const nlohmann::json &description,
                      rmf_fleet_adapter::agv::RobotUpdateHandle::ActionExecution
                          execution) override;

  bool update(std::shared_ptr<const rmf_traffic::agv::Graph> graph);

  void update_current_action();

  // Additional methods needed by robot_discovery
  void set_graph_and_transforms(
      std::shared_ptr<const rmf_traffic::agv::Graph> graph,
      const std::map<std::string, rmf_fleet_adapter::agv::Transformation>
          &transforms);

  // Set command handle after creation (for dynamic discovery)
  void
  set_command_handle(std::shared_ptr<rmf_fleet_adapter::agv::RobotCommandHandle>
                         command_handle);

  // Set navigation interface for action execution (enables undocking actions)
  void set_navigation_interface(
      std::shared_ptr<NavigationInterface> navigation_interface);

  // Get or create action context for executing actions (used by PathHandle for
  // charging)
  std::shared_ptr<action::RobotActionContext> get_or_create_action_context();

private:
  // Configuration
  YAML::Node robot_config_;
  YAML::Node plugins_config_;
  void *zenoh_session_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  double linear_velocity_;
  std::shared_ptr<rmf_fleet_adapter::agv::RobotCommandHandle> command_handle_;

  // TF Handler for pose lookups
  std::unique_ptr<TfHandler> tf_handler_;

  // Frame names and map
  std::string map_frame_;
  std::string robot_frame_;
  std::string map_name_;

  // Battery state
  std::atomic<double> battery_soc_{1.0};

  // Graph and transforms
  std::shared_ptr<const rmf_traffic::agv::Graph> graph_;
  std::map<std::string, rmf_fleet_adapter::agv::Transformation>
      coordinate_transforms_;

  // Action system with plugin loading
  std::map<std::string, std::string> action_to_plugin_name_;
  std::unique_ptr<pluginlib::ClassLoader<action::RobotActionFactory>>
      action_loader_;
  std::map<std::string, std::shared_ptr<action::RobotActionFactory>>
      loaded_factories_;
  std::shared_ptr<action::RobotActionContext> action_context_;
  std::shared_ptr<NavigationInterface> navigation_interface_;

  // Guards current_action_ and current_action_execution_ which are written by
  // the RMF worker thread (execute_action) and read by the timer thread
  // (update_current_action).
  std::mutex action_mutex_;

  // Current action for state machine polling
  std::shared_ptr<action::RobotAction> current_action_;

  // Current ActionExecution for RMF task phase status reporting
  std::optional<rmf_fleet_adapter::agv::RobotUpdateHandle::ActionExecution>
      current_action_execution_;

  // Battery state subscriber (Zenoh)
  void *battery_state_subscriber_{nullptr};

  // Battery state callback
  void battery_state_callback(const std::vector<uint8_t> &payload);
};

} // namespace egb_fleet

#endif // EGB_FLEET__ROBOT_STATE_HPP_
