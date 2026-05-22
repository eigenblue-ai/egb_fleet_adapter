// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#ifndef EGB_FLEET__ROBOT_HANDLE_HPP_
#define EGB_FLEET__ROBOT_HANDLE_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmf_fleet_adapter/agv/FleetUpdateHandle.hpp>
#include <rmf_fleet_adapter/agv/RobotUpdateHandle.hpp>
#include <rmf_task/events/SimpleEventState.hpp>

namespace egb_fleet {

/// Abstract base class for robot adapters integrating with RMF.
class RobotHandle {
public:
  using FleetUpdateHandle = rmf_fleet_adapter::agv::FleetUpdateHandle;
  using RobotUpdateHandle = rmf_fleet_adapter::agv::RobotUpdateHandle;
  using ActivityIdentifier =
      rmf_fleet_adapter::agv::RobotUpdateHandle::ActivityIdentifier;

  /**
   * @brief Constructor
   * @param name Robot name
   * @param node ROS2 node for communication
   * @param fleet_handle Handle to the fleet for robot management
   */
  RobotHandle(const std::string &name, std::shared_ptr<rclcpp::Node> node,
              std::shared_ptr<FleetUpdateHandle> fleet_handle);

  virtual ~RobotHandle() = default;

  /**
   * @brief Get the robot's battery state of charge
   * @return Battery SOC as a float between 0.0 and 1.0
   */
  virtual double get_battery_soc() const = 0;

  /**
   * @brief Get the name of the map the robot is currently localized on
   * @return Map name string
   */
  virtual std::string get_map_name() const = 0;

  /**
   * @brief Get the robot's current 2D pose
   * @return Optional 3D vector [x, y, yaw] in meters and radians, or nullopt if
   * unavailable
   */
  virtual std::optional<Eigen::Vector3d> get_pose() const = 0;

  /**
   * @brief Execute a custom action
   * @param category Action category (e.g., "pickup", "dropoff")
   * @param description Action description as JSON object
   * @param execution ActionExecution object for status reporting and completion
   * callbacks
   */
  virtual void execute_action(
      const std::string &category, const nlohmann::json &description,
      rmf_fleet_adapter::agv::RobotUpdateHandle::ActionExecution execution) = 0;

  /**
   * @brief Set the RMF update handle for this robot
   * @param handle Shared pointer to the robot update handle
   * @note This is managed by the FullControlHandle and shouldn't be called
   * directly
   */
  void set_update_handle(std::shared_ptr<RobotUpdateHandle> handle);
  void set_fleet_handle(std::shared_ptr<FleetUpdateHandle> handle);

  /**
   * @brief Get the robot name
   * @return Robot name string
   */
  const std::string &name() const { return name_; }

  /**
   * @brief Get the ROS2 node
   * @return Shared pointer to the node
   */
  std::shared_ptr<rclcpp::Node> node() { return node_; }

  /**
   * @brief Get the fleet update handle
   * @return Shared pointer to the fleet handle
   */
  std::shared_ptr<FleetUpdateHandle> fleet_handle() { return fleet_handle_; }

  /**
   * @brief Get the robot update handle
   * @return Shared pointer to the update handle, or nullptr if not set
   */
  std::shared_ptr<RobotUpdateHandle> update_handle() { return update_handle_; }

protected:
  std::string name_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<FleetUpdateHandle> fleet_handle_;
  std::shared_ptr<RobotUpdateHandle> update_handle_;
};

} // namespace egb_fleet

#endif // EGB_FLEET__ROBOT_HANDLE_HPP_
