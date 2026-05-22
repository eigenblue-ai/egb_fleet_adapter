// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#ifndef EGB_FLEET__PLUGINS__CHARGING_ACTION_HPP_
#define EGB_FLEET__PLUGINS__CHARGING_ACTION_HPP_

#include "egb_fleet/action/robot_action_factory.hpp"

namespace egb_fleet {
namespace plugins {

/**
 * @brief Factory for creating charging actions
 *
 * The charging action handles automatic battery charging when the robot docks
 * at a charger waypoint. It communicates with the robot's charging action
 * server via Zenoh to monitor and control the charging process.
 */
class ChargingActionFactory : public action::RobotActionFactory {
public:
  ChargingActionFactory() = default; // Required for pluginlib
  explicit ChargingActionFactory(
      std::shared_ptr<action::RobotActionContext> context);
  void initialize(std::shared_ptr<action::RobotActionContext> context) override;
  bool supports_action(const std::string &category) override;
  std::shared_ptr<action::RobotAction>
  perform_action(const std::string &category, const nlohmann::json &description,
                 const ActivityIdentifier &execution) override;
};

} // namespace plugins
} // namespace egb_fleet

#endif // EGB_FLEET__PLUGINS__CHARGING_ACTION_HPP_
