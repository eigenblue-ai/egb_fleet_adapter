// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#ifndef EGB_FLEET__PLUGINS__UNDOCK_ACTION_HPP_
#define EGB_FLEET__PLUGINS__UNDOCK_ACTION_HPP_

#include "egb_fleet/action/robot_action_factory.hpp"

namespace egb_fleet {
namespace plugins {

/**
 * @brief Factory for creating undocking actions
 *
 * The undock action generates a backward path from the robot's current position
 * using the traffic graph. It then executes this path using the navigation
 * system.
 */
class UndockActionFactory : public action::RobotActionFactory {
public:
  UndockActionFactory() = default; // Required for pluginlib
  explicit UndockActionFactory(
      std::shared_ptr<action::RobotActionContext> context);
  void initialize(std::shared_ptr<action::RobotActionContext> context) override;
  bool supports_action(const std::string &category) override;
  std::shared_ptr<action::RobotAction>
  perform_action(const std::string &category, const nlohmann::json &description,
                 const ActivityIdentifier &execution) override;
};

} // namespace plugins
} // namespace egb_fleet

#endif // EGB_FLEET__PLUGINS__UNDOCK_ACTION_HPP_
