// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors
// Mirrors the upstream Python lift_drop_actions implementation (358 lines)

#ifndef EGB_FLEET__PLUGINS__LIFT_DROP_ACTIONS_HPP_
#define EGB_FLEET__PLUGINS__LIFT_DROP_ACTIONS_HPP_

#include "egb_fleet/action/robot_action_factory.hpp"

namespace egb_fleet {
namespace plugins {

class LiftDropActionFactory : public action::RobotActionFactory {
public:
  LiftDropActionFactory() = default; // Required for pluginlib
  explicit LiftDropActionFactory(
      std::shared_ptr<action::RobotActionContext> context);
  void initialize(std::shared_ptr<action::RobotActionContext> context) override;
  bool supports_action(const std::string &category) override;
  std::shared_ptr<action::RobotAction>
  perform_action(const std::string &category, const nlohmann::json &description,
                 const ActivityIdentifier &execution) override;
};

} // namespace plugins
} // namespace egb_fleet

#endif // EGB_FLEET__PLUGINS__LIFT_DROP_ACTIONS_HPP_
