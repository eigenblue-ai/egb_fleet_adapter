// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#include "egb_fleet/action/robot_action_factory.hpp"

namespace egb_fleet {
namespace action {

RobotActionFactory::RobotActionFactory(
    std::shared_ptr<RobotActionContext> context)
    : context_(context) {
  /* Python reference (action.py:102-111):

  def __init__(self, context: RobotActionContext):
      self.context = context

      if 'actions' not in context.action_config:
          raise KeyError(
              'List of supported actions is not provided in the action '
              'config! Unable to instantiate an ActionFactory.')
      self.actions = context.action_config['actions']
  */

  if (context_->action_config["actions"]) {
    for (const auto &action : context_->action_config["actions"]) {
      actions_.push_back(action.as<std::string>());
    }
  } else {
    throw std::runtime_error(
        "List of supported actions is not provided in the action config! "
        "Unable to instantiate an ActionFactory.");
  }
}

} // namespace action
} // namespace egb_fleet
