// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#ifndef EGB_FLEET__ACTION__ROBOT_ACTION_FACTORY_HPP_
#define EGB_FLEET__ACTION__ROBOT_ACTION_FACTORY_HPP_

#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <rmf_task/events/SimpleEventState.hpp>

#include "egb_fleet/action/robot_action.hpp"
#include "egb_fleet/action/robot_action_context.hpp"

namespace egb_fleet {
namespace action {

/**
 * @brief Factory base class for creating robot actions
 *
 * Action plugins should inherit from this class and implement the
 * supports_action() and perform_action() methods. The factory is
 * responsible for creating action instances based on the action category.
 */
class RobotActionFactory {
public:
  using ActivityIdentifier =
      rmf_fleet_adapter::agv::RobotUpdateHandle::ActivityIdentifier;

  /**
   * @brief Default constructor (required for pluginlib)
   */
  RobotActionFactory() = default;

  /**
   * @brief Constructor with context
   * @param context Shared action context
   */
  explicit RobotActionFactory(std::shared_ptr<RobotActionContext> context);

  virtual ~RobotActionFactory() = default;

  /**
   * @brief Initialize the factory with a context (for plugin usage)
   * @param context Shared action context
   */
  virtual void initialize(std::shared_ptr<RobotActionContext> context) {
    context_ = context;
  }

  /**
   * @brief Check if this factory supports a given action category
   * @param category Action category string (e.g., "pickup", "dropoff")
   * @return true if this factory can create actions of this category
   */
  virtual bool supports_action(const std::string &category) = 0;

  /**
   * @brief Create and begin execution of an action
   * @param category Action category
   * @param description Action description as JSON object
   * @param execution Activity identifier from RMF
   * @return Shared pointer to the created action
   */
  virtual std::shared_ptr<RobotAction>
  perform_action(const std::string &category, const nlohmann::json &description,
                 const ActivityIdentifier &execution) = 0;

protected:
  std::shared_ptr<RobotActionContext> context_;
  std::vector<std::string> actions_; ///< List of supported action categories
};

} // namespace action
} // namespace egb_fleet

#endif // EGB_FLEET__ACTION__ROBOT_ACTION_FACTORY_HPP_
