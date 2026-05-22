// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#ifndef EGB_FLEET__ACTION__ROBOT_ACTION_HPP_
#define EGB_FLEET__ACTION__ROBOT_ACTION_HPP_

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "egb_fleet/action/robot_action_context.hpp"

namespace egb_fleet {
namespace action {

/**
 * @brief Action state enumeration
 */
enum class RobotActionState {
  IN_PROGRESS, ///< Action is currently executing
  CANCELING,   ///< Action cancellation has been requested
  CANCELED,    ///< Action was successfully canceled
  COMPLETED,   ///< Action completed successfully
  FAILED       ///< Action failed
};

/**
 * @brief Base class for robot actions
 *
 * This class provides the interface for executing and monitoring custom actions
 * on robots. Implementations should override update_action() to provide
 * action-specific logic.
 */
class RobotAction {
public:
  using ActivityIdentifier =
      rmf_fleet_adapter::agv::RobotUpdateHandle::ActivityIdentifier;

  /**
   * @brief Constructor
   * @param context Shared action context with robot state and communication
   * @param execution Activity identifier from RMF task system
   */
  RobotAction(std::shared_ptr<RobotActionContext> context,
              const ActivityIdentifier &execution);

  virtual ~RobotAction() = default;

  /**
   * @brief Update the action state
   *
   * This method is called periodically by the robot adapter to monitor
   * the progress and completion of the action. Implementations should
   * check the action status and return the current state.
   *
   * @return Current action state
   */
  virtual RobotActionState update_action() = 0;

  /**
   * @brief Cancel the current task associated with this action
   *
   * This is a utility method that can be used by action implementations
   * to cancel the ongoing RMF task.
   *
   * @param cancel_success Callback invoked if cancellation succeeds
   * @param cancel_fail Callback invoked if cancellation fails
   * @param label Optional label for the cancellation request
   */
  void cancel_task_of_action(std::function<void()> cancel_success,
                             std::function<void()> cancel_fail,
                             const std::string &label = "");

protected:
  std::shared_ptr<RobotActionContext> context_;
  std::string action_task_id_;
};

} // namespace action
} // namespace egb_fleet

#endif // EGB_FLEET__ACTION__ROBOT_ACTION_HPP_
