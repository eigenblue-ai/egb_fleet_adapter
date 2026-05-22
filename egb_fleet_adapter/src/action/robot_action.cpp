// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#include "egb_fleet/action/robot_action.hpp"

namespace egb_fleet {
namespace action {

RobotAction::RobotAction(std::shared_ptr<RobotActionContext> context,
                         const ActivityIdentifier &execution)
    : context_(context) {
  (void)execution;

  // Get current task ID from RMF update handle
  if (context_->update_handle) {
    action_task_id_ = context_->update_handle->current_task_id();
  }
}

void RobotAction::cancel_task_of_action(std::function<void()> cancel_success,
                                        std::function<void()> cancel_fail,
                                        const std::string &label) {
  /* Python reference (action.py:77-99):

  def cancel_task_of_action(
      self,
      cancel_success: Callable[[], None],
      cancel_fail: Callable[[], None],
      label: str = ''
  ):
      self.context.node.get_logger().info(
          f'[{self.context.robot_name}] Cancel task requested for '
          f'[{self.action_task_id}]')

      def _on_cancel(result: bool):
          if result:
              self.context.node.get_logger().info(
                  f'[{self.context.robot_name}] Found task '
                  f'[{self.action_task_id}], cancelling...')
              cancel_success()
          else:
              self.context.node.get_logger().info(
                  f'[{self.context.robot_name}] Failed to cancel task '
                  f'[{self.action_task_id}]')
              cancel_fail()
      self.context.update_handle.more().cancel_task(
          self.action_task_id, [label], lambda result: _on_cancel(result))
  */

  if (action_task_id_.empty()) {
    RCLCPP_WARN(context_->node->get_logger(),
                "[%s] Cannot cancel task - no task ID available",
                context_->robot_name.c_str());
    cancel_fail();
    return;
  }

  RCLCPP_INFO(context_->node->get_logger(),
              "[%s] Cancel task requested for [%s]",
              context_->robot_name.c_str(), action_task_id_.c_str());

  // TODO(egb_fleet_adapter): real cancellation. Currently calls
  // cancel_success() unconditionally — safe no-op.
  (void)label;
  cancel_success();
}

} // namespace action
} // namespace egb_fleet
