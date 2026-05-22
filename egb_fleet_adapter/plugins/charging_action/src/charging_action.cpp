// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#include "egb_fleet/plugins/charging_action.hpp"
#include "egb_fleet/utils/zenoh_helpers.hpp"
#include <array>
#include <chrono>
#include <memory>
#include <pluginlib/class_list_macros.hpp>

namespace egb_fleet {
namespace plugins {

/**
 * @brief Charging action implementation for battery charging at charger
 * waypoints
 *
 * This action manages the charging process when the robot docks at a charger.
 * It communicates with the robot's charging action server via Zenoh to:
 * - Send charging start request
 * - Monitor charging progress via battery SOC feedback
 * - Complete when battery reaches target SOC (100%)
 */
class ChargingAction : public action::RobotAction {
public:
  enum class State {
    REQUESTING, // Sending charging action request to robot
    EXECUTING,  // Monitoring charging progress
    COMPLETED,  // Charging completed successfully
    FAILED      // Action failed or timed out
  };

  ChargingAction(std::shared_ptr<action::RobotActionContext> context,
                 const nlohmann::json &description,
                 const ActivityIdentifier &execution)
      : RobotAction(context, execution), state_(State::REQUESTING),
        robot_name_(context_->robot_name),
        request_time_(std::chrono::steady_clock::now()),
        max_charging_time_(60) // 60 minute timeout
  {
    // Extract target SOC from description or use default (1.0 = 100%)
    if (description.contains("target_soc")) {
      target_soc_ = description["target_soc"].get<double>();
    } else {
      target_soc_ = 1.0; // Default to full charge
    }

    // Extract dock name for logging
    if (description.contains("dock_name")) {
      dock_name_ = description["dock_name"].get<std::string>();
    }

    RCLCPP_INFO(context_->node->get_logger(),
                "[%s] ChargingAction created for dock: %s (target SOC: %.1f%%)",
                robot_name_.c_str(), dock_name_.c_str(), target_soc_ * 100.0);
  }

  action::RobotActionState update_action() override {
    switch (state_) {
    case State::REQUESTING:
      return handle_requesting();

    case State::EXECUTING:
      return handle_executing();

    case State::COMPLETED:
      return action::RobotActionState::COMPLETED;

    case State::FAILED:
      return action::RobotActionState::FAILED;
    }

    return action::RobotActionState::IN_PROGRESS;
  }

private:
  State state_;
  std::string robot_name_;
  std::string dock_name_;
  std::array<uint8_t, 16> goal_id_;
  double target_soc_ = 1.0;
  std::chrono::steady_clock::time_point request_time_;
  std::chrono::minutes max_charging_time_;
  int feedback_request_count_ = 0; // For throttling logs

  // Send charging action request via Zenoh
  bool send_action_request() {
    if (!context_->zenoh_session) {
      RCLCPP_ERROR(context_->node->get_logger(),
                   "[%s] Zenoh session not available for charging request",
                   robot_name_.c_str());
      return false;
    }

    try {
      // Generate goal ID using helper function
      goal_id_ = utils::generate_goal_id();

      // Send charging goal via Zenoh using helper function
      bool accepted = utils::send_charge_battery_goal(
          context_->zenoh_session, robot_name_, goal_id_, target_soc_,
          60.0, // timeout in seconds (60 minutes max)
          5000  // Zenoh timeout in milliseconds
      );

      RCLCPP_INFO(context_->node->get_logger(),
                  "[%s] Sending charging action request (target_soc: %.1f%%, "
                  "accepted: %s)",
                  robot_name_.c_str(), target_soc_ * 100.0,
                  accepted ? "true" : "false");

      return accepted;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(context_->node->get_logger(),
                   "[%s] Failed to send charging action request: %s",
                   robot_name_.c_str(), e.what());
      return false;
    }
  }

  // Poll action result from robot via Zenoh
  int get_action_result() {
    if (!context_->zenoh_session) {
      return -1; // Error
    }

    try {
      // Get result via Zenoh using helper function
      int result = utils::get_charge_battery_result(
          context_->zenoh_session, robot_name_, goal_id_,
          100 // 100ms timeout per poll
      );

      // Result codes:
      // 0 = STATUS_SUCCEEDED (goal was executed successfully)
      // negative = error
      // positive = still executing or unknown state
      return result;
    } catch (const std::exception &e) {
      RCLCPP_WARN(context_->node->get_logger(),
                  "[%s] Failed to get charging result: %s", robot_name_.c_str(),
                  e.what());
      return -1;
    }
  }

  action::RobotActionState handle_requesting() {
    // Send charging action request to robot
    if (!send_action_request()) {
      RCLCPP_ERROR(context_->node->get_logger(),
                   "[%s] Failed to send charging action request",
                   robot_name_.c_str());
      state_ = State::FAILED;
      return action::RobotActionState::FAILED;
    }

    request_time_ = std::chrono::steady_clock::now();
    state_ = State::EXECUTING;
    RCLCPP_INFO(context_->node->get_logger(),
                "[%s] Charging started at dock: %s", robot_name_.c_str(),
                dock_name_.c_str());

    return action::RobotActionState::IN_PROGRESS;
  }

  action::RobotActionState handle_executing() {
    // Check timeout
    auto elapsed = std::chrono::steady_clock::now() - request_time_;
    if (elapsed > max_charging_time_) {
      RCLCPP_ERROR(
          context_->node->get_logger(),
          "[%s] Charging action timeout - no progress for %lld minutes",
          robot_name_.c_str(),
          static_cast<long long>(max_charging_time_.count()));
      state_ = State::FAILED;
      return action::RobotActionState::FAILED;
    }

    // Poll result from robot action server via Zenoh
    int result = get_action_result();

    // Result codes:
    // 0 = STATUS_SUCCEEDED (charging completed successfully)
    // negative = error
    // positive = still executing or unknown state

    if (result == 0) {
      // Success
      state_ = State::COMPLETED;
      RCLCPP_INFO(context_->node->get_logger(),
                  "[%s] Charging completed successfully at dock: %s (target "
                  "SOC: %.1f%%)",
                  robot_name_.c_str(), dock_name_.c_str(), target_soc_ * 100.0);
      return action::RobotActionState::COMPLETED;
    } else if (result < 0) {
      // Error
      state_ = State::FAILED;
      RCLCPP_ERROR(context_->node->get_logger(),
                   "[%s] Charging failed at dock: %s (error code: %d)",
                   robot_name_.c_str(), dock_name_.c_str(), result);
      return action::RobotActionState::FAILED;
    }

    // Log progress periodically (throttle to every 10 requests)
    if (feedback_request_count_ % 10 == 0) {
      RCLCPP_INFO(context_->node->get_logger(),
                  "[%s] Charging in progress at dock: %s (target SOC: %.1f%%)",
                  robot_name_.c_str(), dock_name_.c_str(), target_soc_ * 100.0);
    }
    feedback_request_count_++;

    // Continue charging
    return action::RobotActionState::IN_PROGRESS;
  }
};

// Factory implementation
ChargingActionFactory::ChargingActionFactory(
    std::shared_ptr<action::RobotActionContext> context)
    : RobotActionFactory(context) {}

void ChargingActionFactory::initialize(
    std::shared_ptr<action::RobotActionContext> context) {
  RobotActionFactory::initialize(context);
}

bool ChargingActionFactory::supports_action(const std::string &category) {
  return category == "charge";
}

std::shared_ptr<action::RobotAction>
ChargingActionFactory::perform_action(const std::string &category,
                                      const nlohmann::json &description,
                                      const ActivityIdentifier &execution) {
  if (category != "charge") {
    return nullptr;
  }

  return std::make_shared<ChargingAction>(context_, description, execution);
}

} // namespace plugins
} // namespace egb_fleet

// Register the plugin
PLUGINLIB_EXPORT_CLASS(egb_fleet::plugins::ChargingActionFactory,
                       egb_fleet::action::RobotActionFactory)
