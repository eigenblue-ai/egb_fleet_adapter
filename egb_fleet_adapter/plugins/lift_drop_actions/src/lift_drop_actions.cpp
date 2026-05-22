// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#include "egb_fleet/plugins/lift_drop_actions.hpp"
#include "egb_fleet/utils/zenoh_helpers.hpp"
#include <chrono>
#include <mutex>
#include <pluginlib/class_list_macros.hpp>

namespace egb_fleet {
namespace plugins {

// Action implementation
class LiftDropAction : public action::RobotAction {
public:
  enum class State {
    IDLE,        // Initial state, need to send goal
    SENDING,     // Goal sent, waiting for acceptance
    IN_PROGRESS, // Action is being executed
    COMPLETED,   // Action completed successfully
    FAILED       // Action failed
  };

  LiftDropAction(std::shared_ptr<action::RobotActionContext> context,
                 const std::string &category, const nlohmann::json &description,
                 const ActivityIdentifier &execution)
      : RobotAction(context, execution), robot_name_(context_->robot_name),
        category_(category), state_(State::IDLE),
        creation_time_(std::chrono::steady_clock::now()), send_attempts_(0) {
    // Extract parameters from description JSON
    auto desc = description.contains("description") ? description["description"]
                                                    : description;
    timeout_ = desc.value("timeout", 30.0);
    car_location_ = desc.value("car_location", "");
    car_name_ = desc.value("car_name", "");

    // Generate goal ID for this action
    goal_id_ = utils::generate_goal_id();

    RCLCPP_INFO(context_->node->get_logger(),
                "[%s] LiftDropAction created: %s (car: %s, location: %s, "
                "goal_id from uuid)",
                robot_name_.c_str(), category_.c_str(), car_name_.c_str(),
                car_location_.c_str());

    // Track attached car for dropoff
    if (category_ == "pickup") {
      if (!car_name_.empty()) {
        store_attached_car(robot_name_, car_name_);
      }
    } else if (category_ == "dropoff") {
      // Retrieve the car that was picked up
      car_name_ = get_attached_car(robot_name_);
      RCLCPP_INFO(context_->node->get_logger(),
                  "[%s] Retrieved tracked car '%s' for dropoff",
                  robot_name_.c_str(), car_name_.c_str());
    }
  }

  action::RobotActionState update_action() override {
    auto now = std::chrono::steady_clock::now();
    auto elapsed =
        std::chrono::duration_cast<std::chrono::seconds>(now - creation_time_)
            .count();

    // Check for overall timeout
    if (elapsed > timeout_) {
      RCLCPP_ERROR(context_->node->get_logger(),
                   "[%s] LiftDropAction timed out after %.0f seconds",
                   robot_name_.c_str(), timeout_);
      state_ = State::FAILED;
      return action::RobotActionState::FAILED;
    }

    switch (state_) {
    case State::IDLE:
      return handle_idle();

    case State::SENDING:
      return handle_sending();

    case State::IN_PROGRESS:
      return handle_in_progress();

    case State::COMPLETED:
      return action::RobotActionState::COMPLETED;

    case State::FAILED:
      return action::RobotActionState::FAILED;
    }

    return action::RobotActionState::IN_PROGRESS;
  }

private:
  std::string category_;
  std::string robot_name_;
  std::string car_location_;
  std::string car_name_;
  double timeout_;
  State state_;
  std::array<uint8_t, 16> goal_id_;
  std::chrono::steady_clock::time_point creation_time_;
  int send_attempts_;
  static constexpr int MAX_SEND_ATTEMPTS = 3;

  // Static map to track which car each robot has attached.
  // Guarded by robot_to_car_mutex_ — written from RMF worker thread
  // (constructor), read/erased from timer thread (update_action).
  static std::mutex robot_to_car_mutex_;
  static std::map<std::string, std::string> robot_to_car_;

  action::RobotActionState handle_idle() {
    // Determine operation: 1 = LIFT_UP (pickup), 0 = LIFT_DOWN (dropoff)
    int8_t operation = (category_ == "pickup") ? 1 : 0;

    bool accepted = utils::send_lift_drop_goal(
        context_->zenoh_session, robot_name_, goal_id_, operation,
        car_location_, car_name_, timeout_);

    if (accepted) {
      RCLCPP_INFO(
          context_->node->get_logger(),
          "[%s] %s action goal accepted (goal_id sent), monitoring status",
          robot_name_.c_str(), category_.c_str());
      state_ = State::IN_PROGRESS;
      return action::RobotActionState::IN_PROGRESS;
    } else {
      send_attempts_++;
      if (send_attempts_ >= MAX_SEND_ATTEMPTS) {
        RCLCPP_ERROR(context_->node->get_logger(),
                     "[%s] %s action failed: action server rejected goal after "
                     "%d attempts",
                     robot_name_.c_str(), category_.c_str(), send_attempts_);
        state_ = State::FAILED;
        return action::RobotActionState::FAILED;
      }
      RCLCPP_WARN(context_->node->get_logger(),
                  "[%s] %s action goal rejected, will retry (%d/%d)",
                  robot_name_.c_str(), category_.c_str(), send_attempts_,
                  MAX_SEND_ATTEMPTS);
      // Retry on next update
      return action::RobotActionState::IN_PROGRESS;
    }
  }

  action::RobotActionState handle_sending() {
    // This is a transient state, should not reach here in normal flow
    return action::RobotActionState::IN_PROGRESS;
  }

  action::RobotActionState handle_in_progress() {
    // Poll for result with short timeout
    int result = utils::get_lift_drop_result(context_->zenoh_session,
                                             robot_name_, goal_id_,
                                             100); // 100ms timeout per poll

    // 0 = done, <0 = failed; ACTION_RESULT_PENDING (>0) just keeps polling.
    if (result == 0) {
      // Success
      RCLCPP_INFO(context_->node->get_logger(),
                  "[%s] %s action completed successfully", robot_name_.c_str(),
                  category_.c_str());

      // Handle car tracking for dropoff
      if (category_ == "dropoff") {
        clear_attached_car(robot_name_);
      }

      state_ = State::COMPLETED;
      return action::RobotActionState::COMPLETED;
    } else if (result < 0) {
      // Error
      RCLCPP_ERROR(context_->node->get_logger(),
                   "[%s] %s action failed with error code %d",
                   robot_name_.c_str(), category_.c_str(), result);
      state_ = State::FAILED;
      return action::RobotActionState::FAILED;
    }

    // Still executing
    return action::RobotActionState::IN_PROGRESS;
  }

  static void store_attached_car(const std::string &robot,
                                 const std::string &car) {
    std::lock_guard<std::mutex> lock(robot_to_car_mutex_);
    robot_to_car_[robot] = car;
  }

  static std::string get_attached_car(const std::string &robot) {
    std::lock_guard<std::mutex> lock(robot_to_car_mutex_);
    auto it = robot_to_car_.find(robot);
    if (it != robot_to_car_.end()) {
      return it->second;
    }
    return "";
  }

  static void clear_attached_car(const std::string &robot) {
    std::lock_guard<std::mutex> lock(robot_to_car_mutex_);
    robot_to_car_.erase(robot);
  }
};

// Static member initialization
std::mutex LiftDropAction::robot_to_car_mutex_;
std::map<std::string, std::string> LiftDropAction::robot_to_car_;

// Factory implementation
LiftDropActionFactory::LiftDropActionFactory(
    std::shared_ptr<action::RobotActionContext> context)
    : RobotActionFactory(context) {
  actions_ = {"pickup", "dropoff"};
}

void LiftDropActionFactory::initialize(
    std::shared_ptr<action::RobotActionContext> context) {
  RobotActionFactory::initialize(context);
  actions_ = {"pickup", "dropoff"};
}

bool LiftDropActionFactory::supports_action(const std::string &category) {
  return category == "pickup" || category == "dropoff";
}

std::shared_ptr<action::RobotAction>
LiftDropActionFactory::perform_action(const std::string &category,
                                      const nlohmann::json &description,
                                      const ActivityIdentifier &execution) {

  if (!supports_action(category)) {
    return nullptr;
  }

  return std::make_shared<LiftDropAction>(context_, category, description,
                                          execution);
}

} // namespace plugins
} // namespace egb_fleet

// Export plugin
PLUGINLIB_EXPORT_CLASS(egb_fleet::plugins::LiftDropActionFactory,
                       egb_fleet::action::RobotActionFactory)
