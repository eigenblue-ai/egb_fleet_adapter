// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors
// Licensed under the Apache License, Version 2.0

#include "egb_fleet/navigation_controller.hpp"
#include "egb_fleet/utils/zenoh_helpers.hpp"
#include <action_msgs/msg/goal_status.hpp>
#include <cstring>
#include <egb_fleet_msgs/action/follow_waypoints.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <zenoh.hxx>

namespace egb_fleet {

NavigationController::NavigationController(
    const std::string &robot_name, std::shared_ptr<rclcpp::Node> node,
    void *zenoh_session, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
    : robot_name_(robot_name), node_(node), zenoh_session_(zenoh_session),
      tf_buffer_(tf_buffer) {
  setup_zenoh_subscribers();
}

NavigationController::~NavigationController() {
  if (feedback_subscriber_) {
    auto *subscriber =
        static_cast<zenoh::Subscriber<void> *>(feedback_subscriber_);
    delete subscriber;
    feedback_subscriber_ = nullptr;
  }
  if (status_subscriber_) {
    auto *subscriber =
        static_cast<zenoh::Subscriber<void> *>(status_subscriber_);
    delete subscriber;
    status_subscriber_ = nullptr;
  }
}

void NavigationController::set_session(
    std::shared_ptr<NavigationSession> session) {
  std::lock_guard<std::mutex> lock(session_wp_mutex_);
  session_wp_ = session;
}

void NavigationController::execute_path(
    const std::vector<egb_fleet_msgs::msg::Waypoint> &waypoints,
    std::function<void()> completion_callback,
    std::function<void(size_t, double)> arrival_estimator_callback) {

  RCLCPP_INFO(
      node_->get_logger(),
      "[NavigationController] Executing path with %zu waypoints for robot: %s",
      waypoints.size(), robot_name_.c_str());

  std::vector<egb_fleet_msgs::msg::Lane> lanes =
      generate_lanes(waypoints.size());
  auto goal_id = utils::generate_goal_id();
  std::string order_id = "order_" + std::to_string(node_->now().nanoseconds());

  // Store callbacks and goal in the current session
  std::shared_ptr<NavigationSession> session;
  {
    std::lock_guard<std::mutex> lock(session_wp_mutex_);
    session = session_wp_.lock();
  }

  if (!session) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[NavigationController] No active session for robot: %s",
                 robot_name_.c_str());
    if (completion_callback)
      completion_callback();
    return;
  }

  {
    std::lock_guard<std::mutex> lock(session->mutex);
    session->completion_callback = completion_callback;
    session->arrival_estimator = arrival_estimator_callback;
    session->goal_id = goal_id;
    session->done.store(false);
  }

  // Send goal via Zenoh (outside lock — may block on network I/O)
  bool accepted = send_follow_path_goal(goal_id, waypoints, lanes, order_id);

  if (!accepted) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[NavigationController] MPC goal rejected for robot: %s",
                 robot_name_.c_str());

    std::function<void()> cb;
    {
      std::lock_guard<std::mutex> lock(session->mutex);
      session->done.store(true);
      cb = session->completion_callback;
    }
    if (cb)
      cb();
  }
}

void NavigationController::cancel_navigation() {
  std::shared_ptr<NavigationSession> session;
  {
    std::lock_guard<std::mutex> lock(session_wp_mutex_);
    session = session_wp_.lock();
  }

  if (!session)
    return;

  std::lock_guard<std::mutex> lock(session->mutex);

  if (!session->goal_id.has_value()) {
    RCLCPP_WARN(node_->get_logger(),
                "[NavigationController] No active goal to cancel for robot: %s",
                robot_name_.c_str());
    return;
  }

  RCLCPP_INFO(node_->get_logger(),
              "[NavigationController] Canceling navigation for robot: %s",
              robot_name_.c_str());

  session->invalidate();
}

bool NavigationController::is_navigating() const {
  std::shared_ptr<NavigationSession> session;
  {
    std::lock_guard<std::mutex> lock(session_wp_mutex_);
    session = session_wp_.lock();
  }
  if (!session)
    return false;
  return !session->done.load();
}

std::optional<std::array<uint8_t, 16>>
NavigationController::get_current_goal_id() const {
  std::shared_ptr<NavigationSession> session;
  {
    std::lock_guard<std::mutex> lock(session_wp_mutex_);
    session = session_wp_.lock();
  }
  if (!session)
    return std::nullopt;
  std::lock_guard<std::mutex> lock(session->mutex);
  return session->goal_id;
}

std::vector<egb_fleet_msgs::msg::Lane>
NavigationController::generate_lanes(size_t waypoint_count) {
  std::vector<egb_fleet_msgs::msg::Lane> lanes;
  for (size_t i = 0; i + 1 < waypoint_count; ++i) {
    egb_fleet_msgs::msg::Lane lane;
    lane.id = "lane_" + std::to_string(i);
    lane.entry = "wp_" + std::to_string(i);
    lane.exit = "wp_" + std::to_string(i + 1);
    lanes.push_back(lane);
  }
  return lanes;
}

bool NavigationController::send_follow_path_goal(
    const std::array<uint8_t, 16> &goal_id,
    const std::vector<egb_fleet_msgs::msg::Waypoint> &waypoints,
    const std::vector<egb_fleet_msgs::msg::Lane> &lanes,
    const std::string &order_id) {
  return utils::send_follow_path_goal(zenoh_session_, robot_name_, goal_id,
                                      waypoints, lanes, order_id);
}

void NavigationController::setup_zenoh_subscribers() {
  RCLCPP_INFO(
      node_->get_logger(),
      "[NavigationController] Setting up Zenoh subscribers for robot: %s",
      robot_name_.c_str());

  if (!zenoh_session_) {
    RCLCPP_ERROR(node_->get_logger(), "[NavigationController] Cannot setup "
                                      "subscribers: Zenoh session is null");
    return;
  }

  try {
    auto *session = static_cast<zenoh::Session *>(zenoh_session_);

    std::string feedback_key =
        utils::namespacify(robot_name_, "follow_waypoints/_action/feedback");
    RCLCPP_INFO(node_->get_logger(),
                "[NavigationController] Subscribing to feedback: %s",
                feedback_key.c_str());

    auto feedback_cb = [this](const zenoh::Sample &sample) {
      try {
        const auto &payload = sample.get_payload();
        auto payload_str = payload.as_string();
        std::vector<uint8_t> payload_vec(payload_str.begin(),
                                         payload_str.end());
        this->feedback_callback(payload_vec);
      } catch (const std::exception &ex) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[NavigationController] Error processing feedback: %s",
                     ex.what());
      }
    };

    auto feedback_sub = session->declare_subscriber(feedback_key, feedback_cb,
                                                    zenoh::closures::none);
    feedback_subscriber_ = new zenoh::Subscriber<void>(std::move(feedback_sub));

    std::string status_key =
        utils::namespacify(robot_name_, "follow_waypoints/_action/status");
    RCLCPP_INFO(node_->get_logger(),
                "[NavigationController] Subscribing to status: %s",
                status_key.c_str());

    auto status_cb = [this](const zenoh::Sample &sample) {
      try {
        const auto &payload = sample.get_payload();
        auto payload_str = payload.as_string();
        std::vector<uint8_t> payload_vec(payload_str.begin(),
                                         payload_str.end());
        this->status_callback(payload_vec);
      } catch (const std::exception &ex) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[NavigationController] Error processing status: %s",
                     ex.what());
      }
    };

    auto status_sub = session->declare_subscriber(status_key, status_cb,
                                                  zenoh::closures::none);
    status_subscriber_ = new zenoh::Subscriber<void>(std::move(status_sub));

    RCLCPP_INFO(node_->get_logger(),
                "[NavigationController] Successfully set up Zenoh subscribers");

  } catch (const std::exception &ex) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[NavigationController] Failed to setup Zenoh subscribers: %s",
                 ex.what());
  }
}

void NavigationController::feedback_callback(
    const std::vector<uint8_t> &payload) {
  // Promote weak_ptr — if session was replaced, this returns null and we bail
  std::shared_ptr<NavigationSession> session;
  {
    std::lock_guard<std::mutex> lock(session_wp_mutex_);
    session = session_wp_.lock();
  }
  if (!session || session->done.load())
    return;

  try {
    rclcpp::SerializedMessage serialized_msg(payload.size());
    std::memcpy(serialized_msg.get_rcl_serialized_message().buffer,
                payload.data(), payload.size());
    serialized_msg.get_rcl_serialized_message().buffer_length = payload.size();

    rclcpp::Serialization<
        egb_fleet_msgs::action::FollowWaypoints::Impl::FeedbackMessage>
        serializer;
    egb_fleet_msgs::action::FollowWaypoints::Impl::FeedbackMessage feedback_msg;
    serializer.deserialize_message(&serialized_msg, &feedback_msg);

    uint32_t target_waypoint_idx = feedback_msg.feedback.target_waypoint_index;
    double eta_seconds = feedback_msg.feedback.eta_seconds;

    // Drop feedback whose goal_id doesn't match the current session —
    // identity-based stale rejection, no bounds checks needed.
    std::function<void(size_t, double)> cb;
    std::optional<std::array<uint8_t, 16>> expected_goal_id;
    {
      std::lock_guard<std::mutex> lock(session->mutex);
      cb = session->arrival_estimator;
      expected_goal_id = session->goal_id;
    }
    if (!expected_goal_id || feedback_msg.goal_id.uuid != *expected_goal_id) {
      return;
    }
    if (cb) {
      cb(static_cast<size_t>(target_waypoint_idx), eta_seconds);
    }

  } catch (const std::exception &ex) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "[NavigationController] Error deserializing feedback for robot %s: %s",
        robot_name_.c_str(), ex.what());
  }
}

void NavigationController::status_callback(
    const std::vector<uint8_t> &payload) {
  std::shared_ptr<NavigationSession> session;
  {
    std::lock_guard<std::mutex> lock(session_wp_mutex_);
    session = session_wp_.lock();
  }
  if (!session)
    return;

  try {
    rclcpp::SerializedMessage serialized_msg(payload.size());
    std::memcpy(serialized_msg.get_rcl_serialized_message().buffer,
                payload.data(), payload.size());
    serialized_msg.get_rcl_serialized_message().buffer_length = payload.size();

    rclcpp::Serialization<action_msgs::msg::GoalStatusArray> serializer;
    action_msgs::msg::GoalStatusArray status_array;
    serializer.deserialize_message(&serialized_msg, &status_array);

    std::function<void()> cb;
    {
      std::lock_guard<std::mutex> lock(session->mutex);

      if (!session->goal_id.has_value())
        return;

      for (const auto &status : status_array.status_list) {
        if (status.goal_info.goal_id.uuid != session->goal_id.value())
          continue;

        switch (status.status) {
        case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
          RCLCPP_INFO(
              node_->get_logger(),
              "[NavigationController] Navigation succeeded for robot: %s",
              robot_name_.c_str());
          cb = session->completion_callback;
          session->invalidate();
          break;

        case action_msgs::msg::GoalStatus::STATUS_ABORTED:
        case action_msgs::msg::GoalStatus::STATUS_CANCELED:
          RCLCPP_WARN(node_->get_logger(),
                      "[NavigationController] Navigation failed for robot %s "
                      "(status: %d), requesting replan",
                      robot_name_.c_str(), static_cast<int>(status.status));
          cb = session->failure_callback;
          session->invalidate();
          break;

        case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
        case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
          break;

        default:
          RCLCPP_DEBUG(
              node_->get_logger(),
              "[NavigationController] Unknown goal status %d for robot: %s",
              static_cast<int>(status.status), robot_name_.c_str());
        }
        break; // found our goal
      }
    }
    // Fire callback outside lock (may call into RMF which could re-enter)
    if (cb)
      cb();

  } catch (const std::exception &ex) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "[NavigationController] Error deserializing status for robot %s: %s",
        robot_name_.c_str(), ex.what());
  }
}

} // namespace egb_fleet
