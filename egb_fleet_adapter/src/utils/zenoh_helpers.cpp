// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#include "egb_fleet/utils/zenoh_helpers.hpp"
#include <cstring>

#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include <chrono>
#include <egb_fleet_msgs/action/charge_battery.hpp>
#include <egb_fleet_msgs/action/follow_path.hpp>
#include <egb_fleet_msgs/action/follow_waypoints.hpp>
#include <egb_fleet_msgs/action/lift_drop.hpp>
#include <egb_fleet_msgs/msg/lane.hpp>
#include <egb_fleet_msgs/msg/waypoint.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>
#include <random>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <thread>
#include <zenoh.hxx>

namespace egb_fleet {
namespace utils {

std::array<uint8_t, 16> generate_goal_id() {
  static std::mutex rng_mutex;
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_int_distribution<uint8_t> dis(0, 255);

  std::lock_guard<std::mutex> lock(rng_mutex);
  std::array<uint8_t, 16> goal_id;
  for (auto &byte : goal_id) {
    byte = dis(gen);
  }

  // Set UUID version 4 bits
  goal_id[6] = (goal_id[6] & 0x0F) | 0x40;
  goal_id[8] = (goal_id[8] & 0x3F) | 0x80;

  return goal_id;
}

std::string namespacify(const std::string &robot_name,
                        const std::string &topic) {
  if (!robot_name.empty() && !topic.empty()) {
    return robot_name + "/" + topic;
  }
  return topic;
}

bool send_follow_waypoints_goal(
    void *zenoh_session, const std::string &robot_name,
    const std::array<uint8_t, 16> &goal_id,
    const std::vector<egb_fleet_msgs::msg::Waypoint> &waypoints,
    const std::vector<egb_fleet_msgs::msg::Lane> &lanes, int timeout_ms) {
  if (!zenoh_session || waypoints.empty()) {
    return false;
  }

  try {
    auto *session = static_cast<zenoh::Session *>(zenoh_session);

    // Create FollowWaypoints SendGoal request
    egb_fleet_msgs::action::FollowWaypoints_SendGoal_Request request;

    // Set goal_id (UUID)
    std::copy(goal_id.begin(), goal_id.end(), request.goal_id.uuid.begin());

    // Set waypoints and lanes
    request.goal.waypoints = waypoints;
    request.goal.lanes = lanes;

    // Serialize the request
    rclcpp::Serialization<
        egb_fleet_msgs::action::FollowWaypoints_SendGoal_Request>
        serializer;
    rclcpp::SerializedMessage serialized_msg;
    serializer.serialize_message(&request, &serialized_msg);

    // Build Zenoh key expression
    std::string key =
        namespacify(robot_name, "follow_waypoints/_action/send_goal");

    // Create query options
    zenoh::Session::GetOptions options;
    options.timeout_ms = timeout_ms;

    // Attach payload
    auto rcl_msg = serialized_msg.get_rcl_serialized_message();
    std::vector<uint8_t> payload_vec(rcl_msg.buffer,
                                     rcl_msg.buffer + rcl_msg.buffer_length);
    options.payload = zenoh::Bytes(payload_vec);

    // Send Zenoh query
    auto replies =
        session->get(zenoh::KeyExpr(key), "", zenoh::channels::FifoChannel(16),
                     std::move(options));

    // Wait for response
    auto start = std::chrono::steady_clock::now();
    while (true) {
      auto reply = replies.recv();

      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now() - start)
                         .count();

      if (elapsed > timeout_ms) {
        return false;
      }

      if (std::holds_alternative<zenoh::Reply>(reply)) {
        auto &reply_val = std::get<zenoh::Reply>(reply);
        if (reply_val.is_ok()) {
          const auto &sample = reply_val.get_ok();
          const auto &payload = sample.get_payload();
          auto payload_str = payload.as_string();

          // Deserialize response
          egb_fleet_msgs::action::FollowWaypoints_SendGoal_Response response;
          rclcpp::SerializedMessage resp_serialized(payload_str.size());

          // Copy payload data into buffer
          std::memcpy(resp_serialized.get_rcl_serialized_message().buffer,
                      payload_str.data(), payload_str.size());
          resp_serialized.get_rcl_serialized_message().buffer_length =
              payload_str.size();

          rclcpp::Serialization<
              egb_fleet_msgs::action::FollowWaypoints_SendGoal_Response>
              resp_serializer;
          resp_serializer.deserialize_message(&resp_serialized, &response);

          return response.accepted;
        }
      }
    }

  } catch (const std::exception &e) {
    return false;
  }
}

int get_follow_waypoints_result(void *zenoh_session,
                                const std::string &robot_name,
                                const std::array<uint8_t, 16> &goal_id,
                                int timeout_ms) {
  /*
   * Get the status of a FollowWaypoints action goal via Zenoh.
   * Same implementation as get_navigate_through_poses_result() but for
   * FollowWaypoints action.
   */

  if (!zenoh_session) {
    return static_cast<int>(action_msgs::msg::GoalStatus::STATUS_UNKNOWN);
  }

  try {
    auto *session = static_cast<zenoh::Session *>(zenoh_session);

    // Create the FollowWaypoints GetResult request
    egb_fleet_msgs::action::FollowWaypoints_GetResult_Request request;

    // Set goal_id (UUID)
    std::copy(goal_id.begin(), goal_id.end(), request.goal_id.uuid.begin());

    // Serialize the request
    rclcpp::Serialization<
        egb_fleet_msgs::action::FollowWaypoints_GetResult_Request>
        serializer;
    rclcpp::SerializedMessage serialized_msg;
    serializer.serialize_message(&request, &serialized_msg);

    // Build Zenoh key expression for FollowWaypoints
    std::string key =
        namespacify(robot_name, "follow_waypoints/_action/get_result");

    // Create query options with payload and timeout
    zenoh::Session::GetOptions options;
    options.timeout_ms = timeout_ms;

    // Attach serialized request as payload
    auto rcl_msg = serialized_msg.get_rcl_serialized_message();
    std::vector<uint8_t> payload_vec(rcl_msg.buffer,
                                     rcl_msg.buffer + rcl_msg.buffer_length);
    options.payload = zenoh::Bytes(payload_vec);

    // Send Zenoh query
    auto replies =
        session->get(zenoh::KeyExpr(key),
                     "", // Empty parameters string
                     zenoh::channels::FifoChannel(16), std::move(options));

    // Wait for response with timeout
    auto start = std::chrono::steady_clock::now();
    while (true) {
      auto reply = replies.recv();

      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now() - start)
                         .count();

      if (elapsed > timeout_ms) {
        return static_cast<int>(action_msgs::msg::GoalStatus::STATUS_UNKNOWN);
      }

      // Check if reply is valid (not an error)
      if (std::holds_alternative<zenoh::Reply>(reply)) {
        auto &reply_val = std::get<zenoh::Reply>(reply);

        if (reply_val.is_ok()) {
          const auto &sample = reply_val.get_ok();
          const auto &payload = sample.get_payload();
          auto payload_str = payload.as_string();

          // Deserialize FollowWaypoints response
          egb_fleet_msgs::action::FollowWaypoints_GetResult_Response response;
          rclcpp::SerializedMessage resp_serialized(payload_str.size());

          std::memcpy(resp_serialized.get_rcl_serialized_message().buffer,
                      payload_str.data(), payload_str.size());
          resp_serialized.get_rcl_serialized_message().buffer_length =
              payload_str.size();

          rclcpp::Serialization<
              egb_fleet_msgs::action::FollowWaypoints_GetResult_Response>
              resp_serializer;
          resp_serializer.deserialize_message(&resp_serialized, &response);

          return response.status;
        }
      } else {
        // RecvError - channel closed or no more replies
        break;
      }
    }

    return static_cast<int>(action_msgs::msg::GoalStatus::STATUS_UNKNOWN);

  } catch (const std::exception &e) {
    return static_cast<int>(action_msgs::msg::GoalStatus::STATUS_UNKNOWN);
  }
}

bool send_lift_drop_goal(void *zenoh_session, const std::string &robot_name,
                         const std::array<uint8_t, 16> &goal_id,
                         int8_t operation, const std::string &car_location,
                         const std::string &car_name, double timeout,
                         int timeout_ms) {
  if (!zenoh_session) {
    return false;
  }

  try {
    auto *session = static_cast<zenoh::Session *>(zenoh_session);

    // Create LiftDrop SendGoal request
    egb_fleet_msgs::action::LiftDrop_SendGoal_Request request;

    // Set goal_id (UUID)
    std::copy(goal_id.begin(), goal_id.end(), request.goal_id.uuid.begin());

    // Set goal parameters
    request.goal.operation = operation;
    request.goal.timeout = timeout;
    request.goal.car_location = car_location;
    request.goal.car_name = car_name;
    request.goal.robot_name = robot_name;

    // Serialize the request
    rclcpp::Serialization<egb_fleet_msgs::action::LiftDrop_SendGoal_Request>
        serializer;
    rclcpp::SerializedMessage serialized_msg;
    serializer.serialize_message(&request, &serialized_msg);

    // Build Zenoh key expression
    std::string key = namespacify(robot_name, "lift_drop/_action/send_goal");

    // Create query options
    zenoh::Session::GetOptions options;
    options.timeout_ms = timeout_ms;

    // Attach payload
    auto rcl_msg = serialized_msg.get_rcl_serialized_message();
    std::vector<uint8_t> payload_vec(rcl_msg.buffer,
                                     rcl_msg.buffer + rcl_msg.buffer_length);
    options.payload = zenoh::Bytes(payload_vec);

    // Send Zenoh query
    auto replies =
        session->get(zenoh::KeyExpr(key), "", zenoh::channels::FifoChannel(16),
                     std::move(options));

    // Wait for response
    auto start = std::chrono::steady_clock::now();
    while (true) {
      auto reply = replies.recv();

      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now() - start)
                         .count();

      if (elapsed > timeout_ms) {
        return false;
      }

      if (std::holds_alternative<zenoh::Reply>(reply)) {
        auto &reply_val = std::get<zenoh::Reply>(reply);
        if (reply_val.is_ok()) {
          const auto &sample = reply_val.get_ok();
          const auto &payload = sample.get_payload();
          auto payload_str = payload.as_string();

          // Deserialize response
          egb_fleet_msgs::action::LiftDrop_SendGoal_Response response;
          rclcpp::SerializedMessage resp_serialized(payload_str.size());

          // Copy payload data into buffer
          std::memcpy(resp_serialized.get_rcl_serialized_message().buffer,
                      payload_str.data(), payload_str.size());
          resp_serialized.get_rcl_serialized_message().buffer_length =
              payload_str.size();

          rclcpp::Serialization<
              egb_fleet_msgs::action::LiftDrop_SendGoal_Response>
              resp_serializer;
          resp_serializer.deserialize_message(&resp_serialized, &response);

          return response.accepted;
        }
      } else {
        // RecvError - channel closed or no more replies
        break;
      }
    }

    return false; // No valid response

  } catch (const std::exception &e) {
    return false;
  }
}

int get_lift_drop_result(void *zenoh_session, const std::string &robot_name,
                         const std::array<uint8_t, 16> &goal_id,
                         int timeout_ms) {
  if (!zenoh_session) {
    return ACTION_RESULT_PENDING;
  }

  try {
    auto *session = static_cast<zenoh::Session *>(zenoh_session);

    // Create GetResult request
    egb_fleet_msgs::action::LiftDrop_GetResult_Request request;
    std::copy(goal_id.begin(), goal_id.end(), request.goal_id.uuid.begin());

    // Serialize request
    rclcpp::Serialization<egb_fleet_msgs::action::LiftDrop_GetResult_Request>
        serializer;
    rclcpp::SerializedMessage serialized_msg;
    serializer.serialize_message(&request, &serialized_msg);

    // Build Zenoh key
    std::string key = namespacify(robot_name, "lift_drop/_action/get_result");

    // Create query options
    zenoh::Session::GetOptions options;
    options.timeout_ms = timeout_ms;

    // Attach payload
    auto rcl_msg = serialized_msg.get_rcl_serialized_message();
    std::vector<uint8_t> payload_vec(rcl_msg.buffer,
                                     rcl_msg.buffer + rcl_msg.buffer_length);
    options.payload = zenoh::Bytes(payload_vec);

    // Send Zenoh query
    auto replies =
        session->get(zenoh::KeyExpr(key), "", zenoh::channels::FifoChannel(16),
                     std::move(options));

    // Wait for response
    auto start = std::chrono::steady_clock::now();
    while (true) {
      auto reply = replies.recv();

      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now() - start)
                         .count();

      if (elapsed > timeout_ms) {
        return ACTION_RESULT_PENDING;
      }

      if (std::holds_alternative<zenoh::Reply>(reply)) {
        auto &reply_val = std::get<zenoh::Reply>(reply);
        if (reply_val.is_ok()) {
          const auto &sample = reply_val.get_ok();
          const auto &payload = sample.get_payload();
          auto payload_str = payload.as_string();

          // Deserialize response
          egb_fleet_msgs::action::LiftDrop_GetResult_Response response;
          rclcpp::SerializedMessage resp_serialized(payload_str.size());

          std::memcpy(resp_serialized.get_rcl_serialized_message().buffer,
                      payload_str.data(), payload_str.size());
          resp_serialized.get_rcl_serialized_message().buffer_length =
              payload_str.size();

          rclcpp::Serialization<
              egb_fleet_msgs::action::LiftDrop_GetResult_Response>
              resp_serializer;
          resp_serializer.deserialize_message(&resp_serialized, &response);

          // failure must be negative so the caller fails fast, not retries
          if (response.result.success)
            return 0;
          return response.result.error_code < 0 ? response.result.error_code
                                                 : -1;
        }
      } else {
        break;
      }
    }

    return ACTION_RESULT_PENDING;

  } catch (const std::exception &e) {
    return ACTION_RESULT_PENDING;
  }
}

/**
 * @brief Send FollowWaypoints goal to MPC controller via Zenoh
 *
 * Sends a FollowWaypoints goal to the MPC local planner action server
 * using the Zenoh query/reply pattern for cross-domain communication.
 */
bool send_follow_path_goal(
    void *zenoh_session, const std::string &robot_name,
    const std::array<uint8_t, 16> &goal_id,
    const std::vector<egb_fleet_msgs::msg::Waypoint> &waypoints,
    const std::vector<egb_fleet_msgs::msg::Lane> &lanes,
    const std::string &order_id, int timeout_ms) {

  if (!zenoh_session || waypoints.empty()) {
    return false;
  }

  try {
    auto *session = static_cast<zenoh::Session *>(zenoh_session);

    // Create FollowWaypoints SendGoal request
    egb_fleet_msgs::action::FollowWaypoints_SendGoal_Request request;

    // Set goal_id (UUID)
    std::copy(goal_id.begin(), goal_id.end(), request.goal_id.uuid.begin());

    // Set goal data
    request.goal.waypoints = waypoints;
    request.goal.lanes = lanes;

    // Serialize the request
    rclcpp::Serialization<
        egb_fleet_msgs::action::FollowWaypoints_SendGoal_Request>
        serializer;
    rclcpp::SerializedMessage serialized_msg;
    serializer.serialize_message(&request, &serialized_msg);

    // Build Zenoh key expression
    std::string key =
        namespacify(robot_name, "follow_waypoints/_action/send_goal");

    // Create query options
    zenoh::Session::GetOptions options;
    options.timeout_ms = timeout_ms;

    // Attach payload
    auto rcl_msg = serialized_msg.get_rcl_serialized_message();
    std::vector<uint8_t> payload_vec(rcl_msg.buffer,
                                     rcl_msg.buffer + rcl_msg.buffer_length);
    options.payload = zenoh::Bytes(payload_vec);

    // Send Zenoh query
    auto replies =
        session->get(zenoh::KeyExpr(key), "", zenoh::channels::FifoChannel(16),
                     std::move(options));

    // Wait for response
    auto start = std::chrono::steady_clock::now();
    while (true) {
      auto reply = replies.recv();

      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now() - start)
                         .count();

      if (elapsed > timeout_ms) {
        return false;
      }

      if (std::holds_alternative<zenoh::Reply>(reply)) {
        auto &reply_val = std::get<zenoh::Reply>(reply);
        if (reply_val.is_ok()) {
          const auto &sample = reply_val.get_ok();
          const auto &payload = sample.get_payload();
          auto payload_str = payload.as_string();

          // Deserialize response
          egb_fleet_msgs::action::FollowWaypoints_SendGoal_Response response;
          rclcpp::SerializedMessage resp_serialized(payload_str.size());

          // Copy payload data into buffer
          std::memcpy(resp_serialized.get_rcl_serialized_message().buffer,
                      payload_str.data(), payload_str.size());
          resp_serialized.get_rcl_serialized_message().buffer_length =
              payload_str.size();

          rclcpp::Serialization<
              egb_fleet_msgs::action::FollowWaypoints_SendGoal_Response>
              resp_serializer;
          resp_serializer.deserialize_message(&resp_serialized, &response);

          return response.accepted;
        }
      }
    }

  } catch (const std::exception &e) {
    return false;
  }
}

bool send_charge_battery_goal(void *zenoh_session,
                              const std::string &robot_name,
                              const std::array<uint8_t, 16> &goal_id,
                              float target_soc, double timeout,
                              int timeout_ms) {
  if (!zenoh_session) {
    return false;
  }

  try {
    auto *session = static_cast<zenoh::Session *>(zenoh_session);

    // Create ChargeBattery SendGoal request
    egb_fleet_msgs::action::ChargeBattery_SendGoal_Request request;

    // Set goal_id (UUID)
    std::copy(goal_id.begin(), goal_id.end(), request.goal_id.uuid.begin());

    // Set goal parameters
    request.goal.robot_id = robot_name;
    request.goal.target_soc = target_soc;
    request.goal.timeout = timeout;

    // Serialize the request
    rclcpp::Serialization<
        egb_fleet_msgs::action::ChargeBattery_SendGoal_Request>
        serializer;
    rclcpp::SerializedMessage serialized_msg;
    serializer.serialize_message(&request, &serialized_msg);

    // Build Zenoh key expression
    std::string key =
        namespacify(robot_name, "charge_battery/_action/send_goal");

    // Create query options
    zenoh::Session::GetOptions options;
    options.timeout_ms = timeout_ms;

    // Attach payload
    auto rcl_msg = serialized_msg.get_rcl_serialized_message();
    std::vector<uint8_t> payload_vec(rcl_msg.buffer,
                                     rcl_msg.buffer + rcl_msg.buffer_length);
    options.payload = zenoh::Bytes(payload_vec);

    // Send Zenoh query
    auto replies =
        session->get(zenoh::KeyExpr(key), "", zenoh::channels::FifoChannel(16),
                     std::move(options));

    // Wait for response
    auto start = std::chrono::steady_clock::now();
    while (true) {
      auto reply = replies.recv();

      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now() - start)
                         .count();

      if (elapsed > timeout_ms) {
        return false;
      }

      if (std::holds_alternative<zenoh::Reply>(reply)) {
        auto &reply_val = std::get<zenoh::Reply>(reply);
        if (reply_val.is_ok()) {
          const auto &sample = reply_val.get_ok();
          const auto &payload = sample.get_payload();
          auto payload_str = payload.as_string();

          // Deserialize response
          egb_fleet_msgs::action::ChargeBattery_SendGoal_Response response;
          rclcpp::SerializedMessage resp_serialized(payload_str.size());

          // Copy payload data into buffer
          std::memcpy(resp_serialized.get_rcl_serialized_message().buffer,
                      payload_str.data(), payload_str.size());
          resp_serialized.get_rcl_serialized_message().buffer_length =
              payload_str.size();

          rclcpp::Serialization<
              egb_fleet_msgs::action::ChargeBattery_SendGoal_Response>
              resp_serializer;
          resp_serializer.deserialize_message(&resp_serialized, &response);

          return response.accepted;
        }
      } else {
        // RecvError - channel closed or no more replies
        break;
      }
    }

    return false; // No valid response

  } catch (const std::exception &e) {
    return false;
  }
}

int get_charge_battery_result(void *zenoh_session,
                              const std::string &robot_name,
                              const std::array<uint8_t, 16> &goal_id,
                              int timeout_ms) {
  if (!zenoh_session) {
    return ACTION_RESULT_PENDING;
  }

  try {
    auto *session = static_cast<zenoh::Session *>(zenoh_session);

    // Create GetResult request
    egb_fleet_msgs::action::ChargeBattery_GetResult_Request request;
    std::copy(goal_id.begin(), goal_id.end(), request.goal_id.uuid.begin());

    // Serialize request
    rclcpp::Serialization<
        egb_fleet_msgs::action::ChargeBattery_GetResult_Request>
        serializer;
    rclcpp::SerializedMessage serialized_msg;
    serializer.serialize_message(&request, &serialized_msg);

    // Build Zenoh key
    std::string key =
        namespacify(robot_name, "charge_battery/_action/get_result");

    // Create query options
    zenoh::Session::GetOptions options;
    options.timeout_ms = timeout_ms;

    // Attach payload
    auto rcl_msg = serialized_msg.get_rcl_serialized_message();
    std::vector<uint8_t> payload_vec(rcl_msg.buffer,
                                     rcl_msg.buffer + rcl_msg.buffer_length);
    options.payload = zenoh::Bytes(payload_vec);

    // Send Zenoh query
    auto replies =
        session->get(zenoh::KeyExpr(key), "", zenoh::channels::FifoChannel(16),
                     std::move(options));

    // Wait for response
    auto start = std::chrono::steady_clock::now();
    while (true) {
      auto reply = replies.recv();

      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now() - start)
                         .count();

      if (elapsed > timeout_ms) {
        return ACTION_RESULT_PENDING;
      }

      if (std::holds_alternative<zenoh::Reply>(reply)) {
        auto &reply_val = std::get<zenoh::Reply>(reply);
        if (reply_val.is_ok()) {
          const auto &sample = reply_val.get_ok();
          const auto &payload = sample.get_payload();
          auto payload_str = payload.as_string();

          // Deserialize response
          egb_fleet_msgs::action::ChargeBattery_GetResult_Response response;
          rclcpp::SerializedMessage resp_serialized(payload_str.size());

          std::memcpy(resp_serialized.get_rcl_serialized_message().buffer,
                      payload_str.data(), payload_str.size());
          resp_serialized.get_rcl_serialized_message().buffer_length =
              payload_str.size();

          rclcpp::Serialization<
              egb_fleet_msgs::action::ChargeBattery_GetResult_Response>
              resp_serializer;
          resp_serializer.deserialize_message(&resp_serialized, &response);

          // failure must be negative so the caller fails fast, not retries
          if (response.result.success)
            return 0;
          return response.result.error_code < 0 ? response.result.error_code
                                                 : -1;
        }
      } else {
        break;
      }
    }

    return ACTION_RESULT_PENDING; // No valid response

  } catch (const std::exception &e) {
    return ACTION_RESULT_PENDING;
  }
}

} // namespace utils
} // namespace egb_fleet
