// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#ifndef EGB_FLEET__UTILS__ZENOH_ACTION_CLIENT_HPP_
#define EGB_FLEET__UTILS__ZENOH_ACTION_CLIENT_HPP_

#include <array>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

namespace egb_fleet {
namespace utils {

// No result yet (timeout or still running), distinct from a negative error_code.
constexpr int ACTION_RESULT_PENDING = 1;

/// Generate a random UUID for action goal IDs.
std::array<uint8_t, 16> generate_goal_id();

/// Prefix a topic with the robot name, matching the zenoh bridge key layout.
std::string namespacify(const std::string &robot_name,
                        const std::string &topic);

/**
 * @brief Send a serialized action request over Zenoh and return the raw reply.
 *
 * Zenoh has no ROS2 action client, so goals go over query/reply against the
 * bridge's `<robot>/<action_name>/_action/<endpoint>` keys. Bytes in, bytes
 * out, so the IDL stays with whoever owns the action.
 *
 * @param endpoint Action endpoint, "send_goal" or "get_result"
 * @return Reply payload, or nullopt on timeout or error
 */
std::optional<std::vector<uint8_t>>
query_action(void *zenoh_session, const std::string &robot_name,
             const std::string &action_name, const std::string &endpoint,
             const std::vector<uint8_t> &request, int timeout_ms);

/**
 * @brief Typed wrapper over query_action.
 *
 * Request/Response are the generated `<Action>_<SendGoal|GetResult>_<Request|
 * Response>` types. The caller instantiates it, so the action's typesupport
 * links where the action is owned rather than into the core library.
 */
template <typename Request, typename Response>
std::optional<Response>
call_action(void *zenoh_session, const std::string &robot_name,
            const std::string &action_name, const std::string &endpoint,
            const Request &request, int timeout_ms) {
  try {
    rclcpp::SerializedMessage serialized;
    rclcpp::Serialization<Request>().serialize_message(&request, &serialized);

    const auto &raw = serialized.get_rcl_serialized_message();
    const std::vector<uint8_t> payload(raw.buffer,
                                       raw.buffer + raw.buffer_length);

    auto reply = query_action(zenoh_session, robot_name, action_name, endpoint,
                              payload, timeout_ms);
    if (!reply) {
      return std::nullopt;
    }

    rclcpp::SerializedMessage reply_serialized(reply->size());
    std::memcpy(reply_serialized.get_rcl_serialized_message().buffer,
                reply->data(), reply->size());
    reply_serialized.get_rcl_serialized_message().buffer_length = reply->size();

    Response response;
    rclcpp::Serialization<Response>().deserialize_message(&reply_serialized,
                                                          &response);
    return response;

  } catch (const std::exception &) {
    return std::nullopt;
  }
}

} // namespace utils
} // namespace egb_fleet

#endif // EGB_FLEET__UTILS__ZENOH_ACTION_CLIENT_HPP_
