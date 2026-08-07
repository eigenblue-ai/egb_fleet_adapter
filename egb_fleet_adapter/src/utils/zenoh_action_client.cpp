// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#include "egb_fleet/utils/zenoh_action_client.hpp"

#include <chrono>
#include <mutex>
#include <random>
#include <variant>

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

std::optional<std::vector<uint8_t>>
query_action(void *zenoh_session, const std::string &robot_name,
             const std::string &action_name, const std::string &endpoint,
             const std::vector<uint8_t> &request, int timeout_ms) {
  if (!zenoh_session) {
    return std::nullopt;
  }

  try {
    auto *session = static_cast<zenoh::Session *>(zenoh_session);
    const std::string key =
        namespacify(robot_name, action_name + "/_action/" + endpoint);

    zenoh::Session::GetOptions options;
    options.timeout_ms = timeout_ms;
    options.payload = zenoh::Bytes(request);

    auto replies =
        session->get(zenoh::KeyExpr(key), "", zenoh::channels::FifoChannel(16),
                     std::move(options));

    const auto start = std::chrono::steady_clock::now();
    while (true) {
      auto reply = replies.recv();

      const auto elapsed =
          std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now() - start)
              .count();
      if (elapsed > timeout_ms) {
        return std::nullopt;
      }

      // RecvError means the channel closed with no more replies coming.
      if (!std::holds_alternative<zenoh::Reply>(reply)) {
        return std::nullopt;
      }

      auto &reply_val = std::get<zenoh::Reply>(reply);
      if (reply_val.is_ok()) {
        const auto payload = reply_val.get_ok().get_payload().as_string();
        return std::vector<uint8_t>(payload.begin(), payload.end());
      }
    }

  } catch (const std::exception &) {
    return std::nullopt;
  }
}

} // namespace utils
} // namespace egb_fleet
