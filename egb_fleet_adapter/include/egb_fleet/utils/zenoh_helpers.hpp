// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#ifndef EGB_FLEET__UTILS__ZENOH_HELPERS_HPP_
#define EGB_FLEET__UTILS__ZENOH_HELPERS_HPP_

#include <array>
#include <string>
#include <vector>

#include <egb_fleet_msgs/msg/lane.hpp>
#include <egb_fleet_msgs/msg/waypoint.hpp>

// Plugins get generate_goal_id / namespacify / ACTION_RESULT_PENDING from
// here without pulling in the navigation actions below.
#include "egb_fleet/utils/zenoh_action_client.hpp"

namespace egb_fleet {
namespace utils {

/**
 * @brief Send FollowWaypoints goal via Zenoh query/reply
 *
 * @param zenoh_session Zenoh session pointer
 * @param robot_name Robot name for topic namespacing
 * @param goal_id UUID for the goal
 * @param waypoints Vector of Waypoint messages for waypoints
 * @param lanes Vector of Lane messages connecting waypoints
 * @param timeout_ms Timeout in milliseconds
 * @return true if goal was accepted
 */
bool send_follow_waypoints_goal(
    void *zenoh_session, const std::string &robot_name,
    const std::array<uint8_t, 16> &goal_id,
    const std::vector<egb_fleet_msgs::msg::Waypoint> &waypoints,
    const std::vector<egb_fleet_msgs::msg::Lane> &lanes, int timeout_ms = 5000);

/**
 * @brief Send FollowPath goal via Zenoh query/reply (MPC controller)
 *
 * Goes to the same follow_waypoints action server as
 * send_follow_waypoints_goal. order_id is not carried by the goal.
 *
 * @param zenoh_session Zenoh session pointer
 * @param robot_name Robot name for topic namespacing
 * @param goal_id UUID for the goal
 * @param waypoints Vector of Waypoint messages
 * @param lanes Vector of Lane messages connecting waypoints
 * @param order_id Task/order identifier
 * @param timeout_ms Timeout in milliseconds
 * @return true if goal was accepted by the controller
 */
bool send_follow_path_goal(
    void *zenoh_session, const std::string &robot_name,
    const std::array<uint8_t, 16> &goal_id,
    const std::vector<egb_fleet_msgs::msg::Waypoint> &waypoints,
    const std::vector<egb_fleet_msgs::msg::Lane> &lanes,
    const std::string &order_id, int timeout_ms = 5000);

/**
 * @brief Get FollowWaypoints action result via Zenoh
 * @param zenoh_session Void pointer to zenoh::Session
 * @param robot_name Robot name
 * @param goal_id Goal UUID
 * @param timeout_ms Timeout in milliseconds
 * @return Status code (STATUS_SUCCEEDED=4, STATUS_EXECUTING=2, etc.)
 */
int get_follow_waypoints_result(void *zenoh_session,
                                const std::string &robot_name,
                                const std::array<uint8_t, 16> &goal_id,
                                int timeout_ms = 1000);

} // namespace utils
} // namespace egb_fleet

#endif // EGB_FLEET__UTILS__ZENOH_HELPERS_HPP_
