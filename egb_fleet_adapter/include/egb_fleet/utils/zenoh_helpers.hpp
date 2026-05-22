// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#ifndef EGB_FLEET__UTILS__ZENOH_HELPERS_HPP_
#define EGB_FLEET__UTILS__ZENOH_HELPERS_HPP_

#include <array>
#include <egb_fleet_msgs/msg/lane.hpp>
#include <egb_fleet_msgs/msg/waypoint.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <random>
#include <string>
#include <vector>
#include <zenoh.hxx>

namespace egb_fleet {
namespace utils {

/**
 * @brief Generate a random UUID for action goal IDs
 * @return 16-byte array representing a UUID
 */
std::array<uint8_t, 16> generate_goal_id();

/**
 * @brief Create namespaced topic name for Zenoh
 *
 * Python reference (utils.py:20-24):
 * def namespacify(robot_name: str, topic: str) -> str:
 *     '''Adds namespace to a topic by prefixing the robot name'''
 *     if robot_name and topic:
 *         return f'{robot_name}/{topic}'
 *     return topic
 *
 * @param robot_name Robot name prefix
 * @param topic Topic name
 * @return Namespaced topic string
 */
std::string namespacify(const std::string &robot_name,
                        const std::string &topic);

/**
 * @brief Send ROS2 NavigateToPose goal via Zenoh query/reply
 *
 * This uses Zenoh's query-reply pattern to send action goals to ROS2,
 * since Zenoh doesn't support ROS2 action servers directly.
 *
 * Python reference (ros2_robot_adapter.py uses zenoh session.get() for goals):
 *   replies = self.zenoh_session.get(
 *       key,
 *       zenoh.Queue(),
 *       value=req.serialize()
 *   )
 *
 * @param zenoh_session Void pointer to zenoh::Session (cast internally)
 * @param robot_name Robot name for topic namespacing
 * @param goal_id UUID for the goal
 * @param pose Target pose
 * @param timeout_ms Timeout in milliseconds
 * @return true if goal was accepted
 */
bool send_ros2_goal(
    void *zenoh_session, const std::string &robot_name,
    const std::array<uint8_t, 16> &goal_id,
    const std::vector<double> &pose, // [x, y, z, qx, qy, qz, qw]
    int timeout_ms = 5000);

/**
 * @brief Send ROS2 NavigateThroughPoses goal via Zenoh query/reply
 * @param zenoh_session Void pointer to zenoh::Session
 * @param robot_name Robot name for topic namespacing
 * @param goal_id UUID for the goal
 * @param poses Vector of PoseStamped messages for the path
 * @param timeout_ms Timeout in milliseconds
 * @return true if goal was accepted
 */
bool send_navigate_through_poses_goal(
    void *zenoh_session, const std::string &robot_name,
    const std::array<uint8_t, 16> &goal_id,
    const std::vector<geometry_msgs::msg::PoseStamped> &poses,
    int timeout_ms = 5000);

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
 * Sends a FollowPath goal to the MPC local planner action server.
 * The goal includes waypoints, lanes, and task identifier.
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
 * @brief Get ROS2 NavigateToPose action result via Zenoh
 * @param zenoh_session Void pointer to zenoh::Session
 * @param robot_name Robot name
 * @param goal_id Goal UUID
 * @param timeout_ms Timeout in milliseconds
 * @return Status code (0 = success, negative = error, positive = in progress)
 */
int get_ros2_result(void *zenoh_session, const std::string &robot_name,
                    const std::array<uint8_t, 16> &goal_id,
                    int timeout_ms = 1000);

/**
 * @brief Get ROS2 NavigateThroughPoses action result via Zenoh
 * @param zenoh_session Void pointer to zenoh::Session
 * @param robot_name Robot name
 * @param goal_id Goal UUID
 * @param timeout_ms Timeout in milliseconds
 * @return Status code (STATUS_SUCCEEDED=4, STATUS_EXECUTING=2, etc.)
 */
int get_navigate_through_poses_result(void *zenoh_session,
                                      const std::string &robot_name,
                                      const std::array<uint8_t, 16> &goal_id,
                                      int timeout_ms = 1000);

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

/**
 * @brief Cancel ROS2 action via Zenoh
 * @param zenoh_session Void pointer to zenoh::Session
 * @param robot_name Robot name
 * @param goal_id Goal UUID
 * @return true if cancellation was acknowledged
 */
bool cancel_ros2_goal(void *zenoh_session, const std::string &robot_name,
                      const std::array<uint8_t, 16> &goal_id);

/**
 * @brief Send LiftDrop action goal via Zenoh query/reply
 * @param zenoh_session Zenoh session pointer
 * @param robot_name Robot name for topic namespacing
 * @param goal_id UUID for the goal
 * @param operation Operation type (1=LIFT_UP, 0=LIFT_DOWN)
 * @param car_location Location where car is/will be
 * @param car_name Name of the car to attach/detach
 * @param timeout Timeout in seconds
 * @param timeout_ms Zenoh query timeout in milliseconds
 * @return true if goal was accepted
 */
bool send_lift_drop_goal(void *zenoh_session, const std::string &robot_name,
                         const std::array<uint8_t, 16> &goal_id,
                         int8_t operation, const std::string &car_location,
                         const std::string &car_name, double timeout = 30.0,
                         int timeout_ms = 5000);

// No result yet (timeout / still running); distinct from a negative error_code.
constexpr int ACTION_RESULT_PENDING = 1;

// Returns 0 = success, <0 = action error_code, ACTION_RESULT_PENDING = retry.
int get_lift_drop_result(void *zenoh_session, const std::string &robot_name,
                         const std::array<uint8_t, 16> &goal_id,
                         int timeout_ms = 1000);

/**
 * @brief Send ChargeBattery action goal via Zenoh query/reply
 *
 * @param zenoh_session Zenoh session pointer
 * @param robot_name Robot name for topic namespacing
 * @param goal_id UUID for the goal
 * @param target_soc Target state of charge (0.0-1.0)
 * @param timeout Timeout in seconds
 * @param timeout_ms Zenoh query timeout in milliseconds
 * @return true if goal was accepted
 */
bool send_charge_battery_goal(void *zenoh_session,
                              const std::string &robot_name,
                              const std::array<uint8_t, 16> &goal_id,
                              float target_soc, double timeout = 3600.0,
                              int timeout_ms = 5000);

/**
 * @brief Get ChargeBattery action result via Zenoh
 * @param zenoh_session Void pointer to zenoh::Session
 * @param robot_name Robot name
 * @param goal_id Goal UUID
 * @param timeout_ms Timeout in milliseconds
 * @return 0 = success, <0 = action error_code (failed), ACTION_RESULT_PENDING =
 *         no result yet (keep polling)
 */
int get_charge_battery_result(void *zenoh_session,
                              const std::string &robot_name,
                              const std::array<uint8_t, 16> &goal_id,
                              int timeout_ms = 1000);

} // namespace utils
} // namespace egb_fleet

#endif // EGB_FLEET__UTILS__ZENOH_HELPERS_HPP_
