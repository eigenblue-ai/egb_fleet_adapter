// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#include "egb_fleet/utils/zenoh_helpers.hpp"

#include <algorithm>

#include <action_msgs/msg/goal_status.hpp>
#include <egb_fleet_msgs/action/follow_waypoints.hpp>

namespace egb_fleet {
namespace utils {
namespace {

using SendGoalRequest =
    egb_fleet_msgs::action::FollowWaypoints_SendGoal_Request;
using SendGoalResponse =
    egb_fleet_msgs::action::FollowWaypoints_SendGoal_Response;
using GetResultRequest =
    egb_fleet_msgs::action::FollowWaypoints_GetResult_Request;
using GetResultResponse =
    egb_fleet_msgs::action::FollowWaypoints_GetResult_Response;

bool send_waypoints(void *zenoh_session, const std::string &robot_name,
                    const std::array<uint8_t, 16> &goal_id,
                    const std::vector<egb_fleet_msgs::msg::Waypoint> &waypoints,
                    const std::vector<egb_fleet_msgs::msg::Lane> &lanes,
                    int timeout_ms) {
  if (!zenoh_session || waypoints.empty()) {
    return false;
  }

  SendGoalRequest request;
  std::copy(goal_id.begin(), goal_id.end(), request.goal_id.uuid.begin());
  request.goal.waypoints = waypoints;
  request.goal.lanes = lanes;

  auto response = call_action<SendGoalRequest, SendGoalResponse>(
      zenoh_session, robot_name, "follow_waypoints", "send_goal", request,
      timeout_ms);
  return response && response->accepted;
}

} // namespace

bool send_follow_waypoints_goal(
    void *zenoh_session, const std::string &robot_name,
    const std::array<uint8_t, 16> &goal_id,
    const std::vector<egb_fleet_msgs::msg::Waypoint> &waypoints,
    const std::vector<egb_fleet_msgs::msg::Lane> &lanes, int timeout_ms) {
  return send_waypoints(zenoh_session, robot_name, goal_id, waypoints, lanes,
                        timeout_ms);
}

bool send_follow_path_goal(
    void *zenoh_session, const std::string &robot_name,
    const std::array<uint8_t, 16> &goal_id,
    const std::vector<egb_fleet_msgs::msg::Waypoint> &waypoints,
    const std::vector<egb_fleet_msgs::msg::Lane> &lanes,
    const std::string &order_id, int timeout_ms) {
  (void)order_id; // FollowWaypoints goal has no order field
  return send_waypoints(zenoh_session, robot_name, goal_id, waypoints, lanes,
                        timeout_ms);
}

int get_follow_waypoints_result(void *zenoh_session,
                                const std::string &robot_name,
                                const std::array<uint8_t, 16> &goal_id,
                                int timeout_ms) {
  GetResultRequest request;
  std::copy(goal_id.begin(), goal_id.end(), request.goal_id.uuid.begin());

  auto response = call_action<GetResultRequest, GetResultResponse>(
      zenoh_session, robot_name, "follow_waypoints", "get_result", request,
      timeout_ms);

  return response
             ? response->status
             : static_cast<int>(action_msgs::msg::GoalStatus::STATUS_UNKNOWN);
}

} // namespace utils
} // namespace egb_fleet
