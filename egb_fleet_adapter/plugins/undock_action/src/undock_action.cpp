// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#include "egb_fleet/plugins/undock_action.hpp"
#include <chrono>
#include <pluginlib/class_list_macros.hpp>
#include <queue>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace egb_fleet {
namespace plugins {

/**
 * @brief Undock action implementation using graph-based path finding
 *
 * This action generates an undocking path by searching the traffic graph for
 * a dead-end waypoint reachable from the robot's current position, then
 * executes navigation along that path with the robot facing backwards.
 */
class UndockAction : public action::RobotAction {
public:
  enum class State {
    PLANNING,   // Planning the undock path
    NAVIGATING, // Navigation is in progress
    COMPLETED,  // Navigation completed successfully
    FAILED      // Action failed
  };

  UndockAction(std::shared_ptr<action::RobotActionContext> context,
               const nlohmann::json & /*description*/,
               const ActivityIdentifier &execution)
      : RobotAction(context, execution), state_(State::PLANNING),
        robot_name_(context_->robot_name), nav_completed_(false),
        creation_time_(std::chrono::steady_clock::now()) {
    RCLCPP_INFO(context_->node->get_logger(), "[%s] UndockAction created",
                robot_name_.c_str());
  }

  action::RobotActionState update_action() override {
    switch (state_) {
    case State::PLANNING:
      return handle_planning();

    case State::NAVIGATING:
      return handle_navigating();

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
  bool nav_completed_;
  std::chrono::steady_clock::time_point creation_time_;
  static constexpr double TIMEOUT = 60.0; // 60 second timeout for undocking

  // Helper functions
  std::optional<size_t> find_closest_waypoint(const Eigen::Vector3d &robot_pose,
                                              const std::string &map_name) {

    if (!context_->traffic_graph) {
      return std::nullopt;
    }

    auto starts = rmf_traffic::agv::compute_plan_starts(
        *context_->traffic_graph, map_name, robot_pose,
        rmf_traffic::Time(
            rmf_traffic::Duration(context_->node->now().nanoseconds())),
        1.0, // max_merge_waypoint_distance - 1m
        1.0  // max_merge_lane_distance
    );

    if (starts.empty()) {
      return std::nullopt;
    }

    std::set<size_t> candidate_waypoints;
    for (const auto &start : starts) {
      auto wp = start.waypoint();
      if (wp) {
        candidate_waypoints.insert(wp);
      }
      auto ln = start.lane();
      if (ln) {
        const auto &lane = context_->traffic_graph->get_lane(*ln);
        candidate_waypoints.insert(lane.entry().waypoint_index());
        candidate_waypoints.insert(lane.exit().waypoint_index());
      }
    }

    double min_distance = std::numeric_limits<double>::max();
    std::optional<size_t> closest_wp;

    for (size_t wp_idx : candidate_waypoints) {
      const auto &wp = context_->traffic_graph->get_waypoint(wp_idx);
      auto location = wp.get_location();
      Eigen::Vector2d wp_pos(location[0], location[1]);
      double distance = (wp_pos - robot_pose.head<2>()).norm();

      if (distance < min_distance) {
        min_distance = distance;
        closest_wp = wp_idx;
      }
    }

    return closest_wp;
  }

  std::vector<size_t> get_outgoing_lanes(size_t waypoint_index) {
    std::vector<size_t> outgoing_lanes;

    for (size_t lane_id = 0; lane_id < context_->traffic_graph->num_lanes();
         ++lane_id) {
      const auto &lane = context_->traffic_graph->get_lane(lane_id);
      if (lane.entry().waypoint_index() == waypoint_index) {
        outgoing_lanes.push_back(lane_id);
      }
    }

    return outgoing_lanes;
  }

  std::vector<size_t> generate_undock_path(size_t start_waypoint) {
    const int MAX_DEPTH = 5;

    std::queue<std::tuple<size_t, int, std::vector<size_t>>> bfs_queue;
    std::set<size_t> visited;

    bfs_queue.push({start_waypoint, 0, {start_waypoint}});
    visited.insert(start_waypoint);

    while (!bfs_queue.empty()) {
      auto [current_wp, depth, path] = bfs_queue.front();
      bfs_queue.pop();

      if (depth >= MAX_DEPTH) {
        continue;
      }

      // Count distinct waypoints this waypoint connects to (considering
      // bidirectional lanes)
      std::set<size_t> distinct_neighbors;
      for (size_t lane_id = 0; lane_id < context_->traffic_graph->num_lanes();
           ++lane_id) {
        const auto &lane = context_->traffic_graph->get_lane(lane_id);
        if (lane.entry().waypoint_index() == current_wp) {
          distinct_neighbors.insert(lane.exit().waypoint_index());
        }
        if (lane.exit().waypoint_index() == current_wp) {
          distinct_neighbors.insert(lane.entry().waypoint_index());
        }
      }

      // Check if this is a junction (3+ distinct connections)
      bool is_junction = (distinct_neighbors.size() >= 3);

      if (depth > 0 && is_junction) {
        RCLCPP_INFO(
            context_->node->get_logger(),
            "[%s] Found junction waypoint %zu at depth %d with %zu connections",
            robot_name_.c_str(), current_wp, depth, distinct_neighbors.size());
        return path;
      }

      // Continue BFS: get outgoing lanes from current waypoint
      auto outgoing_lanes = get_outgoing_lanes(current_wp);

      // Find unvisited destinations via outgoing lanes
      std::set<size_t> unvisited_destinations;
      for (size_t lane_id : outgoing_lanes) {
        const auto &lane = context_->traffic_graph->get_lane(lane_id);
        size_t next_wp = lane.exit().waypoint_index();
        if (visited.count(next_wp) == 0) {
          unvisited_destinations.insert(next_wp);
        }
      }

      // Enqueue all unvisited neighbors
      for (size_t next_wp : unvisited_destinations) {
        visited.insert(next_wp);
        std::vector<size_t> new_path = path;
        new_path.push_back(next_wp);
        bfs_queue.push({next_wp, depth + 1, new_path});
      }
    }

    RCLCPP_WARN(context_->node->get_logger(),
                "[%s] Failed to find junction waypoint within %d hops",
                robot_name_.c_str(), MAX_DEPTH);
    return {}; // Failed to find path
  }

  action::RobotActionState handle_planning() {
    // Check timeout
    auto now = std::chrono::steady_clock::now();
    auto elapsed =
        std::chrono::duration_cast<std::chrono::seconds>(now - creation_time_)
            .count();
    if (elapsed > TIMEOUT) {
      RCLCPP_ERROR(context_->node->get_logger(),
                   "[%s] Undock planning timed out", robot_name_.c_str());
      state_ = State::FAILED;
      return action::RobotActionState::FAILED;
    }

    // Check prerequisites
    if (!context_->traffic_graph) {
      RCLCPP_ERROR(context_->node->get_logger(),
                   "[%s] Undock action requires traffic_graph in context",
                   robot_name_.c_str());
      state_ = State::FAILED;
      return action::RobotActionState::FAILED;
    }

    if (!context_->coordinate_transforms) {
      RCLCPP_ERROR(
          context_->node->get_logger(),
          "[%s] Undock action requires coordinate_transforms in context",
          robot_name_.c_str());
      state_ = State::FAILED;
      return action::RobotActionState::FAILED;
    }

    if (!context_->execute_navigation) {
      RCLCPP_ERROR(
          context_->node->get_logger(),
          "[%s] Undock action requires execute_navigation callback in context",
          robot_name_.c_str());
      state_ = State::FAILED;
      return action::RobotActionState::FAILED;
    }

    // Step 1: Get current robot pose
    auto robot_pose = context_->get_pose();
    if (!robot_pose) {
      RCLCPP_ERROR(context_->node->get_logger(),
                   "[%s] Cannot get robot pose for undocking",
                   robot_name_.c_str());
      state_ = State::FAILED;
      return action::RobotActionState::FAILED;
    }

    std::string map_name = context_->get_map_name();

    // Step 2: Transform to RMF coordinates
    Eigen::Vector3d rmf_pose;
    if (context_->coordinate_transforms->find(map_name) !=
        context_->coordinate_transforms->end()) {
      const auto &transform = context_->coordinate_transforms->at(map_name);
      rmf_pose = transform.apply_inverse(*robot_pose);
    } else {
      rmf_pose = *robot_pose;
    }

    // Step 3: Find closest waypoint
    auto start_wp = find_closest_waypoint(rmf_pose, map_name);
    if (!start_wp) {
      RCLCPP_ERROR(context_->node->get_logger(),
                   "[%s] No waypoint found near robot position",
                   robot_name_.c_str());
      state_ = State::FAILED;
      return action::RobotActionState::FAILED;
    }

    // Step 4: Generate undock path
    std::vector<size_t> waypoint_indices = generate_undock_path(*start_wp);
    if (waypoint_indices.empty()) {
      RCLCPP_ERROR(context_->node->get_logger(),
                   "[%s] Failed to generate undock path from waypoint %zu",
                   robot_name_.c_str(), *start_wp);
      state_ = State::FAILED;
      return action::RobotActionState::FAILED;
    }

    // Step 5: Convert waypoint indices to ROS2 poses with REVERSE orientation
    std::vector<geometry_msgs::msg::PoseStamped> ros2_poses;
    ros2_poses.reserve(waypoint_indices.size());

    for (size_t i = 0; i < waypoint_indices.size(); ++i) {
      size_t wp_idx = waypoint_indices[i];
      const auto &wp = context_->traffic_graph->get_waypoint(wp_idx);
      auto location = wp.get_location();
      Eigen::Vector2d rmf_position(location[0], location[1]);

      // Apply coordinate transformation: RMF -> Robot
      Eigen::Vector2d robot_position = rmf_position;
      if (!context_->coordinate_transforms->empty()) {
        const auto &transform =
            context_->coordinate_transforms->begin()->second;
        Eigen::Vector3d rmf_pose_3d(rmf_position.x(), rmf_position.y(), 0.0);
        Eigen::Vector3d robot_pose_3d = transform.apply(rmf_pose_3d);
        robot_position = robot_pose_3d.head<2>();
      }

      // Calculate orientation pointing BACKWARDS
      double yaw = 0.0;
      if (i > 0) {
        const auto &prev_wp =
            context_->traffic_graph->get_waypoint(waypoint_indices[i - 1]);
        auto prev_loc = prev_wp.get_location();
        Eigen::Vector2d prev_rmf_pos(prev_loc[0], prev_loc[1]);

        Eigen::Vector2d prev_robot_pos = prev_rmf_pos;
        if (!context_->coordinate_transforms->empty()) {
          const auto &transform =
              context_->coordinate_transforms->begin()->second;
          Eigen::Vector3d prev_pose_3d = transform.apply(
              Eigen::Vector3d(prev_rmf_pos.x(), prev_rmf_pos.y(), 0.0));
          prev_robot_pos = prev_pose_3d.head<2>();
        }

        Eigen::Vector2d back_direction =
            (prev_robot_pos - robot_position).normalized();
        yaw = std::atan2(back_direction.y(), back_direction.x());
      } else if (i < waypoint_indices.size() - 1) {
        const auto &next_wp =
            context_->traffic_graph->get_waypoint(waypoint_indices[i + 1]);
        auto next_loc = next_wp.get_location();
        Eigen::Vector2d next_rmf_pos(next_loc[0], next_loc[1]);

        Eigen::Vector2d next_robot_pos = next_rmf_pos;
        if (!context_->coordinate_transforms->empty()) {
          const auto &transform =
              context_->coordinate_transforms->begin()->second;
          Eigen::Vector3d next_pose_3d = transform.apply(
              Eigen::Vector3d(next_rmf_pos.x(), next_rmf_pos.y(), 0.0));
          next_robot_pos = next_pose_3d.head<2>();
        }

        Eigen::Vector2d travel_direction =
            (next_robot_pos - robot_position).normalized();
        Eigen::Vector2d back_direction = -travel_direction;
        yaw = std::atan2(back_direction.y(), back_direction.x());
      }

      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = "map";
      pose_stamped.header.stamp = context_->node->now();
      pose_stamped.pose.position.x = robot_position.x();
      pose_stamped.pose.position.y = robot_position.y();
      pose_stamped.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose_stamped.pose.orientation = tf2::toMsg(q);

      ros2_poses.push_back(pose_stamped);
    }

    RCLCPP_INFO(context_->node->get_logger(),
                "[%s] Undock path: %zu waypoints converted to %zu ROS2 poses",
                robot_name_.c_str(), waypoint_indices.size(),
                ros2_poses.size());

    // Step 6: Execute navigation
    context_->execute_navigation(ros2_poses, [this]() {
      RCLCPP_INFO(context_->node->get_logger(),
                  "[%s] Undock navigation completed", robot_name_.c_str());
      nav_completed_ = true;
    });

    state_ = State::NAVIGATING;
    return action::RobotActionState::IN_PROGRESS;
  }

  action::RobotActionState handle_navigating() {
    // Check for completion
    if (nav_completed_) {
      RCLCPP_INFO(context_->node->get_logger(),
                  "[%s] Undock navigation completed successfully",
                  robot_name_.c_str());
      state_ = State::COMPLETED;
      return action::RobotActionState::COMPLETED;
    }

    // Check timeout
    auto now = std::chrono::steady_clock::now();
    auto elapsed =
        std::chrono::duration_cast<std::chrono::seconds>(now - creation_time_)
            .count();
    if (elapsed > TIMEOUT) {
      RCLCPP_ERROR(context_->node->get_logger(),
                   "[%s] Undock navigation timed out", robot_name_.c_str());
      state_ = State::FAILED;
      return action::RobotActionState::FAILED;
    }

    return action::RobotActionState::IN_PROGRESS;
  }
};

// Factory implementation
UndockActionFactory::UndockActionFactory(
    std::shared_ptr<action::RobotActionContext> context)
    : RobotActionFactory(context) {
  actions_ = {"undock"};
}

void UndockActionFactory::initialize(
    std::shared_ptr<action::RobotActionContext> context) {
  RobotActionFactory::initialize(context);
  actions_ = {"undock"};
}

bool UndockActionFactory::supports_action(const std::string &category) {
  return category == "undock";
}

std::shared_ptr<action::RobotAction>
UndockActionFactory::perform_action(const std::string &category,
                                    const nlohmann::json &description,
                                    const ActivityIdentifier &execution) {

  if (!supports_action(category)) {
    return nullptr;
  }

  return std::make_shared<UndockAction>(context_, description, execution);
}

} // namespace plugins
} // namespace egb_fleet

// Export plugin
PLUGINLIB_EXPORT_CLASS(egb_fleet::plugins::UndockActionFactory,
                       egb_fleet::action::RobotActionFactory)
