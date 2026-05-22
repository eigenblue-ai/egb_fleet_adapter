// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#include "egb_fleet/path_handle.hpp"
#include "egb_fleet/action/robot_action_factory.hpp"
#include "egb_fleet/navigation_controller.hpp"
#include "egb_fleet/navigation_interface.hpp"
#include "egb_fleet/navigation_session.hpp"

#include <cmath>
#include <egb_fleet_msgs/msg/lane.hpp>
#include <egb_fleet_msgs/msg/waypoint.hpp>
#include <nlohmann/json.hpp>
#include <rmf_traffic/Time.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace egb_fleet {

namespace {

class DockNameInspector : public rmf_traffic::agv::Graph::Lane::Executor {
public:
  explicit DockNameInspector(std::string target_dock_name)
      : target_dock_name_(std::move(target_dock_name)) {}

  void execute(const rmf_traffic::agv::Graph::Lane::Dock &dock) override {
    if (dock.dock_name() == target_dock_name_) {
      match_ = true;
      dock_name_ = dock.dock_name();
    }
  }

  void execute(const rmf_traffic::agv::Graph::Lane::DoorOpen &) override {}
  void execute(const rmf_traffic::agv::Graph::Lane::DoorClose &) override {}
  void
  execute(const rmf_traffic::agv::Graph::Lane::LiftSessionBegin &) override {}
  void execute(const rmf_traffic::agv::Graph::Lane::LiftSessionEnd &) override {
  }
  void execute(const rmf_traffic::agv::Graph::Lane::LiftMove &) override {}
  void execute(const rmf_traffic::agv::Graph::Lane::LiftDoorOpen &) override {}
  void execute(const rmf_traffic::agv::Graph::Lane::Wait &) override {}

  bool has_match() const { return match_; }

private:
  std::string target_dock_name_;
  bool match_ = false;
  std::string dock_name_;
};

} // anonymous namespace

PathHandle::PathHandle(
    const std::string &robot_name,
    std::shared_ptr<NavigationInterface> control_adapter,
    std::shared_ptr<rclcpp::Node> node,
    const std::map<std::string, rmf_fleet_adapter::agv::Transformation>
        &coordinate_transforms,
    std::shared_ptr<const rmf_traffic::agv::Graph> graph)
    : robot_name_(robot_name), control_adapter_(control_adapter), node_(node),
      coordinate_transforms_(coordinate_transforms), graph_(graph) {
  RCLCPP_INFO(node_->get_logger(), "[%s] FullControlHandle created",
              robot_name_.c_str());
}

Eigen::Vector2d
PathHandle::transform_position(const Eigen::Vector2d &rmf_position) const {
  if (coordinate_transforms_.empty()) {
    RCLCPP_WARN_ONCE(node_->get_logger(),
                     "[%s] No coordinate transforms available",
                     robot_name_.c_str());
    return rmf_position;
  }

  const auto &transform = coordinate_transforms_.begin()->second;
  Eigen::Vector3d rmf_pose_3d(rmf_position.x(), rmf_position.y(), 0.0);
  Eigen::Vector3d robot_pose_3d = transform.apply(rmf_pose_3d);

  return robot_pose_3d.head<2>();
}

egb_fleet_msgs::msg::Waypoint
PathHandle::waypoint_to_msg(const rmf_traffic::agv::Plan::Waypoint &waypoint,
                            size_t waypoint_index,
                            size_t total_waypoints) const {
  egb_fleet_msgs::msg::Waypoint wp_msg;
  wp_msg.id = "wp_" + std::to_string(waypoint_index);
  wp_msg.sequence_id = static_cast<uint32_t>(waypoint_index);

  Eigen::Vector3d waypoint_pos_3d = waypoint.position();
  Eigen::Vector2d rmf_position(waypoint_pos_3d[0], waypoint_pos_3d[1]);
  Eigen::Vector2d robot_position = transform_position(rmf_position);

  wp_msg.pose.position.x = robot_position.x();
  wp_msg.pose.position.y = robot_position.y();
  wp_msg.pose.position.z = 0.0;

  double theta = waypoint_pos_3d[2];
  wp_msg.pose.orientation.x = 0.0;
  wp_msg.pose.orientation.y = 0.0;
  wp_msg.pose.orientation.z = std::sin(theta / 2.0);
  wp_msg.pose.orientation.w = std::cos(theta / 2.0);

  return wp_msg;
}

std::optional<size_t> PathHandle::find_best_approach_lane(
    const rmf_traffic::agv::Graph::Lane &docking_lane,
    size_t dock_waypoint_index) const {
  if (!graph_)
    return std::nullopt;

  size_t dock_entry_wp = docking_lane.entry().waypoint_index();
  size_t dock_exit_wp = docking_lane.exit().waypoint_index();

  const auto &entry_wp = graph_->get_waypoint(dock_entry_wp);
  const auto &exit_wp = graph_->get_waypoint(dock_exit_wp);

  Eigen::Vector2d dock_entry_pos(entry_wp.get_location()[0],
                                 entry_wp.get_location()[1]);
  Eigen::Vector2d dock_exit_pos(exit_wp.get_location()[0],
                                exit_wp.get_location()[1]);

  Eigen::Vector2d dock_direction =
      (dock_exit_pos - dock_entry_pos).normalized();
  double dock_angle = std::atan2(dock_direction.y(), dock_direction.x());

  double best_angle_diff = M_PI + 1.0;
  std::optional<size_t> best_lane;

  const auto &dock_wp = graph_->get_waypoint(dock_waypoint_index);
  Eigen::Vector2d dock_pos(dock_wp.get_location()[0],
                           dock_wp.get_location()[1]);

  for (size_t lane_idx = 0; lane_idx < graph_->num_lanes(); ++lane_idx) {
    const auto &lane = graph_->get_lane(lane_idx);

    if (lane.entry().waypoint_index() == dock_waypoint_index) {
      size_t app_exit_wp = lane.exit().waypoint_index();
      const auto &app_exit = graph_->get_waypoint(app_exit_wp);

      Eigen::Vector2d app_exit_pos(app_exit.get_location()[0],
                                   app_exit.get_location()[1]);
      Eigen::Vector2d app_direction = (app_exit_pos - dock_pos).normalized();
      double app_angle = std::atan2(app_direction.y(), app_direction.x());

      double angle_diff = app_angle - dock_angle;
      while (angle_diff > M_PI)
        angle_diff -= 2 * M_PI;
      while (angle_diff < -M_PI)
        angle_diff += 2 * M_PI;
      double dist_from_180 = std::abs(std::abs(angle_diff) - M_PI);

      if (dist_from_180 < best_angle_diff) {
        best_angle_diff = dist_from_180;
        best_lane = lane_idx;
      }
    }
  }

  if (best_lane) {
    RCLCPP_INFO(
        node_->get_logger(),
        "[%s] Selected best approach lane %zu (angle diff from 180: %.4f rad)",
        robot_name_.c_str(), *best_lane, best_angle_diff);
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "[%s] No approach lanes found from waypoint %zu",
                robot_name_.c_str(), dock_waypoint_index);
  }

  return best_lane;
}

// ---------------------------------------------------------------------------
// follow_new_path: creates a NavigationSession and hands it to the controller
// ---------------------------------------------------------------------------
void PathHandle::follow_new_path(
    const std::vector<rmf_traffic::agv::Plan::Waypoint> &waypoints,
    ArrivalEstimator next_arrival_estimator,
    RequestCompleted path_finished_callback) {
  if (waypoints.empty()) {
    RCLCPP_WARN(node_->get_logger(), "[%s] Received empty waypoint path",
                robot_name_.c_str());
    if (path_finished_callback)
      path_finished_callback();
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "[%s] Following path with %zu waypoints",
              robot_name_.c_str(), waypoints.size());

  // Convert waypoints to the internal format
  std::vector<egb_fleet_msgs::msg::Waypoint> wp_msgs;
  wp_msgs.reserve(waypoints.size());
  for (size_t i = 0; i < waypoints.size(); ++i) {
    wp_msgs.push_back(waypoint_to_msg(waypoints[i], i, waypoints.size()));
  }

  // Create a new session. The old session is invalidated below under its
  // mutex, which blocks until any in-flight feedback callback (which holds
  // that same mutex across the next_arrival_estimator call) completes.
  // This ensures no stale RMF callback runs after the plan is destroyed.
  auto new_session = std::make_shared<NavigationSession>();
  for (const auto &wp : waypoints) {
    new_session->waypoint_approach_lanes.push_back(wp.approach_lanes());
    new_session->waypoint_graph_indices.push_back(wp.graph_index());
  }

  // CRITICAL: Invalidate the old session FIRST, before doing anything else.
  // RMF may have already freed old plan state before calling follow_new_path.
  // By invalidating under lock, we block any in-flight Zenoh feedback callback
  // that is currently inside the old arrival_estimator, then null it out so
  // no future callback can call into freed plan data.
  {
    std::shared_ptr<NavigationSession> old_session;
    {
      std::lock_guard<std::mutex> lock(session_mutex_);
      old_session = session_;
      session_ = new_session;
    }
    if (old_session) {
      std::lock_guard<std::mutex> lock(old_session->mutex);
      old_session->invalidate();
    }
  }

  // Store the RMF arrival estimator in the session. It is ONLY called from
  // the timer/update thread (same executor as RMF), never from Zenoh threads.
  // The Zenoh feedback thread just buffers the data in pending_feedback.
  new_session->rmf_arrival_estimator = next_arrival_estimator;

  // The wrapped_estimator is called by NavigationController::feedback_callback
  // on the Zenoh thread. It must NOT call rmf_arrival_estimator — just buffer.
  // Stale-feedback rejection happens upstream via the goal_id check.
  std::function<void(size_t, double)> wrapped_estimator;
  if (next_arrival_estimator) {
    std::weak_ptr<NavigationSession> session_wp = new_session;
    wrapped_estimator = [session_wp](size_t waypoint_index, double seconds) {
      auto session = session_wp.lock();
      if (!session || session->done.load())
        return;
      session->current_target_index.store(waypoint_index);
      std::lock_guard<std::mutex> lock(session->mutex);
      session->pending_feedback =
          NavigationSession::PendingFeedback{waypoint_index, seconds};
    };
  }

  // Wrap completion callback: clear the session on completion.
  std::weak_ptr<NavigationSession> completion_wp = new_session;
  auto wrapped_completion = [this, path_finished_callback, completion_wp]() {
    if (auto s = completion_wp.lock()) {
      std::lock_guard<std::mutex> lock(s->mutex);
      s->invalidate();
    }
    {
      std::lock_guard<std::mutex> lock(session_mutex_);
      session_.reset();
    }
    if (path_finished_callback)
      path_finished_callback();
  };

  // Tell the controller about the new session
  if (auto nav =
          std::dynamic_pointer_cast<NavigationController>(control_adapter_)) {
    nav->set_session(new_session);
  }

  // Execute
  if (!control_adapter_) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Control adapter not initialized",
                 robot_name_.c_str());
    {
      std::lock_guard<std::mutex> lock(session_mutex_);
      session_.reset();
    }
    if (path_finished_callback)
      path_finished_callback();
    return;
  }

  control_adapter_->execute_path(wp_msgs, wrapped_completion,
                                 wrapped_estimator);
}

void PathHandle::stop() {
  RCLCPP_INFO(node_->get_logger(), "[%s] Stop requested", robot_name_.c_str());

  if (control_adapter_) {
    control_adapter_->cancel_navigation();
  }

  // Grab session, then release session_mutex_ before locking session->mutex.
  // Lock order: session->mutex before session_mutex_ (same as
  // wrapped_completion).
  std::shared_ptr<NavigationSession> s;
  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    s = session_;
  }
  if (s) {
    std::lock_guard<std::mutex> slock(s->mutex);
    s->invalidate();
  }
  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    session_.reset();
  }
}

void PathHandle::dock(const std::string &dock_name,
                      RequestCompleted docking_finished_callback) {
  RCLCPP_INFO(node_->get_logger(), "[%s] Dock requested: %s",
              robot_name_.c_str(), dock_name.c_str());

  if (!graph_) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] No graph available for docking",
                 robot_name_.c_str());
    if (docking_finished_callback)
      docking_finished_callback();
    return;
  }

  std::vector<std::size_t> nearby_lanes_snapshot;
  {
    std::lock_guard<std::mutex> lock(position_mutex_);
    nearby_lanes_snapshot = nearby_lanes_;
  }

  if (nearby_lanes_snapshot.empty()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[%s] No nearby lanes cached. Robot position not updated.",
                 robot_name_.c_str());
    if (docking_finished_callback)
      docking_finished_callback();
    return;
  }

  bool found = false;
  std::size_t dock_wp_index = 0;
  const rmf_traffic::agv::Graph::Lane *docking_lane = nullptr;

  for (std::size_t lane_idx : nearby_lanes_snapshot) {
    const auto &lane = graph_->get_lane(lane_idx);
    if (!found && lane.exit().event()) {
      DockNameInspector inspector(dock_name);
      lane.exit().event()->execute(inspector);
      if (inspector.has_match()) {
        dock_wp_index = lane.exit().waypoint_index();
        docking_lane = &lane;
        found = true;
        RCLCPP_INFO(node_->get_logger(),
                    "[%s] Found dock '%s' at waypoint %zu (exit of lane %zu)",
                    robot_name_.c_str(), dock_name.c_str(), dock_wp_index,
                    lane_idx);
        break;
      }
    }
  }

  if (!found) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] No nearby lane matches dock '%s'",
                 robot_name_.c_str(), dock_name.c_str());
    if (docking_finished_callback)
      docking_finished_callback();
    return;
  }

  bool has_undock_paths = false;

  RCLCPP_INFO(node_->get_logger(),
              "[%s] Checking for outgoing lanes from docking waypoint %zu",
              robot_name_.c_str(), dock_wp_index);

  for (size_t lane_idx = 0; lane_idx < graph_->num_lanes(); ++lane_idx) {
    const auto &lane = graph_->get_lane(lane_idx);
    if (lane.entry().waypoint_index() == dock_wp_index) {
      size_t lane_exit = lane.exit().waypoint_index();
      RCLCPP_INFO(node_->get_logger(),
                  "[%s] Found outgoing lane %zu from dock_wp %zu to %zu",
                  robot_name_.c_str(), lane_idx, dock_wp_index, lane_exit);

      size_t docking_lane_entry = docking_lane->entry().waypoint_index();
      if (lane_exit != docking_lane_entry) {
        RCLCPP_INFO(node_->get_logger(),
                    "[%s] Lane %zu leads to waypoint %zu (not back to docking "
                    "lane entry %zu) - has undock paths",
                    robot_name_.c_str(), lane_idx, lane_exit,
                    docking_lane_entry);
        has_undock_paths = true;
        break;
      }
    }
  }

  RCLCPP_INFO(node_->get_logger(),
              "[%s] Undock path check result: has_undock_paths=%s",
              robot_name_.c_str(), has_undock_paths ? "true" : "false");

  if (has_undock_paths) {
    RCLCPP_INFO(
        node_->get_logger(),
        "[%s] Docking lane is a leaf lane - performing UNDOCK operation",
        robot_name_.c_str());

    auto approach_lane_idx =
        find_best_approach_lane(*docking_lane, dock_wp_index);
    if (!approach_lane_idx) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[%s] No approach lane found for undocking",
                   robot_name_.c_str());
      if (docking_finished_callback)
        docking_finished_callback();
      return;
    }

    const auto &approach_lane = graph_->get_lane(*approach_lane_idx);
    std::vector<egb_fleet_msgs::msg::Waypoint> undock_path;

    const auto &dock_wp = graph_->get_waypoint(dock_wp_index);
    Eigen::Vector2d dock_pos(dock_wp.get_location()[0],
                             dock_wp.get_location()[1]);

    size_t docking_lane_entry = docking_lane->entry().waypoint_index();
    const auto &entry_wp = graph_->get_waypoint(docking_lane_entry);
    Eigen::Vector2d entry_pos(entry_wp.get_location()[0],
                              entry_wp.get_location()[1]);

    Eigen::Vector2d dock_direction = (dock_pos - entry_pos).normalized();
    double dock_yaw = std::atan2(dock_direction.y(), dock_direction.x());

    size_t app_entry_wp = approach_lane.entry().waypoint_index();
    const auto &app_entry = graph_->get_waypoint(app_entry_wp);
    Eigen::Vector2d app_entry_pos(app_entry.get_location()[0],
                                  app_entry.get_location()[1]);

    Eigen::Vector2d app_direction = (dock_pos - app_entry_pos).normalized();
    Eigen::Vector2d app_opposite = -app_direction;
    double app_opposite_yaw = std::atan2(app_opposite.y(), app_opposite.x());

    egb_fleet_msgs::msg::Waypoint dock_waypoint;
    dock_waypoint.id = "dock_" + dock_name;
    dock_waypoint.sequence_id = 0;
    Eigen::Vector2d dock_robot_pos = transform_position(dock_pos);
    dock_waypoint.pose.position.x = dock_robot_pos.x();
    dock_waypoint.pose.position.y = dock_robot_pos.y();
    dock_waypoint.pose.position.z = 0.0;
    dock_waypoint.pose.orientation.x = 0.0;
    dock_waypoint.pose.orientation.y = 0.0;
    dock_waypoint.pose.orientation.z = std::sin(dock_yaw / 2.0);
    dock_waypoint.pose.orientation.w = std::cos(dock_yaw / 2.0);
    undock_path.push_back(dock_waypoint);

    egb_fleet_msgs::msg::Waypoint app_entry_waypoint;
    app_entry_waypoint.id = "app_entry";
    app_entry_waypoint.sequence_id = 1;
    Eigen::Vector2d app_entry_robot_pos = transform_position(app_entry_pos);
    app_entry_waypoint.pose.position.x = app_entry_robot_pos.x();
    app_entry_waypoint.pose.position.y = app_entry_robot_pos.y();
    app_entry_waypoint.pose.position.z = 0.0;
    app_entry_waypoint.pose.orientation.x = 0.0;
    app_entry_waypoint.pose.orientation.y = 0.0;
    app_entry_waypoint.pose.orientation.z = std::sin(app_opposite_yaw / 2.0);
    app_entry_waypoint.pose.orientation.w = std::cos(app_opposite_yaw / 2.0);
    undock_path.push_back(app_entry_waypoint);

    if (!control_adapter_) {
      RCLCPP_ERROR(node_->get_logger(), "[%s] Control adapter not initialized",
                   robot_name_.c_str());
      if (docking_finished_callback)
        docking_finished_callback();
      return;
    }

    control_adapter_->execute_path(undock_path, docking_finished_callback,
                                   nullptr);

    RCLCPP_INFO(node_->get_logger(),
                "[%s] Undock navigation started with %zu waypoints",
                robot_name_.c_str(), undock_path.size());

  } else {
    RCLCPP_INFO(node_->get_logger(),
                "[%s] Docking lane is NOT a leaf lane - performing DOCK "
                "operation (no navigation)",
                robot_name_.c_str());

    if (dock_name.find("charge") != std::string::npos) {
      RCLCPP_INFO(
          node_->get_logger(),
          "[%s] Detected charger dock: %s - charging will occur while docked",
          robot_name_.c_str(), dock_name.c_str());
    }
    if (docking_finished_callback)
      docking_finished_callback();
  }
}

std::shared_ptr<NavigationSession> PathHandle::current_session() const {
  std::lock_guard<std::mutex> lock(session_mutex_);
  return session_;
}

void PathHandle::drain_pending_feedback() {
  auto session = current_session();
  if (!session || session->done.load())
    return;

  // Snapshot and clear pending feedback under lock
  std::optional<NavigationSession::PendingFeedback> feedback;
  NavigationSession::RmfArrivalEstimator estimator;
  {
    std::lock_guard<std::mutex> lock(session->mutex);
    if (!session->pending_feedback || !session->rmf_arrival_estimator)
      return;
    feedback = session->pending_feedback;
    estimator = session->rmf_arrival_estimator;
    session->pending_feedback.reset();
  }

  // Call RMF estimator OUTSIDE the lock but on the timer thread (same executor
  // as RMF)
  if (feedback && estimator) {
    estimator(feedback->waypoint_index,
              rmf_traffic::time::from_seconds(feedback->eta_seconds));
  }
}

std::optional<PathHandle::NavigationLaneInfo>
PathHandle::get_navigation_lanes() const {
  auto session = current_session();
  if (!session)
    return std::nullopt;

  std::size_t idx = session->current_target_index.load();
  if (idx >= session->waypoint_approach_lanes.size())
    return std::nullopt;

  NavigationLaneInfo info;
  info.lanes = session->waypoint_approach_lanes[idx];
  info.graph_index = session->waypoint_graph_indices[idx];

  if (info.lanes.empty() &&
      (idx + 1) < session->waypoint_approach_lanes.size()) {
    info.lanes = session->waypoint_approach_lanes[idx + 1];
  }

  return info;
}

void PathHandle::update_position(const Eigen::Vector3d &position,
                                 const std::vector<std::size_t> &lanes) {
  std::lock_guard<std::mutex> lock(position_mutex_);
  current_position_ = position;
  nearby_lanes_ = lanes;
}

void PathHandle::update_position(std::size_t waypoint_index,
                                 double orientation) {
  std::lock_guard<std::mutex> lock(position_mutex_);
  current_position_[0] = 0.0;
  current_position_[1] = 0.0;
  current_position_[2] = orientation;

  nearby_lanes_.clear();
  if (graph_) {
    for (std::size_t lane_idx = 0; lane_idx < graph_->num_lanes(); ++lane_idx) {
      const auto &lane = graph_->get_lane(lane_idx);
      if (lane.entry().waypoint_index() == waypoint_index ||
          lane.exit().waypoint_index() == waypoint_index) {
        nearby_lanes_.push_back(lane_idx);
      }
    }
  }

  RCLCPP_DEBUG(node_->get_logger(),
               "[%s] Updated position: on waypoint %zu, found %zu nearby lanes",
               robot_name_.c_str(), waypoint_index, nearby_lanes_.size());
}

} // namespace egb_fleet
