// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#include "egb_fleet/robot_state.hpp"
#include "egb_fleet/action/robot_action_context.hpp"
#include "egb_fleet/action/robot_action_factory.hpp"
#include "egb_fleet/navigation_interface.hpp"
#include "egb_fleet/navigation_session.hpp"
#include "egb_fleet/path_handle.hpp"
#include "egb_fleet/utils/zenoh_helpers.hpp"
#include <cstring>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <zenoh.hxx>

namespace egb_fleet {

RobotState::RobotState(
    const std::string &name, const YAML::Node &robot_config,
    const YAML::Node &plugins_config, std::shared_ptr<rclcpp::Node> node,
    void *zenoh_session, std::shared_ptr<FleetUpdateHandle> fleet_handle,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer, double linear_velocity,
    std::shared_ptr<rmf_fleet_adapter::agv::RobotCommandHandle> command_handle)
    : RobotHandle(name, node, fleet_handle), robot_config_(robot_config),
      plugins_config_(plugins_config), zenoh_session_(zenoh_session),
      tf_buffer_(tf_buffer), linear_velocity_(linear_velocity),
      command_handle_(command_handle) {
  // Parse configuration from robot_config
  map_frame_ = robot_config["map_frame"]
                   ? robot_config["map_frame"].as<std::string>()
                   : "map";
  robot_frame_ = robot_config["robot_frame"]
                     ? robot_config["robot_frame"].as<std::string>()
                     : "base_link";
  map_name_ = robot_config["initial_map"]
                  ? robot_config["initial_map"].as<std::string>()
                  : "map";

  // Create TF handler
  tf_handler_ = std::make_unique<TfHandler>(name, zenoh_session, tf_buffer,
                                            node, robot_frame_, map_frame_);

  // Subscribe to battery state via Zenoh
  if (zenoh_session_) {
    std::string battery_topic = utils::namespacify(name, "battery_state");
    auto *session = static_cast<zenoh::Session *>(zenoh_session_);

    try {
      auto battery_callback = [this](const zenoh::Sample &sample) {
        auto payload_str = sample.get_payload().as_string();
        std::vector<uint8_t> data(payload_str.begin(), payload_str.end());
        this->battery_state_callback(data);
      };

      auto subscriber = session->declare_subscriber(
          battery_topic, battery_callback, zenoh::closures::none);

      battery_state_subscriber_ =
          new zenoh::Subscriber<void>(std::move(subscriber));

      RCLCPP_INFO(node_->get_logger(), "[%s] Subscribed to battery state: %s",
                  name_.c_str(), battery_topic.c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[%s] Failed to subscribe to battery state: %s",
                   name_.c_str(), e.what());
    }
  }

  // NOTE: Plugin loading is deferred until we have a RobotUpdateHandle
  // (which is set after the robot is added to the fleet via add_robot call)
  // For now, just build the action-to-plugin mapping
  if (plugins_config) {
    for (const auto &plugin : plugins_config) {
      std::string plugin_name = plugin.first.as<std::string>();
      YAML::Node plugin_cfg = plugin.second;

      if (plugin_cfg["actions"]) {
        for (const auto &action : plugin_cfg["actions"]) {
          std::string action_name = action.as<std::string>();
          action_to_plugin_name_[action_name] = plugin_name;
        }
      }
    }
  }

  RCLCPP_INFO(node_->get_logger(), "[%s] RobotState created", name_.c_str());
}

RobotState::~RobotState() {
  if (battery_state_subscriber_) {
    auto *subscriber =
        static_cast<zenoh::Subscriber<void> *>(battery_state_subscriber_);
    delete subscriber;
    battery_state_subscriber_ = nullptr;
  }
}

double RobotState::get_battery_soc() const {
  // Return cached battery SOC
  // In production, this would be updated by a Zenoh subscriber to
  // /{robot_name}/battery_state
  return battery_soc_.load();
}

std::string RobotState::get_map_name() const { return map_name_; }

std::optional<Eigen::Vector3d> RobotState::get_pose() const {
  if (!tf_handler_) {
    return std::nullopt;
  }

  auto transform = tf_handler_->get_transform();
  if (!transform) {
    return std::nullopt;
  }

  // Extract position
  double x = transform->transform.translation.x;
  double y = transform->transform.translation.y;

  // Extract yaw from quaternion
  tf2::Quaternion q(
      transform->transform.rotation.x, transform->transform.rotation.y,
      transform->transform.rotation.z, transform->transform.rotation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  return Eigen::Vector3d(x, y, yaw);
}

std::shared_ptr<action::RobotActionContext>
RobotState::get_or_create_action_context() {
  if (!action_context_) {
    // Lazily create action context with all required callbacks and state
    action_context_ = std::make_shared<action::RobotActionContext>(
        node_, name_,
        nullptr, // update_handle will be set by set_update_handle if robot is
                 // added
        YAML::Node(), // action_config (empty for now)
        [this]() { return this->get_battery_soc(); },
        [this]() { return this->get_map_name(); },
        [this]() { return this->get_pose(); }, zenoh_session_, graph_,
        std::make_shared<const std::map<
            std::string, rmf_fleet_adapter::agv::Transformation>>(
            coordinate_transforms_),
        // execute_navigation callback: convert PoseStamped to waypoints
        [this](const std::vector<geometry_msgs::msg::PoseStamped> &poses,
               std::function<void()> completion_callback) {
          if (!navigation_interface_) {
            RCLCPP_ERROR(node_->get_logger(),
                         "[%s] execute_navigation called but "
                         "navigation_interface is not set",
                         name_.c_str());
            return;
          }

          // Convert PoseStamped to waypoints format
          std::vector<egb_fleet_msgs::msg::Waypoint> waypoints;
          for (size_t i = 0; i < poses.size(); ++i) {
            const auto &pose = poses[i];
            egb_fleet_msgs::msg::Waypoint wp;
            wp.id = "wp_" + std::to_string(i);
            wp.sequence_id = static_cast<uint32_t>(i);
            wp.pose.position.x = pose.pose.position.x;
            wp.pose.position.y = pose.pose.position.y;
            wp.pose.position.z = 0.0;
            wp.pose.orientation = pose.pose.orientation;

            waypoints.push_back(wp);
          }

          // Execute navigation through the navigation interface
          navigation_interface_->execute_path(waypoints, completion_callback);
        },
        // cancel_navigation callback
        [this]() {
          if (navigation_interface_) {
            navigation_interface_->cancel_navigation();
          }
        });
  }
  return action_context_;
}

void RobotState::execute_action(
    const std::string &category, const nlohmann::json &description,
    rmf_fleet_adapter::agv::RobotUpdateHandle::ActionExecution execution) {
  RCLCPP_INFO(node_->get_logger(), "[%s] Executing action: category=%s",
              name_.c_str(), category.c_str());

  // Extract identifier before storing execution (since we'll move execution)
  auto activity_id = execution.identifier();

  // Check if action is registered (action_to_plugin_name_ is only written at
  // construction time, so no lock needed for the read)
  auto it = action_to_plugin_name_.find(category);
  if (it == action_to_plugin_name_.end()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[%s] No plugin registered for action category: %s",
                 name_.c_str(), category.c_str());
    return;
  }

  std::string plugin_name = it->second;

  try {
    // Lazily initialize the plugin loader if not already done
    if (!action_loader_) {
      // ClassLoader's first arg is the ament_index resource-prefix package
      // name. rules_ros2's ros2_plugin derives this from the first ::-segment
      // of the C++ class type ("egb_fleet" here), not from the Bazel target
      // name. Keep this aligned with the namespace.
      action_loader_ =
          std::make_unique<pluginlib::ClassLoader<action::RobotActionFactory>>(
              "egb_fleet", "egb_fleet::action::RobotActionFactory");
    }

    // Check if we've already loaded this plugin factory
    auto factory_it = loaded_factories_.find(plugin_name);
    std::shared_ptr<action::RobotActionFactory> factory;

    if (factory_it != loaded_factories_.end()) {
      factory = factory_it->second;
    } else {
      // Load the plugin dynamically
      RCLCPP_INFO(node_->get_logger(), "[%s] Loading action plugin: %s",
                  name_.c_str(), plugin_name.c_str());

      factory = action_loader_->createSharedInstance(plugin_name);
      if (!factory) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] Failed to load plugin: %s",
                     name_.c_str(), plugin_name.c_str());
        return;
      }

      // Initialize the factory with the action context
      auto context = get_or_create_action_context();
      factory->initialize(context);

      // Cache the factory for future use
      loaded_factories_[plugin_name] = factory;

      RCLCPP_INFO(node_->get_logger(), "[%s] Successfully loaded plugin: %s",
                  name_.c_str(), plugin_name.c_str());
    }

    // Use the factory to perform the action
    if (factory->supports_action(category)) {
      RCLCPP_INFO(node_->get_logger(), "[%s] Executing action via plugin: %s",
                  name_.c_str(), category.c_str());

      auto action =
          factory->perform_action(category, description, *activity_id);
      if (!action) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[%s] Plugin failed to create action for category: %s",
                     name_.c_str(), category.c_str());
        return;
      }

      // Store action and execution under lock — these are read by the
      // timer thread in update_current_action()
      {
        std::lock_guard<std::mutex> lock(action_mutex_);
        current_action_execution_ = std::move(execution);
        current_action_ = action;
      }

      RCLCPP_INFO(node_->get_logger(), "[%s] Action stored for polling: %s",
                  name_.c_str(), category.c_str());
    } else {
      RCLCPP_ERROR(node_->get_logger(),
                   "[%s] Plugin %s does not support action category: %s",
                   name_.c_str(), plugin_name.c_str(), category.c_str());
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[%s] Exception while executing action %s: %s", name_.c_str(),
                 category.c_str(), e.what());
  }
}

void RobotState::update_current_action() {
  // Snapshot the action under lock — update_action() may block on Zenoh I/O
  // so we must not hold the lock during the call
  std::shared_ptr<action::RobotAction> action;
  {
    std::lock_guard<std::mutex> lock(action_mutex_);
    action = current_action_;
  }

  if (!action) {
    return;
  }

  auto state = action->update_action();

  switch (state) {
  case action::RobotActionState::IN_PROGRESS:
  case action::RobotActionState::CANCELING:
    // Action is still running, continue monitoring
    break;

  case action::RobotActionState::COMPLETED:
    RCLCPP_INFO(node_->get_logger(), "[%s] Action completed successfully",
                name_.c_str());
    {
      std::lock_guard<std::mutex> lock(action_mutex_);
      if (current_action_execution_) {
        current_action_execution_->finished();
        current_action_execution_.reset();
      }
      current_action_.reset();
    }
    break;

  case action::RobotActionState::CANCELED:
    RCLCPP_INFO(node_->get_logger(), "[%s] Action was canceled", name_.c_str());
    {
      std::lock_guard<std::mutex> lock(action_mutex_);
      if (current_action_execution_) {
        current_action_execution_->finished();
        current_action_execution_.reset();
      }
      current_action_.reset();
    }
    break;

  case action::RobotActionState::FAILED:
    RCLCPP_ERROR(node_->get_logger(), "[%s] Action failed", name_.c_str());
    {
      std::lock_guard<std::mutex> lock(action_mutex_);
      if (current_action_execution_) {
        current_action_execution_->error("Action failed");
        current_action_execution_.reset();
      }
      current_action_.reset();
    }
    break;
  }
}

void RobotState::set_graph_and_transforms(
    std::shared_ptr<const rmf_traffic::agv::Graph> graph,
    const std::map<std::string, rmf_fleet_adapter::agv::Transformation>
        &transforms) {
  graph_ = graph;
  coordinate_transforms_ = transforms;
}

void RobotState::set_command_handle(
    std::shared_ptr<rmf_fleet_adapter::agv::RobotCommandHandle>
        command_handle) {
  command_handle_ = command_handle;
}

void RobotState::set_navigation_interface(
    std::shared_ptr<NavigationInterface> navigation_interface) {
  navigation_interface_ = navigation_interface;
}

void RobotState::battery_state_callback(const std::vector<uint8_t> &payload) {
  try {
    // Deserialize BatteryState message from CDR payload
    rclcpp::SerializedMessage serialized_msg(payload.size());
    std::memcpy(serialized_msg.get_rcl_serialized_message().buffer,
                payload.data(), payload.size());
    serialized_msg.get_rcl_serialized_message().buffer_length = payload.size();

    rclcpp::Serialization<sensor_msgs::msg::BatteryState> serializer;
    sensor_msgs::msg::BatteryState battery_state;
    serializer.deserialize_message(&serialized_msg, &battery_state);

    // Update battery SOC (percentage as 0.0-1.0)
    battery_soc_.store(battery_state.percentage);

    RCLCPP_DEBUG(node_->get_logger(), "[%s] Battery state updated: %.1f%%",
                 name_.c_str(), battery_state.percentage * 100.0);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[%s] Error deserializing battery state: %s", name_.c_str(),
                 e.what());
  }
}

bool RobotState::update(
    std::shared_ptr<const rmf_traffic::agv::Graph> /* graph */) {
  try {
    // Get current robot state
    auto pose = get_pose();
    auto battery_soc = get_battery_soc();
    auto map_name = get_map_name();

    if (!update_handle_) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                            "[%s] No update handle available", name_.c_str());
      return false;
    }

    if (!pose) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                            "[%s] No pose available from TF", name_.c_str());
      return false;
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "[%s] Update: pose=(%.2f, %.2f, %.2f), battery=%.1f%%",
                         name_.c_str(), (*pose)[0], (*pose)[1], (*pose)[2],
                         battery_soc * 100.0);

    // Transform robot coordinates to RMF coordinates
    Eigen::Vector3d rmf_pose;
    if (coordinate_transforms_.find(map_name) != coordinate_transforms_.end()) {
      const auto &transform = coordinate_transforms_.at(map_name);
      rmf_pose = transform.apply_inverse(*pose);

      RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 5000,
          "[%s] Transformed pose: robot(%.2f, %.2f) -> RMF(%.2f, %.2f)",
          name_.c_str(), (*pose)[0], (*pose)[1], rmf_pose[0], rmf_pose[1]);
    } else {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                            "[%s] No coordinate transform for map '%s'",
                            name_.c_str(), map_name.c_str());
      return false;
    }

    // Nav-graph merge radii. RMF defaults the waypoint one to 0.1m.
    const double max_merge_waypoint_distance = 1.0;
    const double max_merge_lane_distance = 1.0;

    // Update position: use lane-aware overload during navigation, fallback to
    // map-based
    bool used_lane_update = false;
    if (auto path_handle =
            std::dynamic_pointer_cast<PathHandle>(command_handle_)) {
      auto nav_lanes = path_handle->get_navigation_lanes();
      if (nav_lanes.has_value()) {
        if (!nav_lanes->lanes.empty()) {
          update_handle_->update_position(rmf_pose, nav_lanes->lanes);
          used_lane_update = true;
        } else if (nav_lanes->graph_index.has_value()) {
          update_handle_->update_position(*nav_lanes->graph_index, rmf_pose[2]);
          used_lane_update = true;
        }
      }
    }
    if (!used_lane_update) {
      update_handle_->update_position(map_name, rmf_pose,
                                      max_merge_waypoint_distance,
                                      max_merge_lane_distance);
    }

    // Update command handle with position and nearby lanes for docking
    if (command_handle_ && graph_) {
      auto starts = rmf_traffic::agv::compute_plan_starts(
          *graph_, map_name, rmf_pose,
          rmf_traffic::Time(rmf_traffic::Duration(node_->now().nanoseconds())),
          max_merge_waypoint_distance, max_merge_lane_distance);

      // Extract lane indices from starts
      std::vector<std::size_t> lanes;
      for (const auto &start : starts) {
        if (start.lane()) {
          lanes.push_back(start.lane().value());
        }
      }

      // If no lanes found, use all lanes in graph (robot might be exactly on
      // waypoint)
      if (lanes.empty()) {
        for (std::size_t i = 0; i < graph_->num_lanes(); ++i) {
          lanes.push_back(i);
        }
        RCLCPP_DEBUG(node_->get_logger(),
                     "[%s] No nearby lanes from compute_plan_starts, using all "
                     "%zu lanes",
                     name_.c_str(), lanes.size());
      }

      // Update command handle with position and lanes, and drain buffered
      // feedback
      if (auto path_handle =
              std::dynamic_pointer_cast<PathHandle>(command_handle_)) {
        path_handle->update_position(rmf_pose, lanes);
        path_handle->drain_pending_feedback();
        if (path_handle->take_pending_failure()) {
          RCLCPP_WARN(node_->get_logger(),
                      "[%s] Navigation failed, requesting replan",
                      name_.c_str());
          update_handle_->replan();
        }
      }

      RCLCPP_DEBUG(node_->get_logger(),
                   "[%s] Command handle: YES, Graph: YES, Lanes: %zu",
                   name_.c_str(), lanes.size());
    } else {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 5000,
          "[%s] Cannot update command handle - Command handle: %s, Graph: %s",
          name_.c_str(), command_handle_ ? "YES" : "NO", graph_ ? "YES" : "NO");
    }

    // Update battery SOC
    update_handle_->update_battery_soc(battery_soc);

    check_deadlock_recovery(map_name, rmf_pose);

    RCLCPP_DEBUG(node_->get_logger(),
                 "[%s] Successfully updated position and battery to RMF",
                 name_.c_str());

    return true;

  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Update error: %s", name_.c_str(),
                 e.what());
    return false;
  }
}

void RobotState::check_deadlock_recovery(const std::string &map_name,
                                         const Eigen::Vector3d &rmf_pose) {
  const auto now = node_->now();

  if (holding_) {
    if (hold_until_ && now >= *hold_until_) {
      if (deadlock_interruption_)
        deadlock_interruption_->resume({"goal_on_obstacle_cleared"});
      deadlock_interruption_.reset();
      holding_ = false;
      hold_until_.reset();
      blocked_since_.reset();
      cooldown_until_ = now + rclcpp::Duration::from_seconds(15.0);
    }
    return;
  }

  auto path = std::dynamic_pointer_cast<PathHandle>(command_handle_);
  auto session = path ? path->current_session() : nullptr;
  const bool blocked = session && !session->done.load() &&
                       session->failure_reason.load() != NavFailure::None;
  if (!blocked) {
    blocked_since_.reset();
    return;
  }
  if (cooldown_until_ && now < *cooldown_until_)
    return;
  if (!blocked_since_) {
    blocked_since_ = now;
    return;
  }
  if ((now - *blocked_since_).seconds() < 8.0)
    return;

  auto handle = update_handle_;
  const std::string map = map_name;
  const Eigen::Vector3d pose = rmf_pose;
  deadlock_interruption_ =
      handle->interrupt({"goal_on_obstacle"}, [handle, map, pose]() {
        handle->unstable().declare_holding(map, pose, std::chrono::seconds(10));
      });
  holding_ = true;
  hold_until_ = now + rclcpp::Duration::from_seconds(10.0);
  RCLCPP_WARN(node_->get_logger(),
              "[%s] goal-on-obstacle deadlock: interrupt + hold 10s",
              name_.c_str());
}

} // namespace egb_fleet
