// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#include "egb_fleet/robot_discovery.hpp"
#include "egb_fleet/navigation_controller.hpp"
#include "egb_fleet/path_handle.hpp"
#include "egb_fleet/robot_state.hpp"

#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <regex>
#include <rmf_traffic/geometry/Circle.hpp>
#include <sstream>
#include <tf2_ros/buffer.h>
#include <thread>

namespace egb_fleet {

// Helper function for curl write callback
static size_t WriteCallback(void *contents, size_t size, size_t nmemb,
                            void *userp) {
  ((std::string *)userp)->append((char *)contents, size * nmemb);
  return size * nmemb;
}

RestRobotDiscovery::RestRobotDiscovery(
    std::shared_ptr<rclcpp::Node> node, void *zenoh_session,
    std::shared_ptr<rmf_fleet_adapter::agv::FleetUpdateHandle> fleet_handle,
    std::shared_ptr<const rmf_traffic::agv::Graph> graph,
    const YAML::Node &config, const YAML::Node &discovery_config,
    const YAML::Node &robot_template_config, const YAML::Node &plugins_config,
    const std::map<std::string, rmf_fleet_adapter::agv::Transformation>
        &coordinate_transforms)
    : node_(node), zenoh_session_(zenoh_session), fleet_handle_(fleet_handle),
      graph_(graph), config_(config), discovery_config_(discovery_config),
      robot_template_config_(robot_template_config),
      plugins_config_(plugins_config),
      coordinate_transforms_(coordinate_transforms) {
  // Parse configuration
  if (discovery_config["robot_name_pattern"]) {
    robot_name_pattern_ =
        discovery_config["robot_name_pattern"].as<std::string>();
  }

  if (discovery_config["discovery_frequency"]) {
    discovery_frequency_ = discovery_config["discovery_frequency"].as<double>();
  }

  if (discovery_config["rest_endpoint"]) {
    rest_endpoint_ = discovery_config["rest_endpoint"].as<std::string>();
  }

  // NOTE: This implementation has HTTP client stubbed
  // In production, use cpr library for REST API calls

  RCLCPP_INFO(node_->get_logger(),
              "RestRobotDiscovery initialized (pattern: %s, frequency: %.2f "
              "Hz, endpoint: %s)",
              robot_name_pattern_.c_str(), discovery_frequency_,
              rest_endpoint_.c_str());
}

std::map<std::string, std::shared_ptr<RobotState>>
RestRobotDiscovery::discover_robots() {
  // Check if enough time has passed since last discovery
  auto now = std::chrono::steady_clock::now();
  if (last_discovery_time_.time_since_epoch().count() > 0) {
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now - last_discovery_time_)
                          .count();

    double discovery_period_ms =
        1000.0 / discovery_frequency_; // Convert Hz to milliseconds
    if (elapsed_ms < discovery_period_ms) {
      // Not time yet, return existing robots
      return robots_;
    }
  }

  last_discovery_time_ = now;

  // Scan for robots via REST API
  auto discovered_names = scan_for_robots_via_rest();

  // Add new robots that we haven't seen before
  for (const auto &robot_name : discovered_names) {
    if (robots_.find(robot_name) == robots_.end()) {
      RCLCPP_INFO(node_->get_logger(), "Discovered new robot: %s",
                  robot_name.c_str());

      if (add_dynamic_robot(robot_name)) {
        RCLCPP_INFO(node_->get_logger(), "Successfully added robot: %s",
                    robot_name.c_str());
      } else {
        RCLCPP_WARN(node_->get_logger(), "Failed to add robot: %s",
                    robot_name.c_str());
      }
    }
  }

  return robots_;
}

std::set<std::string> RestRobotDiscovery::scan_for_robots_via_rest() {
  std::set<std::string> discovered;

  // Build URL: endpoint + "/@/*/ros2/route/**/tf"
  std::string url = rest_endpoint_ + "/@/*/ros2/route/**/tf";

  RCLCPP_DEBUG(node_->get_logger(), "Scanning for robots via REST API: %s",
               url.c_str());

  // Initialize CURL
  CURL *curl = curl_easy_init();
  if (!curl) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize CURL");
    return discovered;
  }

  std::string response_data;
  CURLcode res;

  // Set CURL options
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_data);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L); // 5 second timeout

  // Perform the request
  res = curl_easy_perform(curl);

  if (res != CURLE_OK) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(),
                         10000, // Log once every 10 seconds
                         "REST API request failed: %s",
                         curl_easy_strerror(res));
    curl_easy_cleanup(curl);
    return discovered;
  }

  // Check HTTP response code
  long http_code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
  curl_easy_cleanup(curl);

  if (http_code != 200) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                         "REST API returned HTTP %ld", http_code);
    return discovered;
  }

  // Parse JSON response
  try {
    auto data = nlohmann::json::parse(response_data);

    // Compile regex pattern for robot name matching
    std::regex pattern(robot_name_pattern_);

    // Iterate through JSON array
    for (const auto &item : data) {
      if (item.contains("value") && item["value"].contains("zenoh_key_expr")) {
        std::string key_expr = item["value"]["zenoh_key_expr"];

        // Extract robot name from key_expr (first part before /)
        // Example: "robot_123/tf" -> "robot_123"
        size_t slash_pos = key_expr.find('/');
        if (slash_pos != std::string::npos) {
          std::string potential_name = key_expr.substr(0, slash_pos);

          // Check if name matches pattern
          if (std::regex_match(potential_name, pattern)) {
            discovered.insert(potential_name);
            RCLCPP_DEBUG(node_->get_logger(),
                         "Found robot matching pattern: %s",
                         potential_name.c_str());
          }
        }
      }
    }

    RCLCPP_DEBUG(node_->get_logger(), "REST scan complete: found %zu robots",
                 discovered.size());

  } catch (const nlohmann::json::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to parse REST API response: %s",
                 e.what());
  }

  return discovered;
}

bool RestRobotDiscovery::add_dynamic_robot(const std::string &robot_name) {
  try {
    RCLCPP_INFO(node_->get_logger(), "Adding dynamic robot: %s",
                robot_name.c_str());

    // 1. Create robot config from template
    YAML::Node robot_config = robot_template_config_;
    robot_config["name"] = robot_name;

    // 2. Create RobotState with plugins config
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_buffer->setUsingDedicatedThread(
        true); // Zenoh callback thread populates TF data

    // Get linear velocity from fleet config
    double linear_velocity =
        config_["rmf_fleet"]["limits"]["linear"][0].as<double>();

    // Create RobotState (command_handle will be set later after creation)
    auto adapter = std::make_shared<RobotState>(
        robot_name, robot_config, plugins_config_, node_, zenoh_session_,
        fleet_handle_, tf_buffer, linear_velocity,
        nullptr // command_handle will be created below
    );

    // Set graph and coordinate transforms for undocking
    adapter->set_graph_and_transforms(graph_, coordinate_transforms_);

    // 3. Get initial robot pose - retry a few times to allow TF data to arrive
    std::optional<Eigen::Vector3d> initial_pose;
    int max_retries = 10;
    for (int i = 0; i < max_retries && !initial_pose; ++i) {
      initial_pose = adapter->get_pose();
      if (!initial_pose) {
        RCLCPP_DEBUG(node_->get_logger(),
                     "[%s] Waiting for TF data... (attempt %d/%d)",
                     robot_name.c_str(), i + 1, max_retries);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }

    if (!initial_pose) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Could not get initial pose for robot %s after %d attempts. "
                   "TF data may not be available yet. Will retry on next "
                   "discovery cycle.",
                   robot_name.c_str(), max_retries);
      return false;
    }

    RCLCPP_INFO(node_->get_logger(),
                "[%s] Got initial pose: (%.2f, %.2f, %.2f)", robot_name.c_str(),
                (*initial_pose)[0], (*initial_pose)[1], (*initial_pose)[2]);

    // 4. Create MPC Control Adapter for navigation
    auto control_adapter = std::make_shared<NavigationController>(
        robot_name, node_, zenoh_session_, tf_buffer);

    // 5. Create PathHandle for this robot
    std::string map_name = adapter->get_map_name();

    auto command_handle = std::make_shared<PathHandle>(
        robot_name, control_adapter, node_, coordinate_transforms_, graph_);

    // Set command handle on the adapter for docking support
    adapter->set_command_handle(command_handle);
    command_handles_[robot_name] = command_handle;

    // Set navigation interface for action execution (enables undocking)
    adapter->set_navigation_interface(control_adapter);

    // 6. Get vehicle profile from config (use the rmf_fleet profile, not
    // robot_template)
    RCLCPP_DEBUG(node_->get_logger(), "[%s] Reading profile from config",
                 robot_name.c_str());

    double footprint_radius;
    double vicinity_radius;

    try {
      footprint_radius =
          config_["rmf_fleet"]["profile"]["footprint"].as<double>();
      RCLCPP_DEBUG(node_->get_logger(), "[%s] Footprint radius: %.2f",
                   robot_name.c_str(), footprint_radius);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(node_->get_logger(), "[%s] Failed to read footprint: %s",
                   robot_name.c_str(), e.what());
      throw;
    }

    vicinity_radius = footprint_radius * 2.0; // Default to 2x footprint
    if (config_["rmf_fleet"]["profile"]["vicinity"]) {
      try {
        vicinity_radius =
            config_["rmf_fleet"]["profile"]["vicinity"].as<double>();
        RCLCPP_DEBUG(node_->get_logger(), "[%s] Vicinity radius: %.2f",
                     robot_name.c_str(), vicinity_radius);
      } catch (const std::exception &e) {
        RCLCPP_WARN(node_->get_logger(),
                    "[%s] Failed to read vicinity, using default: %s",
                    robot_name.c_str(), e.what());
      }
    }

    RCLCPP_DEBUG(node_->get_logger(), "[%s] Creating geometry shapes",
                 robot_name.c_str());
    auto footprint_shape =
        std::make_shared<rmf_traffic::geometry::FinalConvexShape>(
            rmf_traffic::geometry::Circle(footprint_radius).finalize_convex());
    auto vicinity_shape =
        std::make_shared<rmf_traffic::geometry::FinalConvexShape>(
            rmf_traffic::geometry::Circle(vicinity_radius).finalize_convex());

    RCLCPP_DEBUG(node_->get_logger(), "[%s] Creating profile",
                 robot_name.c_str());
    auto profile = rmf_traffic::Profile(footprint_shape, vicinity_shape);

    // 6. Transform robot coordinates to RMF coordinates
    Eigen::Vector3d rmf_pose;
    if (coordinate_transforms_.find(map_name) != coordinate_transforms_.end()) {
      const auto &transform = coordinate_transforms_.at(map_name);
      // Apply inverse transformation: robot coords -> RMF coords
      rmf_pose = transform.apply_inverse(*initial_pose);

      RCLCPP_INFO(node_->get_logger(),
                  "[%s] Transformed pose - Robot: (%.2f, %.2f, %.2f) -> RMF: "
                  "(%.2f, %.2f, %.2f)",
                  robot_name.c_str(), (*initial_pose)[0], (*initial_pose)[1],
                  (*initial_pose)[2], rmf_pose[0], rmf_pose[1], rmf_pose[2]);
    } else {
      RCLCPP_ERROR(node_->get_logger(),
                   "[%s] No coordinate transformation found for map [%s]. "
                   "Available maps: [",
                   robot_name.c_str(), map_name.c_str());
      for (const auto &[map, _] : coordinate_transforms_) {
        RCLCPP_ERROR(node_->get_logger(), "  %s", map.c_str());
      }
      RCLCPP_ERROR(node_->get_logger(), "]");
      return false;
    }

    // 7. Compute starting waypoints from robot's current position
    auto starts = rmf_traffic::agv::compute_plan_starts(
        *graph_, map_name,
        rmf_pose, // Use transformed RMF coordinates
        rmf_traffic::Time(rmf_traffic::Duration(node_->now().nanoseconds())),
        1.0, // max_merge_waypoint_distance
        1.0  // max_merge_lane_distance
    );

    RCLCPP_DEBUG(node_->get_logger(), "[%s] Computed %zu plan starts",
                 robot_name.c_str(), starts.size());

    // Check if we got any valid starts
    if (starts.empty()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Failed to find starting waypoint for robot [%s] at RMF "
                   "pose (%.2f, %.2f, %.2f) on map [%s]. "
                   "The robot is too far from any waypoint (max distances: "
                   "1.0m from waypoint, 1.0m from lane). "
                   "Graph has %zu waypoints.",
                   robot_name.c_str(), rmf_pose[0], rmf_pose[1], rmf_pose[2],
                   map_name.c_str(), graph_->num_waypoints());
      return false;
    }

    // 7. Register robot with RMF fleet
    RCLCPP_DEBUG(node_->get_logger(), "[%s] Registering robot with RMF fleet",
                 robot_name.c_str());
    fleet_handle_->add_robot(
        command_handle, robot_name, profile, starts,
        [adapter, robot_name, node = node_](
            std::shared_ptr<rmf_fleet_adapter::agv::RobotUpdateHandle> handle) {
          // Store update handle - needed for update_position() calls
          adapter->set_update_handle(handle);

          // Register action executor to receive ActionExecution for task
          // actions
          handle->set_action_executor(
              [adapter, robot_name](
                  const std::string &category,
                  const nlohmann::json &description,
                  rmf_fleet_adapter::agv::RobotUpdateHandle::ActionExecution
                      execution) {
                RCLCPP_INFO(rclcpp::get_logger("egb_fleet_adapter"),
                            "[%s] ===== ACTION EXECUTOR CALLED ===== "
                            "category=%s, description=%s",
                            adapter->name().c_str(), category.c_str(),
                            description.dump().c_str());

                // Execute action through adapter (passes full ActionExecution)
                adapter->execute_action(category, description,
                                        std::move(execution));
              });

          RCLCPP_INFO(node->get_logger(),
                      "Dynamic robot %s registered with RMF, update handle and "
                      "action executor stored",
                      robot_name.c_str());
        });

    // Store the adapter and command handle - MUST keep command_handle alive!
    robots_[robot_name] = adapter;
    command_handles_[robot_name] = command_handle;

    RCLCPP_INFO(
        node_->get_logger(),
        "Successfully added dynamic robot: %s on map %s at (%.2f, %.2f, %.2f)",
        robot_name.c_str(), map_name.c_str(), (*initial_pose)[0],
        (*initial_pose)[1], (*initial_pose)[2]);

    return true;

  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to add dynamic robot %s: %s",
                 robot_name.c_str(), e.what());
    return false;
  }
}

} // namespace egb_fleet
