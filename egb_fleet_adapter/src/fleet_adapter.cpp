// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#include "egb_fleet/fleet_adapter.hpp"
#include "egb_fleet/navigation_controller.hpp"
#include "egb_fleet/path_handle.hpp"
#include "egb_fleet/robot_discovery.hpp"
#include "egb_fleet/robot_state.hpp"
#include "egb_fleet/utils/coordinate_transform.hpp"
#include <rmf_fleet_adapter/agv/parse_graph.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <zenoh.hxx>

#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>

#include <curl/curl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <chrono>
#include <unordered_set>

namespace egb_fleet {

namespace {

size_t curl_append(char *ptr, size_t size, size_t nmemb, void *userdata) {
  static_cast<std::string *>(userdata)->append(ptr, size * nmemb);
  return size * nmemb;
}

std::string http_get(const std::string &url, double timeout_s) {
  CURL *curl = curl_easy_init();
  if (!curl)
    throw std::runtime_error("curl_easy_init failed");
  std::string body;
  long code = 0;
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_append);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &body);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS,
                   static_cast<long>(timeout_s * 1000.0));
  const CURLcode rc = curl_easy_perform(curl);
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &code);
  curl_easy_cleanup(curl);
  if (rc != CURLE_OK)
    throw std::runtime_error(std::string("nav graph GET failed: ") +
                             curl_easy_strerror(rc));
  if (code != 200)
    throw std::runtime_error("nav graph GET returned HTTP " +
                             std::to_string(code));
  return body;
}

// Parse YAML text with RMF's canonical parse_graph via an anonymous in-memory
// fd, so nothing is written to disk.
rmf_traffic::agv::Graph
parse_graph_text(const std::string &yaml,
                 const rmf_traffic::agv::VehicleTraits &traits) {
  int fd = memfd_create("navgraph", MFD_CLOEXEC);
  if (fd < 0)
    throw std::runtime_error("memfd_create failed");
  const char *p = yaml.data();
  size_t remaining = yaml.size();
  while (remaining > 0) {
    const ssize_t n = ::write(fd, p, remaining);
    if (n <= 0) {
      ::close(fd);
      throw std::runtime_error("failed writing nav graph to memfd");
    }
    p += n;
    remaining -= static_cast<size_t>(n);
  }
  try {
    auto graph = rmf_fleet_adapter::agv::parse_graph(
        "/proc/self/fd/" + std::to_string(fd), traits);
    ::close(fd);
    return graph;
  } catch (...) {
    ::close(fd);
    throw;
  }
}

} // namespace

FleetAdapter::FleetAdapter(const std::string &config_path)
    : nav_graph_id_("0"), zenoh_session_(nullptr) {
  // Load configuration
  config_ = YAML::LoadFile(config_path);
  if (config_["rmf_fleet"]["nav_graph_id"]) {
    nav_graph_id_ = config_["rmf_fleet"]["nav_graph_id"].as<std::string>();
  }
  if (config_["rmf_fleet"]["nav_graph_url"]) {
    nav_graph_url_ = config_["rmf_fleet"]["nav_graph_url"].as<std::string>();
  }
  if (config_["rmf_fleet"]["building_id"]) {
    building_id_ = config_["rmf_fleet"]["building_id"].as<std::string>();
  }
  if (config_["rmf_fleet"]["nav_graph_timeout_s"]) {
    nav_graph_timeout_s_ =
        config_["rmf_fleet"]["nav_graph_timeout_s"].as<double>();
  }

  // Create RMF Adapter
  std::string node_name = config_["rmf_fleet"]["name"]
                              ? config_["rmf_fleet"]["name"].as<std::string>()
                              : "egb_fleet_adapter";

  adapter_ = rmf_fleet_adapter::agv::Adapter::make(node_name);

  // Declare and get server_uri ROS parameter (for API server connection)
  auto node = adapter_->node();

  // Enable sim_time for the adapter (parameter already auto-declared by ROS)
  node->set_parameter(rclcpp::Parameter("use_sim_time", true));

  RCLCPP_INFO(node->get_logger(), "RMF Adapter configured to use sim_time");
  node->declare_parameter<std::string>("server_uri", "");
  std::string server_uri_param = node->get_parameter("server_uri").as_string();

  // Store in config for later use (overrides config file if parameter is set)
  if (!server_uri_param.empty()) {
    config_["server_uri"] = server_uri_param;
    RCLCPP_INFO(node->get_logger(), "Using server_uri from ROS parameter: %s",
                server_uri_param.c_str());
  } else if (!config_["server_uri"]) {
    // Neither parameter nor config file has server_uri
    RCLCPP_INFO(
        node->get_logger(),
        "No server_uri configured (neither ROS parameter nor config file)");
  }

  // Create TF buffer
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(adapter_->node()->get_clock());

  // Initialize Zenoh session
  try {
    auto config = zenoh::Config::create_default();
    auto session = zenoh::Session::open(std::move(config));
    zenoh_session_ = new zenoh::Session(std::move(session));

    RCLCPP_INFO(adapter_->node()->get_logger(),
                "Zenoh session initialized successfully");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(adapter_->node()->get_logger(),
                 "Failed to initialize Zenoh session: %s", e.what());
    zenoh_session_ = nullptr;
  }

  RCLCPP_INFO(adapter_->node()->get_logger(), "Fleet adapter created: %s",
              node_name.c_str());
}

void FleetAdapter::start() {
  initialize_fleet();

  RCLCPP_INFO(adapter_->node()->get_logger(), "Starting RMF adapter...");

  // Start adapter
  adapter_->start();

  RCLCPP_INFO(adapter_->node()->get_logger(),
              "RMF adapter started successfully");

  RCLCPP_INFO(adapter_->node()->get_logger(), "Starting update loop...");

  // Run update loop
  update_loop();
}

void FleetAdapter::initialize_fleet() {
  auto node = adapter_->node();

  // Parse vehicle traits
  double linear_velocity =
      config_["rmf_fleet"]["limits"]["linear"][0].as<double>();
  double linear_accel =
      config_["rmf_fleet"]["limits"]["linear"][1].as<double>();
  double angular_velocity =
      config_["rmf_fleet"]["limits"]["angular"][0].as<double>();
  double angular_accel =
      config_["rmf_fleet"]["limits"]["angular"][1].as<double>();
  double footprint_radius =
      config_["rmf_fleet"]["profile"]["footprint"].as<double>();
  double vicinity_radius =
      config_["rmf_fleet"]["profile"]["vicinity"].as<double>();
  bool reversible = config_["rmf_fleet"]["reversible"]
                        ? config_["rmf_fleet"]["reversible"].as<bool>()
                        : true;

  auto linear_limits =
      rmf_traffic::agv::VehicleTraits::Limits{linear_velocity, linear_accel};
  auto angular_limits =
      rmf_traffic::agv::VehicleTraits::Limits{angular_velocity, angular_accel};
  auto footprint = rmf_traffic::geometry::make_final_convex(
      rmf_traffic::geometry::Circle(footprint_radius));
  auto vicinity = rmf_traffic::geometry::make_final_convex(
      rmf_traffic::geometry::Circle(vicinity_radius));
  auto profile = rmf_traffic::Profile{footprint, vicinity};
  auto steering = rmf_traffic::agv::VehicleTraits::Differential(
      Eigen::Vector2d::UnitX(), reversible);

  rmf_traffic::agv::VehicleTraits traits{linear_limits, angular_limits, profile,
                                         steering};

  // Fetch the canonical nav graph YAML from imrmf-map-editor over HTTP and
  // parse it in memory with RMF's parse_graph. The editor regenerates it from
  // the S3 building on demand, so this carries full fidelity (orientation
  // constraints, docks, lifts). Fail-fast if it can't be fetched or parsed.
  if (nav_graph_url_.empty() || building_id_.empty()) {
    throw std::runtime_error(
        "rmf_fleet.nav_graph_url and rmf_fleet.building_id must be set");
  }
  auto graph = std::make_shared<rmf_traffic::agv::Graph>();
  {
    const std::string url = nav_graph_url_ + "/buildings/" + building_id_ +
                            "/nav_graph/" + nav_graph_id_;
    RCLCPP_INFO(node->get_logger(), "Fetching nav graph from %s", url.c_str());
    const std::string yaml = http_get(url, nav_graph_timeout_s_);
    *graph = parse_graph_text(yaml, traits);
    RCLCPP_INFO(node->get_logger(),
                "Loaded nav graph (building='%s', fleet='%s', %zu waypoints)",
                building_id_.c_str(), nav_graph_id_.c_str(),
                graph->num_waypoints());
  }

  graph_ = graph;

  // Parse coordinate transformations
  coordinate_transforms_ = utils::parse_transformations(config_);

  RCLCPP_INFO(node->get_logger(), "Parsed %zu coordinate transformations",
              coordinate_transforms_.size());

  // Log available coordinate transform maps for debugging
  if (coordinate_transforms_.empty()) {
    RCLCPP_WARN(node->get_logger(),
                "No coordinate transformations loaded! Robots must match graph "
                "coordinates exactly.");
  } else {
    RCLCPP_INFO(node->get_logger(), "Available coordinate transform maps:");
    for (const auto &[map_name, transform] : coordinate_transforms_) {
      RCLCPP_INFO(node->get_logger(), "  - %s", map_name.c_str());
    }
  }

  // Create fleet update handle with server_uri for API server connection
  std::string fleet_name = config_["rmf_fleet"]["name"].as<std::string>();

  // Get server URI from config (optional - needed for web dashboard)
  std::optional<std::string> server_uri = std::nullopt;
  if (config_["server_uri"]) {
    server_uri = config_["server_uri"].as<std::string>();
    RCLCPP_INFO(node->get_logger(), "Fleet %s connecting to API server: %s",
                fleet_name.c_str(), server_uri->c_str());
  } else {
    RCLCPP_INFO(node->get_logger(),
                "Fleet %s: No server_uri configured, fleet state will not be "
                "published to API server",
                fleet_name.c_str());
  }

  // Pass server_uri as 4th parameter to enable API server connection
  fleet_handle_ = adapter_->add_fleet(fleet_name, traits, *graph, server_uri);

  // Register "go_to_place" as a performable action so RMF routes it through
  // ActionExecutor
  fleet_handle_->add_performable_action(
      "go_to_place", [](const nlohmann::json &description,
                        rmf_fleet_adapter::agv::FleetUpdateHandle::Confirmation
                            &confirmation) {
        (void)description;
        // Accept all go_to_place actions
        confirmation.accept();
      });

  RCLCPP_INFO(node->get_logger(),
              "Registered 'go_to_place' as performable action");

  fleet_handle_->add_performable_action(
      "undock", [node](const nlohmann::json &description,
                       rmf_fleet_adapter::agv::FleetUpdateHandle::Confirmation
                           &confirmation) {
        (void)description;
        RCLCPP_INFO(node->get_logger(),
                    "Undock action requested - confirming acceptance");
        // Accept all undock actions
        confirmation.accept();
      });

  RCLCPP_INFO(node->get_logger(), "Registered 'undock' as performable action");

  fleet_handle_->add_performable_action(
      "pickup", [node](const nlohmann::json &description,
                       rmf_fleet_adapter::agv::FleetUpdateHandle::Confirmation
                           &confirmation) {
        (void)description;
        RCLCPP_INFO(node->get_logger(),
                    "Pickup action requested - confirming acceptance");
        // Accept all pickup actions
        confirmation.accept();
      });

  RCLCPP_INFO(node->get_logger(), "Registered 'pickup' as performable action");

  fleet_handle_->add_performable_action(
      "dropoff", [node](const nlohmann::json &description,
                        rmf_fleet_adapter::agv::FleetUpdateHandle::Confirmation
                            &confirmation) {
        (void)description;
        RCLCPP_INFO(node->get_logger(),
                    "Dropoff action requested - confirming acceptance");
        // Accept all dropoff actions
        confirmation.accept();
      });

  RCLCPP_INFO(node->get_logger(), "Registered 'dropoff' as performable action");

  // Configure battery system
  if (config_["rmf_fleet"]["battery_system"]) {
    auto battery_cfg = config_["rmf_fleet"]["battery_system"];
    double voltage = battery_cfg["voltage"].as<double>();
    double capacity = battery_cfg["capacity"].as<double>();
    double charging_current = battery_cfg["charging_current"].as<double>();

    auto battery_sys = rmf_battery::agv::BatterySystem::make(voltage, capacity,
                                                             charging_current);

    auto mech_cfg = config_["rmf_fleet"]["mechanical_system"];
    double mass = mech_cfg["mass"].as<double>();
    double moment = mech_cfg["moment_of_inertia"].as<double>();
    double friction = mech_cfg["friction_coefficient"].as<double>();

    auto mech_sys =
        rmf_battery::agv::MechanicalSystem::make(mass, moment, friction);

    auto battery_sys_ptr =
        std::make_shared<rmf_battery::agv::BatterySystem>(*battery_sys);
    auto motion_sink =
        std::make_shared<rmf_battery::agv::SimpleMotionPowerSink>(
            *battery_sys_ptr, *mech_sys);

    auto ambient_cfg = config_["rmf_fleet"]["ambient_system"];
    double ambient_power = ambient_cfg["power"].as<double>();
    auto ambient_power_sys = rmf_battery::agv::PowerSystem::make(ambient_power);
    auto ambient_sink =
        std::make_shared<rmf_battery::agv::SimpleDevicePowerSink>(
            *battery_sys_ptr, *ambient_power_sys);

    auto tool_cfg = config_["rmf_fleet"]["tool_system"];
    double tool_power = tool_cfg["power"].as<double>();
    auto tool_power_sys = rmf_battery::agv::PowerSystem::make(tool_power);
    auto tool_sink = std::make_shared<rmf_battery::agv::SimpleDevicePowerSink>(
        *battery_sys_ptr, *tool_power_sys);

    double recharge_threshold =
        config_["rmf_fleet"]["recharge_threshold"].as<double>();
    double recharge_soc = config_["rmf_fleet"]["recharge_soc"].as<double>();
    bool account_for_battery_drain =
        config_["rmf_fleet"]["account_for_battery_drain"].as<bool>();
    std::string finishing_request =
        config_["rmf_fleet"]["finishing_request"].as<std::string>();

    fleet_handle_->set_task_planner_params(
        battery_sys_ptr, motion_sink, ambient_sink, tool_sink,
        recharge_threshold, recharge_soc, account_for_battery_drain,
        nullptr); // TODO: Set finishing request if needed
  }

  // Set fleet state update period for ROS2 topics and API server
  if (config_["rmf_fleet"]["publish_fleet_state"]) {
    double period = config_["rmf_fleet"]["publish_fleet_state"].as<double>();

    // Convert to std::optional<rmf_traffic::Duration>
    std::optional<rmf_traffic::Duration> period_duration =
        rmf_traffic::time::from_seconds(1.0 / period);

    // Publish to database/API server
    fleet_handle_->fleet_state_update_period(period_duration);

    // Publish to ROS2 /fleet_states topic
    fleet_handle_->fleet_state_topic_publish_period(period_duration);

    RCLCPP_INFO(
        node->get_logger(),
        "Fleet state publishing configured: %.1f Hz (ROS2 topic + API server)",
        period);
  }

  // Initialize robots
  initialize_static_robots();

  // Start dynamic discovery if enabled
  initialize_discovery();

  RCLCPP_INFO(node->get_logger(), "Fleet initialized: %s", fleet_name.c_str());
}

void FleetAdapter::initialize_static_robots() {
  /*
   * Load static robots from configuration and add them to the fleet.
   * Each robot gets its own Ros2RobotAdapter and is registered with RMF.
   */

  auto node = adapter_->node();

  if (!config_["rmf_fleet"]["robots"]) {
    RCLCPP_INFO(node->get_logger(), "No static robots configured");
    return;
  }

  // Get linear velocity from fleet config for all robots
  double linear_velocity =
      config_["rmf_fleet"]["limits"]["linear"][0].as<double>();

  for (const auto &robot_config : config_["rmf_fleet"]["robots"]) {
    if (!robot_config["name"]) {
      RCLCPP_WARN(node->get_logger(), "Robot config missing name, skipping");
      continue;
    }

    std::string robot_name = robot_config["name"].as<std::string>();

    try {
      RCLCPP_INFO(node->get_logger(), "Initializing robot: %s",
                  robot_name.c_str());

      // Create MPC Control Adapter for navigation (needed early for PathHandle)
      auto control_adapter = std::make_shared<NavigationController>(
          robot_name, node, zenoh_session_, tf_buffer_);

      // Create FullControlHandle for this robot with coordinate transforms and
      // graph
      auto command_handle = std::make_shared<PathHandle>(
          robot_name, control_adapter, node, coordinate_transforms_, graph_);

      // Create RobotState with command_handle for docking updates
      auto adapter = std::make_shared<RobotState>(
          robot_name, robot_config, config_["plugins"], node, zenoh_session_,
          fleet_handle_, tf_buffer_, linear_velocity, command_handle);

      // Set graph and coordinate transforms for undocking
      adapter->set_graph_and_transforms(graph_, coordinate_transforms_);

      // Set navigation interface for action execution (enables undocking)
      adapter->set_navigation_interface(control_adapter);

      // Get initial robot pose
      auto initial_pose = adapter->get_pose();
      if (!initial_pose) {
        RCLCPP_ERROR(
            node->get_logger(),
            "Could not get initial pose for robot %s after initialization. "
            "TF data may not be available. Skipping robot - it can be added "
            "via dynamic discovery.",
            robot_name.c_str());
        continue; // Skip this robot, don't add with invalid pose
      }

      // Get map name
      std::string map_name = adapter->get_map_name();

      // Get vehicle profile from config (footprint and vicinity)
      double footprint_radius =
          config_["rmf_fleet"]["profile"]["footprint"].as<double>();
      double vicinity_radius =
          footprint_radius * 2.0; // Default to 2x footprint
      if (config_["rmf_fleet"]["profile"]["vicinity"]) {
        vicinity_radius =
            config_["rmf_fleet"]["profile"]["vicinity"].as<double>();
      }

      auto footprint_shape =
          std::make_shared<rmf_traffic::geometry::FinalConvexShape>(
              rmf_traffic::geometry::Circle(footprint_radius)
                  .finalize_convex());
      auto vicinity_shape =
          std::make_shared<rmf_traffic::geometry::FinalConvexShape>(
              rmf_traffic::geometry::Circle(vicinity_radius).finalize_convex());
      auto profile = rmf_traffic::Profile(footprint_shape, vicinity_shape);

      // Compute starting waypoints from robot's current position
      RCLCPP_INFO(node->get_logger(),
                  "[%s] Computing plan starts: map=%s, pose=(%.2f, %.2f, %.2f)",
                  robot_name.c_str(), map_name.c_str(), (*initial_pose)[0],
                  (*initial_pose)[1], (*initial_pose)[2]);

      auto starts = rmf_traffic::agv::compute_plan_starts(
          *graph_, map_name, *initial_pose,
          rmf_traffic::Time(
              rmf_traffic::Duration(adapter_->node()->now().nanoseconds())),
          1.0, // max_merge_waypoint_distance
          1.0  // max_merge_lane_distance
      );

      RCLCPP_INFO(node->get_logger(),
                  "[%s] Computed %zu plan starts from position",
                  robot_name.c_str(), starts.size());

      // Check if we got any valid starts
      if (starts.empty()) {
        RCLCPP_ERROR(
            node->get_logger(),
            "[%s] Failed to find starting waypoint at pose (%.2f, %.2f, %.2f) "
            "on map [%s]. "
            "Robot is too far from any waypoint (>1.0m) or lane (>1.0m). "
            "Graph has %zu waypoints. Skipping robot.",
            robot_name.c_str(), (*initial_pose)[0], (*initial_pose)[1],
            (*initial_pose)[2], map_name.c_str(), graph_->num_waypoints());
        continue; // Don't add robot with empty StartSet - will cause segfault!
      }

      // Register robot with RMF fleet
      // Capture adapter as shared_ptr to ensure proper lifetime
      auto adapter_ptr = adapter;
      fleet_handle_->add_robot(
          command_handle, robot_name, profile, starts,
          [adapter_ptr, robot_name](
              std::shared_ptr<rmf_fleet_adapter::agv::RobotUpdateHandle>
                  handle) {
            // Store update handle - needed for update_position() calls
            adapter_ptr->set_update_handle(handle);

            // Register action executor to receive ActionExecution for task
            // actions
            handle->set_action_executor(
                [adapter_ptr, robot_name](
                    const std::string &category,
                    const nlohmann::json &description,
                    rmf_fleet_adapter::agv::RobotUpdateHandle::ActionExecution
                        execution) {
                  RCLCPP_INFO(rclcpp::get_logger("egb_fleet_adapter"),
                              "[%s] ===== ACTION EXECUTOR CALLED ===== "
                              "category=%s, description=%s",
                              adapter_ptr->name().c_str(), category.c_str(),
                              description.dump().c_str());

                  // Execute action through adapter
                  // The adapter will extract the identifier and store the
                  // execution internally
                  adapter_ptr->execute_action(category, description,
                                              std::move(execution));

                  // execution.finished() will be called when action completes
                  // in the navigation completion or action completion callback
                });

            RCLCPP_INFO(rclcpp::get_logger("egb_fleet_adapter"),
                        "Robot %s registered with RMF, update handle stored",
                        robot_name.c_str());
          });

      // Store the adapter and command handle - MUST keep command_handle alive!
      robots_[robot_name] = adapter;
      command_handles_[robot_name] = command_handle;

      RCLCPP_INFO(
          node->get_logger(),
          "Robot %s initialized on map %s at position (%.2f, %.2f, %.2f)",
          robot_name.c_str(), map_name.c_str(), (*initial_pose)[0],
          (*initial_pose)[1], (*initial_pose)[2]);

    } catch (const std::exception &e) {
      RCLCPP_ERROR(node->get_logger(), "Failed to initialize robot %s: %s",
                   robot_name.c_str(), e.what());
    }
  }

  RCLCPP_INFO(node->get_logger(), "Initialized %zu static robots",
              robots_.size());
}

void FleetAdapter::initialize_discovery() {
  /*
   * Initialize dynamic robot discovery if enabled in configuration.
   * Discovery scans for new robots and adds them to the fleet automatically.
   */

  auto node = adapter_->node();

  if (!config_["rmf_fleet"]["dynamic_discovery"]) {
    RCLCPP_INFO(node->get_logger(), "Dynamic discovery not configured");
    return;
  }

  if (!config_["rmf_fleet"]["dynamic_discovery"]["enabled"] ||
      !config_["rmf_fleet"]["dynamic_discovery"]["enabled"].as<bool>()) {
    RCLCPP_INFO(node->get_logger(), "Dynamic discovery disabled");
    return;
  }

  try {
    RCLCPP_INFO(node->get_logger(), "Initializing dynamic robot discovery");

    discovery_ = std::make_shared<RestRobotDiscovery>(
        node, zenoh_session_, fleet_handle_, graph_, config_,
        config_["rmf_fleet"]["dynamic_discovery"],
        config_["rmf_fleet"]["robot_template"], config_["plugins"],
        coordinate_transforms_);

    RCLCPP_INFO(node->get_logger(), "Dynamic discovery initialized");

  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize discovery: %s",
                 e.what());
  }
}

void FleetAdapter::update_loop() {
  auto node = adapter_->node();

  // Update rate (e.g., 20 Hz)
  double update_frequency =
      config_["rmf_fleet"]["robot_state_update_frequency"]
          ? config_["rmf_fleet"]["robot_state_update_frequency"].as<double>()
          : 20.0;

  auto update_period =
      std::chrono::milliseconds(static_cast<int>(1000.0 / update_frequency));

  RCLCPP_INFO(node->get_logger(), "Starting update loop at %.1f Hz",
              update_frequency);

  // Create timer callback for periodic robot state updates and discovery
  auto timer_callback = [this, node]() {
    RCLCPP_DEBUG(node->get_logger(), "Update tick: %zu robots", robots_.size());

    // Update all robot states via adapter's update() method
    for (auto &[name, adapter] : robots_) {
      try {
        // Adapter handles: pose transformation, graph localization, lane
        // tracking, waypoint progress, arrival estimation, and battery updates
        adapter->update(graph_);

        // Poll action state machine for any active actions
        adapter->update_current_action();
      } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "[%s] Update error: %s", name.c_str(),
                     e.what());
      }
    }

    // Run discovery to find new robots
    if (discovery_) {
      try {
        auto discovered_robots = discovery_->discover_robots();

        RCLCPP_DEBUG(
            node->get_logger(),
            "Discovery returned %zu robots, currently have %zu in main map",
            discovered_robots.size(), robots_.size());

        // Merge newly discovered robots into robots_ map
        for (auto &[name, robot_adapter] : discovered_robots) {
          if (robots_.find(name) == robots_.end()) {
            RCLCPP_INFO(node->get_logger(),
                        "Merging discovered robot into main map: %s",
                        name.c_str());
            robots_[name] = robot_adapter;
          }
        }
      } catch (const std::exception &e) {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(),
                             10000, // Log once every 10 seconds
                             "Discovery error: %s", e.what());
      }
    }
  };

  // Create and store timer - MUST keep this alive or it will be destroyed
  update_timer_ = node->create_wall_timer(update_period, timer_callback);

  RCLCPP_INFO(node->get_logger(), "Update timer created and stored");

  // Keep alive - wait for shutdown signal
  // The RMF adapter will handle spinning the node
  while (rclcpp::ok() && !shutdown_requested_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(node->get_logger(), "Update loop stopped");
}

void FleetAdapter::shutdown() {
  RCLCPP_INFO(adapter_->node()->get_logger(), "Fleet adapter shutting down");

  // Set shutdown flag to stop update loop
  shutdown_requested_ = true;

  // Cancel the update timer
  if (update_timer_) {
    update_timer_->cancel();
    update_timer_.reset();
  }

  // Cancel all active navigations
  for (auto &[name, adapter] : robots_) {
    try {
      RCLCPP_INFO(adapter_->node()->get_logger(),
                  "Canceling navigation for robot: %s", name.c_str());
    } catch (const std::exception &e) {
      RCLCPP_WARN(adapter_->node()->get_logger(),
                  "Error canceling navigation for %s: %s", name.c_str(),
                  e.what());
    }
  }

  // Clear discovery
  discovery_.reset();

  // Clear all robots
  robots_.clear();

  // Cleanup Zenoh session
  if (zenoh_session_) {
    delete static_cast<zenoh::Session *>(zenoh_session_);
    zenoh_session_ = nullptr;
    RCLCPP_INFO(adapter_->node()->get_logger(), "Zenoh session closed");
  }

  RCLCPP_INFO(adapter_->node()->get_logger(),
              "Fleet adapter shutdown complete");
}

} // namespace egb_fleet
