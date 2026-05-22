// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#include "egb_fleet/fleet_adapter.hpp"
#include <curl/curl.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  curl_global_init(CURL_GLOBAL_DEFAULT);

  if (argc < 2) {
    std::cerr << "Usage: fleet_adapter_node <config_path>" << std::endl;
    curl_global_cleanup();
    return 1;
  }

  try {
    egb_fleet::FleetAdapter adapter(argv[1]);
    adapter.start();
  } catch (const std::exception &e) {
    std::cerr << "Fleet adapter error: " << e.what() << std::endl;
    curl_global_cleanup();
    return 1;
  }

  curl_global_cleanup();
  rclcpp::shutdown();
  return 0;
}
