// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#include "egb_fleet/robot_handle.hpp"

namespace egb_fleet {

//==============================================================================
// ExecutionHandle implementation
//==============================================================================

//==============================================================================
// RobotHandle implementation
//==============================================================================

RobotHandle::RobotHandle(const std::string &name,
                         std::shared_ptr<rclcpp::Node> node,
                         std::shared_ptr<FleetUpdateHandle> fleet_handle)
    : name_(name), node_(node), fleet_handle_(fleet_handle),
      update_handle_(nullptr) {}

void RobotHandle::set_update_handle(std::shared_ptr<RobotUpdateHandle> handle) {
  update_handle_ = handle;
}

void RobotHandle::set_fleet_handle(std::shared_ptr<FleetUpdateHandle> handle) {
  fleet_handle_ = handle;
}

} // namespace egb_fleet
