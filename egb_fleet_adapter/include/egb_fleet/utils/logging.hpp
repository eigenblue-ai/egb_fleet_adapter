// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#ifndef EGB_FLEET__UTILS__LOGGING_HPP_
#define EGB_FLEET__UTILS__LOGGING_HPP_

#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace egb_fleet {
namespace utils {

/**
 * @brief Format Eigen::Vector3d as string for logging
 * @param vec 3D vector [x, y, z] or [x, y, yaw]
 * @return Formatted string like "[x=1.23, y=4.56, z=7.89]"
 */
std::string format_vector(const Eigen::Vector3d &vec);

/**
 * @brief Format goal ID bytes as hex string
 * @param goal_id 16-byte UUID
 * @return Hex string like "a1b2c3d4-e5f6-..."
 */
std::string format_goal_id(const std::array<uint8_t, 16> &goal_id);

} // namespace utils
} // namespace egb_fleet

#endif // EGB_FLEET__UTILS__LOGGING_HPP_
