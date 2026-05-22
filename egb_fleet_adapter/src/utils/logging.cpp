// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#include "egb_fleet/utils/logging.hpp"

#include <iomanip>
#include <sstream>

namespace egb_fleet {
namespace utils {

std::string format_vector(const Eigen::Vector3d &vec) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2);
  oss << "[x=" << vec.x() << ", y=" << vec.y() << ", z=" << vec.z() << "]";
  return oss.str();
}

std::string format_goal_id(const std::array<uint8_t, 16> &goal_id) {
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');

  for (size_t i = 0; i < 16; ++i) {
    if (i == 4 || i == 6 || i == 8 || i == 10) {
      oss << "-";
    }
    oss << std::setw(2) << static_cast<int>(goal_id[i]);
  }

  return oss.str();
}

} // namespace utils
} // namespace egb_fleet
