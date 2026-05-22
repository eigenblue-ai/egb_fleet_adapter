// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#ifndef EGB_FLEET__UTILS__COORDINATE_TRANSFORM_HPP_
#define EGB_FLEET__UTILS__COORDINATE_TRANSFORM_HPP_

#include <map>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

// RMF transformation class
#include <rmf_fleet_adapter/agv/Transformation.hpp>
#include <rmf_traffic/Trajectory.hpp>

namespace egb_fleet {
namespace utils {

/**
 * @brief Parse coordinate transformations from YAML config
 *
 * @param config YAML node containing reference_coordinates
 * @return Map of level_name -> Transformation
 */
std::map<std::string, rmf_fleet_adapter::agv::Transformation>
parse_transformations(const YAML::Node &config);

/**
 * @brief Estimate 2D transformation from reference point pairs
 *
 * Uses least-squares estimation to find rotation, scale, and translation
 * that best maps RMF coordinates to robot coordinates.
 *
 * @param rmf_coords Reference points in RMF coordinate system [[x1, y1], [x2,
 * y2], ...]
 * @param robot_coords Corresponding points in robot coordinate system
 * @return Transformation object (RMF -> robot)
 */
rmf_fleet_adapter::agv::Transformation
estimate_transformation(const std::vector<std::vector<double>> &rmf_coords,
                        const std::vector<std::vector<double>> &robot_coords);

} // namespace utils
} // namespace egb_fleet

#endif // EGB_FLEET__UTILS__COORDINATE_TRANSFORM_HPP_
