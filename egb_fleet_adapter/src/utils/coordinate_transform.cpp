// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#include "egb_fleet/utils/coordinate_transform.hpp"

#include <cmath>
#include <stdexcept>

namespace egb_fleet {
namespace utils {

std::map<std::string, rmf_fleet_adapter::agv::Transformation>
parse_transformations(const YAML::Node &config) {
  /* Python reference (fleet_adapter.py:200-230):

  reference_coordinates = config_yaml.get('reference_coordinates')
  coordinate_transforms = {}

  if reference_coordinates:
      for level_name, coords in reference_coordinates.items():
          rmf_coords = coords.get('rmf', [])
          robot_coords = coords.get('robot', [])

          if len(rmf_coords) < 1 or len(robot_coords) < 1:
              node.get_logger().warn(
                  f'Insufficient reference coordinates for level {level_name}')
              continue

          # Estimate transformation using nudged library
          tf = nudged.estimate(rmf_coords, robot_coords)
          coordinate_transforms[level_name] = tf

  return coordinate_transforms
  */

  std::map<std::string, rmf_fleet_adapter::agv::Transformation> transforms;

  if (!config["reference_coordinates"]) {
    return transforms;
  }

  for (const auto &level_node : config["reference_coordinates"]) {
    std::string level_name = level_node.first.as<std::string>();
    YAML::Node coords = level_node.second;

    if (!coords["rmf"] || !coords["robot"]) {
      continue;
    }

    std::vector<std::vector<double>> rmf_coords;
    std::vector<std::vector<double>> robot_coords;

    for (const auto &pt : coords["rmf"]) {
      if (pt.size() >= 2) {
        rmf_coords.push_back({pt[0].as<double>(), pt[1].as<double>()});
      }
    }

    for (const auto &pt : coords["robot"]) {
      if (pt.size() >= 2) {
        robot_coords.push_back({pt[0].as<double>(), pt[1].as<double>()});
      }
    }

    if (rmf_coords.empty() || robot_coords.empty() ||
        rmf_coords.size() != robot_coords.size()) {
      continue;
    }

    transforms.emplace(level_name,
                       estimate_transformation(rmf_coords, robot_coords));
  }

  return transforms;
}

rmf_fleet_adapter::agv::Transformation
estimate_transformation(const std::vector<std::vector<double>> &rmf_coords,
                        const std::vector<std::vector<double>> &robot_coords) {
  /* Python reference using nudged library:

  import nudged
  tf = nudged.estimate(rmf_coords, robot_coords)

  # nudged provides: rotation, scale, translation
  # For 2D similarity transformation: [x', y'] = s*R*[x, y] + t
  # where s = scale, R = rotation matrix, t = translation

  # Apply transformation:
  robot_pose_array = tf.apply_inverse(rmf_pose_array)  # RMF -> robot
  */

  if (rmf_coords.size() != robot_coords.size() || rmf_coords.empty()) {
    throw std::invalid_argument(
        "Coordinate arrays must be non-empty and same size");
  }

  // For single point, return pure translation
  if (rmf_coords.size() == 1) {
    Eigen::Vector2d translation(robot_coords[0][0] - rmf_coords[0][0],
                                robot_coords[0][1] - rmf_coords[0][1]);

    // Return transformation with no rotation, unit scale, and translation
    return rmf_fleet_adapter::agv::Transformation(0.0, // rotation (radians)
                                                  1.0, // scale
                                                  translation);
  }

  // For multiple points, use least-squares estimation
  // Compute centroids
  Eigen::Vector2d rmf_centroid(0.0, 0.0);
  Eigen::Vector2d robot_centroid(0.0, 0.0);

  for (size_t i = 0; i < rmf_coords.size(); ++i) {
    rmf_centroid += Eigen::Vector2d(rmf_coords[i][0], rmf_coords[i][1]);
    robot_centroid += Eigen::Vector2d(robot_coords[i][0], robot_coords[i][1]);
  }

  rmf_centroid /= rmf_coords.size();
  robot_centroid /= robot_coords.size();

  // Compute covariance matrix
  Eigen::Matrix2d H = Eigen::Matrix2d::Zero();

  for (size_t i = 0; i < rmf_coords.size(); ++i) {
    Eigen::Vector2d rmf_pt(rmf_coords[i][0], rmf_coords[i][1]);
    Eigen::Vector2d robot_pt(robot_coords[i][0], robot_coords[i][1]);

    Eigen::Vector2d rmf_centered = rmf_pt - rmf_centroid;
    Eigen::Vector2d robot_centered = robot_pt - robot_centroid;

    H += robot_centered * rmf_centered.transpose();
  }

  // SVD to extract rotation
  Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
  Eigen::Matrix2d U = svd.matrixU();
  Eigen::Matrix2d V = svd.matrixV();

  Eigen::Matrix2d R = U * V.transpose();

  // Ensure proper rotation (det = 1)
  if (R.determinant() < 0) {
    U.col(1) *= -1;
    R = U * V.transpose();
  }

  // Extract rotation angle
  double rotation = std::atan2(R(1, 0), R(0, 0));

  // Compute scale (ratio of distances)
  double rmf_scale = 0.0;
  double robot_scale = 0.0;

  for (size_t i = 0; i < rmf_coords.size(); ++i) {
    Eigen::Vector2d rmf_pt(rmf_coords[i][0], rmf_coords[i][1]);
    Eigen::Vector2d robot_pt(robot_coords[i][0], robot_coords[i][1]);

    rmf_scale += (rmf_pt - rmf_centroid).norm();
    robot_scale += (robot_pt - robot_centroid).norm();
  }

  double scale = robot_scale / rmf_scale;

  // Compute translation: t = robot_centroid - s*R*rmf_centroid
  Eigen::Vector2d translation = robot_centroid - scale * R * rmf_centroid;

  return rmf_fleet_adapter::agv::Transformation(rotation, scale, translation);
}

} // namespace utils
} // namespace egb_fleet
