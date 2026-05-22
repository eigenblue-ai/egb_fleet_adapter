// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#ifndef EGB_FLEET__TF_HANDLER_HPP_
#define EGB_FLEET__TF_HANDLER_HPP_

#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

namespace egb_fleet {

/// TF subscription + lookup over Zenoh, scoped to one robot namespace.
class TfHandler {
public:
  /**
   * @brief Constructor
   * @param robot_name Robot name for namespacing
   * @param zenoh_session Void pointer to zenoh::Session
   * @param tf_buffer Shared TF2 buffer
   * @param node ROS2 node for logging
   * @param robot_frame Robot base frame (default: "base_footprint")
   * @param map_frame Map frame (default: "map")
   */
  TfHandler(const std::string &robot_name, void *zenoh_session,
            std::shared_ptr<tf2_ros::Buffer> tf_buffer,
            std::shared_ptr<rclcpp::Node> node,
            const std::string &robot_frame = "base_footprint",
            const std::string &map_frame = "map");

  ~TfHandler();

  /**
   * @brief Get the transform from map to robot base
   * @return Optional TransformStamped if available
   */
  std::optional<geometry_msgs::msg::TransformStamped> get_transform();

private:
  /**
   * @brief Callback for TF messages from Zenoh
   */
  void tf_callback(const std::vector<uint8_t> &payload);

  std::string robot_name_;
  void *zenoh_session_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<rclcpp::Node> node_;
  std::string robot_frame_;
  std::string map_frame_;

  // Type-erased to avoid pulling zenoh-cpp into this header; the .cpp casts
  // back.
  void *tf_subscriber_;
};

} // namespace egb_fleet

#endif // EGB_FLEET__TF_HANDLER_HPP_
