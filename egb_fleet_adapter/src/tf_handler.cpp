// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors

#include "egb_fleet/tf_handler.hpp"
#include "egb_fleet/utils/zenoh_helpers.hpp"

#include <rclcpp/serialization.hpp>
#include <tf2/exceptions.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <zenoh.hxx>

namespace egb_fleet {

TfHandler::TfHandler(const std::string &robot_name, void *zenoh_session,
                     std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                     std::shared_ptr<rclcpp::Node> node,
                     const std::string &robot_frame,
                     const std::string &map_frame)
    : robot_name_(robot_name), zenoh_session_(zenoh_session),
      tf_buffer_(tf_buffer), node_(node), robot_frame_(robot_frame),
      map_frame_(map_frame), tf_subscriber_(nullptr) {
  if (zenoh_session) {
    try {
      auto *session = static_cast<zenoh::Session *>(zenoh_session);
      std::string tf_topic = utils::namespacify(robot_name, "tf");

      // Create subscriber with callback
      auto data_callback = [this](const zenoh::Sample &sample) {
        // Extract payload as string and convert to bytes
        auto payload_str = sample.get_payload().as_string();
        std::vector<uint8_t> data(payload_str.begin(), payload_str.end());
        this->tf_callback(data);
      };

      auto subscriber = session->declare_subscriber(tf_topic, data_callback,
                                                    zenoh::closures::none);

      // Store subscriber (keep it alive)
      tf_subscriber_ = new zenoh::Subscriber<void>(std::move(subscriber));

      RCLCPP_INFO(node_->get_logger(),
                  "[%s] TF Handler initialized with Zenoh subscription to: %s",
                  robot_name_.c_str(), tf_topic.c_str());

    } catch (const std::exception &e) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[%s] Failed to create Zenoh TF subscription: %s",
                   robot_name_.c_str(), e.what());
      tf_subscriber_ = nullptr;
    }
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "[%s] TF Handler initialized without Zenoh session - will use "
                "tf_buffer lookups only",
                robot_name_.c_str());
  }
}

TfHandler::~TfHandler() {
  if (tf_subscriber_) {
    delete static_cast<zenoh::Subscriber<void> *>(tf_subscriber_);
    tf_subscriber_ = nullptr;
  }
}

std::optional<geometry_msgs::msg::TransformStamped> TfHandler::get_transform() {
  try {
    std::string namespaced_map = utils::namespacify(robot_name_, map_frame_);
    std::string namespaced_robot =
        utils::namespacify(robot_name_, robot_frame_);

    auto transform =
        tf_buffer_->lookupTransform(namespaced_map, namespaced_robot,
                                    tf2::TimePointZero, // Latest available
                                    tf2::durationFromSec(0.5));

    return transform;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_DEBUG(node_->get_logger(), "[%s] TF lookup failed: %s",
                 robot_name_.c_str(), ex.what());
    return std::nullopt;
  }
}

void TfHandler::tf_callback(const std::vector<uint8_t> &payload) {
  try {
    // Deserialize TFMessage using ROS2 serialization
    tf2_msgs::msg::TFMessage tf_msg;
    rclcpp::SerializedMessage serialized_msg;

    // Copy payload into serialized message
    serialized_msg.reserve(payload.size());
    std::memcpy(serialized_msg.get_rcl_serialized_message().buffer,
                payload.data(), payload.size());
    serialized_msg.get_rcl_serialized_message().buffer_length = payload.size();

    // Deserialize
    rclcpp::Serialization<tf2_msgs::msg::TFMessage> serializer;
    serializer.deserialize_message(&serialized_msg, &tf_msg);

    // Process each transform
    for (auto &transform : tf_msg.transforms) {
      // Namespace the frame IDs with robot name
      transform.header.frame_id =
          utils::namespacify(robot_name_, transform.header.frame_id);
      transform.child_frame_id =
          utils::namespacify(robot_name_, transform.child_frame_id);

      // Add to TF buffer
      // Use "zenoh" as authority to indicate source
      try {
        tf_buffer_->setTransform(transform, "zenoh", false);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_DEBUG(node_->get_logger(),
                     "[%s] Failed to set transform %s -> %s: %s",
                     robot_name_.c_str(), transform.header.frame_id.c_str(),
                     transform.child_frame_id.c_str(), ex.what());
      }
    }

  } catch (const std::exception &e) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(),
                         5000, // Log once every 5 seconds
                         "[%s] Failed to process TF message: %s",
                         robot_name_.c_str(), e.what());
  }
}

} // namespace egb_fleet
