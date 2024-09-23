// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "ros2_plugin/ros2_adapter_subscription.h"
#include "aimrt_module_cpp_interface/util/type_support.h"
#include "ros2_plugin/global.h"

namespace aimrt::plugins::ros2_plugin {
std::shared_ptr<void> Ros2AdapterSubscription::create_message() {
  return subscribe_wrapper_.info.msg_type_support_ref.CreateSharedPtr();
}

std::shared_ptr<rclcpp::SerializedMessage>
Ros2AdapterSubscription::create_serialized_message() {
  return std::make_shared<rclcpp::SerializedMessage>();
}

void Ros2AdapterSubscription::handle_message(
    std::shared_ptr<void>& message, const rclcpp::MessageInfo& message_info) {
  if (!run_flag_.load()) return;

  try {
    auto ctx_ptr = std::make_shared<aimrt::channel::Context>(aimrt_channel_context_type_t::AIMRT_CHANNEL_SUBSCRIBER_CONTEXT);

    sub_tool_.DoSubscribeCallback(ctx_ptr, subscribe_wrapper_, message);
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
  }
}

void Ros2AdapterSubscription::handle_serialized_message(
    const std::shared_ptr<rclcpp::SerializedMessage>& serialized_message,
    const rclcpp::MessageInfo& message_info) {
  AIMRT_WARN("not support ros2 serialized message");
}

void Ros2AdapterSubscription::handle_loaned_message(
    void* loaned_message, const rclcpp::MessageInfo& message_info) {
  AIMRT_WARN("not support ros2 loaned message");
}

void Ros2AdapterSubscription::return_message(std::shared_ptr<void>& message) {
  message.reset();
}

void Ros2AdapterSubscription::return_serialized_message(
    std::shared_ptr<rclcpp::SerializedMessage>& message) {
  message.reset();
}

}  // namespace aimrt::plugins::ros2_plugin