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

// DYNAMIC TYPE ==================================================================================
// TODO(methylDragon): Reorder later
// TODO(methylDragon): Implement later...
rclcpp::dynamic_typesupport::DynamicMessageType::SharedPtr
Ros2AdapterSubscription::get_shared_dynamic_message_type() {
  throw rclcpp::exceptions::UnimplementedError(
      "get_shared_dynamic_message_type is not implemented for Subscription");
}

rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr
Ros2AdapterSubscription::get_shared_dynamic_message() {
  throw rclcpp::exceptions::UnimplementedError(
      "get_shared_dynamic_message is not implemented for Subscription");
}

rclcpp::dynamic_typesupport::DynamicSerializationSupport::SharedPtr
Ros2AdapterSubscription::get_shared_dynamic_serialization_support() {
  throw rclcpp::exceptions::UnimplementedError(
      "get_shared_dynamic_serialization_support is not implemented for Subscription");
}

rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr
Ros2AdapterSubscription::create_dynamic_message() {
  throw rclcpp::exceptions::UnimplementedError(
      "create_dynamic_message is not implemented for Subscription");
}

void Ros2AdapterSubscription::return_dynamic_message(rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr& message) {
  (void)message;
  throw rclcpp::exceptions::UnimplementedError(
      "return_dynamic_message is not implemented for Subscription");
}
void Ros2AdapterSubscription::handle_dynamic_message(
    const rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr& message,
    const rclcpp::MessageInfo& message_info) {
  (void)message;
  (void)message_info;
  throw rclcpp::exceptions::UnimplementedError(
      "handle_dynamic_message is not implemented for Subscription");
}

}  // namespace aimrt::plugins::ros2_plugin