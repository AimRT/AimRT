// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "core/channel/channel_backend_base.h"

#include "core/channel/channel_backend_tools.h"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/subscription_options.hpp"

namespace aimrt::plugins::ros2_plugin {

class Ros2AdapterSubscription : public rclcpp::SubscriptionBase {
 public:
  Ros2AdapterSubscription(
      rclcpp::node_interfaces::NodeBaseInterface* node_base,
      const rosidl_message_type_support_t& type_support_handle,
      const std::string& topic_name,
      const rcl_subscription_options_t& subscription_options,
      const aimrt::runtime::core::channel::SubscribeWrapper& subscribe_wrapper,
      const aimrt::runtime::core::channel::SubscribeTool& sub_tool)
      : rclcpp::SubscriptionBase(
            node_base,
            type_support_handle,
            topic_name,
            subscription_options,
            rclcpp::SubscriptionEventCallbacks{},      // 空的事件回调
            false,                                     // 不使用 intra-process
            rclcpp::DeliveredMessageKind::ROS_MESSAGE  // 默认消息传递类型
            ),
        subscribe_wrapper_(subscribe_wrapper),
        sub_tool_(sub_tool) {}

  ~Ros2AdapterSubscription() override = default;

  std::shared_ptr<void> create_message() override;

  std::shared_ptr<rclcpp::SerializedMessage> create_serialized_message()
      override;

  void handle_message(std::shared_ptr<void>& message,
                      const rclcpp::MessageInfo& message_info) override;

  void handle_serialized_message(
      const std::shared_ptr<rclcpp::SerializedMessage>& serialized_message,
      const rclcpp::MessageInfo& message_info) override;

  void handle_loaned_message(void* loaned_message,
                             const rclcpp::MessageInfo& message_info) override;

  void return_message(std::shared_ptr<void>& message) override;

  void return_serialized_message(
      std::shared_ptr<rclcpp::SerializedMessage>& message) override;

  rclcpp::dynamic_typesupport::DynamicMessageType::SharedPtr
  get_shared_dynamic_message_type() override;

  rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr
  get_shared_dynamic_message() override;

  rclcpp::dynamic_typesupport::DynamicSerializationSupport::SharedPtr
  get_shared_dynamic_serialization_support() override;

  rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr
  create_dynamic_message() override;

  void return_dynamic_message(rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr& message) override;

  void handle_dynamic_message(
      const rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr& message,
      const rclcpp::MessageInfo& message_info) override;

  void Start() { run_flag_.store(true); }
  void Shutdown() { run_flag_.store(false); }

 private:
  std::atomic_bool run_flag_ = false;
  const runtime::core::channel::SubscribeWrapper& subscribe_wrapper_;
  const aimrt::runtime::core::channel::SubscribeTool& sub_tool_;
};

}  // namespace aimrt::plugins::ros2_plugin
