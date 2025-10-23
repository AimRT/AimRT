// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <aimrt_module_cpp_interface/channel/channel_handle.h>
#include <aimrt_module_cpp_interface/co/task.h>
#include <aimrt_module_cpp_interface/context/details/concepts.h>
#include <aimrt_module_cpp_interface/util/function.h>
#include <aimrt_module_protobuf_interface/channel/protobuf_channel.h>

#ifdef AIMRT_BUILD_WITH_ROS2
  #include <aimrt_module_ros2_interface/channel/ros2_channel.h>
#endif

#include <memory>
#include <string>
#include <string_view>
#include <type_traits>

namespace aimrt::context::details {

template <concepts::DirectlySupportedType T>
const aimrt_type_support_base_t* GetMessageTypeSupport() {
  if constexpr (concepts::RosMessage<T>) {
#ifdef AIMRT_BUILD_WITH_ROS2
    return aimrt::GetRos2MessageTypeSupport<T>();
#else
    static_assert(std::is_void_v<T>, "ROS2 support is not enabled.");
    return nullptr;
#endif
  } else if constexpr (concepts::Protobuf<T>) {
    return aimrt::GetProtobufMessageTypeSupport<T>();
  } else {
    static_assert(std::is_void_v<T>, "Unsupported channel message type.");
    return nullptr;
  }
}

template <class T>
std::shared_ptr<const T> MakeSharedMessage(
    const void* msg_raw_ptr,
    aimrt_function_base_t* release_callback_base) {
  aimrt::channel::SubscriberReleaseCallback release_callback(release_callback_base);
  return std::shared_ptr<const T>(
      static_cast<const T*>(msg_raw_ptr),
      [release_callback = std::move(release_callback)](const T*) mutable { release_callback(); });
}

template <concepts::DirectlySupportedType T>
bool RegisterPublishType(aimrt::channel::PublisherRef publisher) {
  if constexpr (concepts::RosMessage<T>) {
#ifdef AIMRT_BUILD_WITH_ROS2
    return aimrt::channel::RegisterPublishType<T>(publisher);
#else
    static_assert(std::is_void_v<T>, "ROS2 support is not enabled.");
    return false;
#endif
  } else if constexpr (concepts::Protobuf<T>) {
    return aimrt::channel::RegisterPublishType<T>(publisher);
  } else {
    static_assert(std::is_void_v<T>, "Unsupported channel message type.");
    return false;
  }
}

template <concepts::DirectlySupportedType T>
void Publish(aimrt::channel::PublisherRef publisher, aimrt::channel::ContextRef ctx_ref, const T& msg) {
  if constexpr (concepts::RosMessage<T>) {
#ifdef AIMRT_BUILD_WITH_ROS2
    aimrt::channel::Publish(publisher, ctx_ref, msg);
#else
    static_assert(std::is_void_v<T>, "ROS2 support is not enabled.");
#endif
  } else if constexpr (concepts::Protobuf<T>) {
    aimrt::channel::Publish(publisher, ctx_ref, msg);
  } else {
    static_assert(std::is_void_v<T>, "Unsupported channel message type.");
  }
}

}  // namespace aimrt::context::details
