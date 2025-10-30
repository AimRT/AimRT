// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <aimrt_module_c_interface/util/type_support_base.h>
#include <aimrt_module_cpp_interface/channel/channel_handle.h>
#include <aimrt_module_cpp_interface/co/task.h>
#include <aimrt_module_cpp_interface/context/details/concepts.h>
#include <aimrt_module_cpp_interface/util/function.h>

#include <cassert>
#include <memory>

namespace aimrt::context::details {

template <class T>
std::shared_ptr<const T> MakeSharedMessage(
    const void* msg_raw_ptr,
    aimrt_function_base_t* release_callback_base) {
  aimrt::channel::SubscriberReleaseCallback release_callback(release_callback_base);
  return std::shared_ptr<const T>(
      static_cast<const T*>(msg_raw_ptr),
      [release_callback = std::move(release_callback)](const T*) mutable { release_callback(); });
}
}  // namespace aimrt::context::details

namespace aimrt {

template <typename T>
struct MessageTypeSupportTraits {
  static const aimrt_type_support_base_t* Get() {
    static_assert(sizeof(T) == 0,
                  "GetMessageTypeSupport<T> is not specialized for this type T. "
                  "Please include the correct type support header (e.g., aimrt_module_protobuf_interface.h or aimrt_module_ros2_interface.h).");
    assert(false);
    return nullptr;
  }
};

template <class T>
const aimrt_type_support_base_t* GetMessageTypeSupport() {
  return MessageTypeSupportTraits<T>::Get();
}

}  // namespace aimrt

namespace aimrt::channel {

template <class T>
struct MessagePublisherTraits {
  static void PublishMsg(aimrt::channel::PublisherRef publisher, aimrt::channel::ContextRef ctx_ref, const T& msg) {
    static_assert(std::is_same_v<T, void>,
                  "Publish<T> is not specialized for this message type. "
                  "Please include the correct type support header (e.g., aimrt_module_protobuf_interface.h or aimrt_module_ros2_interface.h).");
  }
};

template <class T>
void Publish(aimrt::channel::PublisherRef publisher, aimrt::channel::ContextRef ctx_ref, const T& msg) {
  MessagePublisherTraits<T>::PublishMsg(publisher, ctx_ref, msg);
  return;
}

}  // namespace aimrt::channel
