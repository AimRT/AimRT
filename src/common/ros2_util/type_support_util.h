// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

namespace aimrt::common::ros2_util {

inline const rosidl_message_type_support_t* ImproveToIntrospectionTypeSupport(
    const rosidl_message_type_support_t* type_supports) {
  return get_message_typesupport_handle(
      type_supports, rosidl_typesupport_introspection_cpp::typesupport_identifier);
}

template <typename MsgType>
inline const rosidl_message_type_support_t* GetTypeSupport() {
  return rosidl_typesupport_cpp::get_message_type_support_handle<MsgType>();
}

template <typename MsgType>
inline const rosidl_message_type_support_t* GetIntrospectionTypeSupport() {
  return ImproveToIntrospectionTypeSupport(
      rosidl_typesupport_cpp::get_message_type_support_handle<MsgType>());
}

inline const rosidl_typesupport_introspection_cpp::MessageMembers* GetRosMembersInfo(
    const rosidl_message_type_support_t* ts) {
  return reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(
      ImproveToIntrospectionTypeSupport(ts)->data);
}

}  // namespace aimrt::common::ros2_util