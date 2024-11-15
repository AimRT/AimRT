// Copyright(c) 2024 The AimRT Authors.
// AimRT is licensed under Mulan PSL v2.

#pragma once

#include "pybind11/pybind11.h"

#include <iostream>

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"

namespace aimrt::runtime::python_runtime {

namespace py = pybind11;

// The following code is adapted from ros2 rclpy.
// see: https://github.com/ros2/rclpy/blob/humble/rclpy/src/rclpy/utils.cpp

typedef void destroy_ros_message_function(void*);
typedef void* create_ros_message_function();
typedef bool convert_from_py_function(PyObject*, void*);
typedef PyObject* convert_to_py_function(void*);

inline void* common_get_type_support(py::object pymessage) {
  py::object pymetaclass = pymessage.attr("__class__");

  py::object value = pymetaclass.attr("_TYPE_SUPPORT");
  auto* capsule_ptr = static_cast<void*>(value.cast<py::capsule>());

  return capsule_ptr;
}

inline std::unique_ptr<void, destroy_ros_message_function*>
create_from_py(py::object pymessage) {
  py::object pymetaclass = pymessage.attr("__class__");

  py::object value = pymetaclass.attr("_CREATE_ROS_MESSAGE");
  auto capsule_ptr = static_cast<void*>(value.cast<py::capsule>());
  auto create_ros_message =
      reinterpret_cast<create_ros_message_function*>(capsule_ptr);
  if (!create_ros_message) {
    throw py::error_already_set();
  }

  value = pymetaclass.attr("_DESTROY_ROS_MESSAGE");
  capsule_ptr = static_cast<void*>(value.cast<py::capsule>());
  auto destroy_ros_message =
      reinterpret_cast<destroy_ros_message_function*>(capsule_ptr);
  if (!destroy_ros_message) {
    throw py::error_already_set();
  }

  void* message = create_ros_message();
  if (!message) {
    throw std::bad_alloc();
  }
  return std::unique_ptr<
      void, destroy_ros_message_function*>(message, destroy_ros_message);
}

inline std::unique_ptr<void, destroy_ros_message_function*>
convert_from_py(py::object pymessage) {
  std::unique_ptr<void, destroy_ros_message_function*> message =
      create_from_py(pymessage);

  py::object pymetaclass = pymessage.attr("__class__");

  auto capsule_ptr = static_cast<void*>(
      pymetaclass.attr("_CONVERT_FROM_PY").cast<py::capsule>());
  auto convert =
      reinterpret_cast<convert_from_py_function*>(capsule_ptr);
  if (!convert) {
    throw py::error_already_set();
  }

  if (!convert(pymessage.ptr(), message.get())) {
    throw py::error_already_set();
  }

  return message;
}

inline create_ros_message_function* get_create_ros_message_function(py::object pyclass) {
  py::object pymetaclass = pyclass.attr("__class__");
  auto* capsule_ptr = static_cast<void*>(pymetaclass.attr("_CREATE_ROS_MESSAGE").cast<py::capsule>());
  auto create = reinterpret_cast<create_ros_message_function*>(capsule_ptr);
  if (!create) {
    throw py::error_already_set();
  }
  return create;
}

inline destroy_ros_message_function* get_destroy_ros_message_function(py::object pyclass) {
  py::object pymetaclass = pyclass.attr("__class__");
  auto* capsule_ptr = static_cast<void*>(pymetaclass.attr("_DESTROY_ROS_MESSAGE").cast<py::capsule>());
  auto destroy = reinterpret_cast<destroy_ros_message_function*>(capsule_ptr);
  if (!destroy) {
    throw py::error_already_set();
  }
  return destroy;
}

inline convert_to_py_function* get_convert_to_py_function(py::object pyclass) {
  py::object pymetaclass = pyclass.attr("__class__");
  auto* capsule_ptr = static_cast<void*>(pymetaclass.attr("_CONVERT_TO_PY").cast<py::capsule>());
  auto convert = reinterpret_cast<convert_to_py_function*>(capsule_ptr);
  if (!convert) {
    throw py::error_already_set();
  }
  return convert;
}

inline convert_from_py_function* get_convert_from_py_function(py::object pyclass) {
  py::object pymetaclass = pyclass.attr("__class__");
  auto* capsule_ptr = static_cast<void*>(pymetaclass.attr("_CONVERT_FROM_PY").cast<py::capsule>());
  auto convert = reinterpret_cast<convert_from_py_function*>(capsule_ptr);
  if (!convert) {
    throw py::error_already_set();
  }
  return convert;
}

// End of adapted code from ros2 rclpy.

inline const rosidl_message_type_support_t* ImproveToIntrospectionTypeSupport(
    const rosidl_message_type_support_t* type_support) {
  return get_message_typesupport_handle(type_support, rosidl_typesupport_introspection_c__identifier);
}

inline const rosidl_typesupport_introspection_c__MessageMembers* GetRosMembersInfo(
    const rosidl_message_type_support_t* type_support) {
  // TODO(zhangyi1357): Remove the temporary ts variable.
  const auto* ts = ImproveToIntrospectionTypeSupport(type_support);
  if (!ts) {
    throw std::runtime_error("Failed to get introspection type support.");
  }
  return reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers*>(ts->data);
}

inline void CopyBasicMember(
    const rosidl_typesupport_introspection_c__MessageMember& member_info,
    const void* from_ptr, void* to_ptr) {
  switch (member_info.type_id_) {
    case rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT:
      *reinterpret_cast<float*>(to_ptr) = *reinterpret_cast<const float*>(from_ptr);
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE:
      *reinterpret_cast<double*>(to_ptr) = *reinterpret_cast<const double*>(from_ptr);
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE:
      *reinterpret_cast<long double*>(to_ptr) = *reinterpret_cast<const long double*>(from_ptr);
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_CHAR:
      *reinterpret_cast<char*>(to_ptr) = *reinterpret_cast<const char*>(from_ptr);
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR:
      *reinterpret_cast<char16_t*>(to_ptr) = *reinterpret_cast<const char16_t*>(from_ptr);
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN:
      *reinterpret_cast<bool*>(to_ptr) = *reinterpret_cast<const bool*>(from_ptr);
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_OCTET:
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT8:
      *reinterpret_cast<uint8_t*>(to_ptr) = *reinterpret_cast<const uint8_t*>(from_ptr);
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT8:
      *reinterpret_cast<int8_t*>(to_ptr) = *reinterpret_cast<const int8_t*>(from_ptr);
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT16:
      *reinterpret_cast<uint16_t*>(to_ptr) = *reinterpret_cast<const uint16_t*>(from_ptr);
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT16:
      *reinterpret_cast<int16_t*>(to_ptr) = *reinterpret_cast<const int16_t*>(from_ptr);
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT32:
      *reinterpret_cast<uint32_t*>(to_ptr) = *reinterpret_cast<const uint32_t*>(from_ptr);
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT32:
      *reinterpret_cast<int32_t*>(to_ptr) = *reinterpret_cast<const int32_t*>(from_ptr);
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT64:
      *reinterpret_cast<uint64_t*>(to_ptr) = *reinterpret_cast<const uint64_t*>(from_ptr);
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT64:
      *reinterpret_cast<int64_t*>(to_ptr) = *reinterpret_cast<const int64_t*>(from_ptr);
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
      // TODO(zhangyi1357): Handle string type. This should be deep copy.
      *reinterpret_cast<char*>(to_ptr) = *reinterpret_cast<const char*>(from_ptr);
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING:
      // TODO(zhangyi1357): Handle wstring type. This should be deep copy.
      *reinterpret_cast<char16_t*>(to_ptr) = *reinterpret_cast<const char16_t*>(from_ptr);
      break;
    default:
      throw std::runtime_error("Unsupported basic type: " + std::to_string(member_info.type_id_));
  }
}

}  // namespace aimrt::runtime::python_runtime
