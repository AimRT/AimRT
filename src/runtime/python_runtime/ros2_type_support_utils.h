// Copyright(c) 2024 The AimRT Authors.
// AimRT is licensed under Mulan PSL v2.

#pragma once

#include "pybind11/pybind11.h"  // IWYU pragma: keep

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

void* common_get_type_support(py::object pymessage);

std::unique_ptr<void, destroy_ros_message_function*>
create_from_py(py::object pymessage);

std::unique_ptr<void, destroy_ros_message_function*>
convert_from_py(py::object pymessage);

create_ros_message_function* get_create_ros_message_function(py::object pyclass);

destroy_ros_message_function* get_destroy_ros_message_function(py::object pyclass);

convert_to_py_function* get_convert_to_py_function(py::object pyclass);

convert_from_py_function* get_convert_from_py_function(py::object pyclass);

// End of adapted code from ros2 rclpy.

inline const rosidl_message_type_support_t* ImproveToIntrospectionTypeSupport(
    const rosidl_message_type_support_t* type_support) {
  return get_message_typesupport_handle(type_support, rosidl_typesupport_introspection_c__identifier);
}

inline const rosidl_typesupport_introspection_c__MessageMembers* GetRosMembersInfo(
    const rosidl_message_type_support_t* type_support) {
  const auto* ts = ImproveToIntrospectionTypeSupport(type_support);
  if (!ts) [[unlikely]] {
    throw std::runtime_error("Failed to get introspection type support.");
  }
  return reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers*>(ts->data);
}

void CopyRosMessage(const rosidl_typesupport_introspection_c__MessageMembers* members, const void* from, void* to);

void MoveRosMessage(const rosidl_typesupport_introspection_c__MessageMembers* members, void* from, void* to);

}  // namespace aimrt::runtime::python_runtime
