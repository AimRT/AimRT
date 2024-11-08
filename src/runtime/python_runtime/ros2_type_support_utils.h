#pragma once

#include "pybind11/pybind11.h"

namespace aimrt::runtime::python_runtime {

namespace py = pybind11;

// The following code is adapted from ros2 rclpy.
// see: https://github.com/ros2/rclpy/blob/humble/rclpy/src/rclpy/utils.cpp

typedef void destroy_ros_message_function(void*);
typedef void* create_ros_message_function();

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
  typedef bool convert_from_py_signature(PyObject*, void*);

  std::unique_ptr<void, destroy_ros_message_function*> message =
      create_from_py(pymessage);

  py::object pymetaclass = pymessage.attr("__class__");

  auto capsule_ptr = static_cast<void*>(
      pymetaclass.attr("_CONVERT_FROM_PY").cast<py::capsule>());
  auto convert =
      reinterpret_cast<convert_from_py_signature*>(capsule_ptr);
  if (!convert) {
    throw py::error_already_set();
  }

  if (!convert(pymessage.ptr(), message.get())) {
    throw py::error_already_set();
  }

  return message;
}

inline py::object convert_to_py(void* message, py::object pyclass) {
  py::object pymetaclass = pyclass.attr("__class__");

  auto capsule_ptr = static_cast<void*>(
      pymetaclass.attr("_CONVERT_TO_PY").cast<py::capsule>());

  typedef PyObject* convert_to_py_function(void*);
  auto convert = reinterpret_cast<convert_to_py_function*>(capsule_ptr);
  if (!convert) {
    throw py::error_already_set();
  }
  return py::reinterpret_steal<py::object>(convert(message));
}

// End of adapted code from ros2 rclpy.

}  // namespace aimrt::runtime::python_runtime