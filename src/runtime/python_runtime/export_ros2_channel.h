// Copyright (c) 2024 The AimRT Authors.
// AimRT is licensed under Mulan PSL v2.

#pragma once

#include <pybind11/pybind11.h>

#include <pybind11/functional.h>  // IWYU pragma: keep
#include <pybind11/stl.h>         // IWYU pragma: keep

#include "aimrt_module_cpp_interface/channel/channel_context.h"
#include "aimrt_module_cpp_interface/channel/channel_handle.h"

#include "python_runtime/export_ros2_type_support.h"

namespace aimrt::runtime::python_runtime {

inline bool Ros2RegisterPublishType(
    aimrt::channel::PublisherRef& publisher_ref,
    const std::shared_ptr<const PyRos2TypeSupport>& py_ros2_type_support) {
  static std::vector<std::shared_ptr<const PyRos2TypeSupport>> py_ros2_ts_vec;
  py_ros2_ts_vec.emplace_back(py_ros2_type_support);

  return publisher_ref.RegisterPublishType(py_ros2_type_support->NativeHandle());
}

inline void Ros2PublishWithCtx(
    aimrt::channel::PublisherRef& publisher_ref,
    std::string_view msg_type,
    const aimrt::channel::ContextRef& ctx_ref,
    pybind11::object msg_obj) {
  auto msg_ptr = convert_from_py(msg_obj);
  if (!msg_ptr) {
    throw py::error_already_set();
  }

  publisher_ref.Publish(msg_type, ctx_ref, static_cast<const void*>(msg_ptr.get()));
}

inline bool Ros2SubscribeWithCtx(
    aimrt::channel::SubscriberRef& subscriber_ref,
    const std::shared_ptr<const PyRos2TypeSupport>& py_ros2_type_support,
    pybind11::object pyclass,
    std::function<void(aimrt::channel::ContextRef, pybind11::object)>&& callback) {
  static std::vector<std::shared_ptr<const PyRos2TypeSupport>> py_ros2_ts_vec;
  py_ros2_ts_vec.emplace_back(py_ros2_type_support);

  pybind11::gil_scoped_acquire acquire;

  pybind11::object pymetaclass = pyclass.attr("__class__");
  auto* capsule_ptr = static_cast<void*>(pymetaclass.attr("_CONVERT_TO_PY").cast<py::capsule>());
  typedef PyObject* convert_to_py_function(void*);
  auto convert = reinterpret_cast<convert_to_py_function*>(capsule_ptr);
  if (!convert) {
    throw py::error_already_set();
  }

  pybind11::gil_scoped_release release;

  return subscriber_ref.Subscribe(
      py_ros2_type_support->NativeHandle(),
      [callback = std::move(callback), convert](
          const aimrt_channel_context_base_t* ctx_ptr,
          const void* msg_ptr,
          aimrt_function_base_t* release_callback_base) {
        aimrt::channel::SubscriberReleaseCallback release_callback(release_callback_base);

        pybind11::gil_scoped_acquire acquire;

        auto msg_obj = pybind11::reinterpret_steal<pybind11::object>(convert(const_cast<void*>(msg_ptr)));
        if (!msg_obj) {
          throw py::error_already_set();
        }

        auto ctx_ref = aimrt::channel::ContextRef(ctx_ptr);
        callback(ctx_ref, msg_obj);

        pybind11::gil_scoped_release release;

        release_callback();
      });
}

inline void ExportRos2PublisherFunc(pybind11::module& m) {
  m.def("Ros2PublishWithCtx", &Ros2PublishWithCtx);
  m.def("Ros2RegisterPublishType", &Ros2RegisterPublishType);
}

inline void ExportRos2SubscribeFunc(pybind11::module& m) {
  m.def("Ros2SubscribeWithCtx", &Ros2SubscribeWithCtx);
}

}  // namespace aimrt::runtime::python_runtime
