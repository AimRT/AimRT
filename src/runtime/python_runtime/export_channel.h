// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <utility>

#include "aimrt_module_cpp_interface/channel/channel_context.h"
#include "aimrt_module_cpp_interface/channel/channel_handle.h"

#include "python_runtime/export_ros2_type_support.h"
#include "python_runtime/export_type_support.h"

#include "pybind11/pybind11.h"

namespace aimrt::runtime::python_runtime {

inline void ExportContext(const pybind11::object& m) {
  using aimrt::channel::Context;
  using aimrt::channel::ContextRef;

  pybind11::enum_<aimrt_channel_context_type_t>(m, "aimrt_channel_context_type_t")
      .value("AIMRT_CHANNEL_PUBLISHER_CONTEXT", aimrt_channel_context_type_t::AIMRT_CHANNEL_PUBLISHER_CONTEXT)
      .value("AIMRT_CHANNEL_SUBSCRIBER_CONTEXT", aimrt_channel_context_type_t::AIMRT_CHANNEL_SUBSCRIBER_CONTEXT);

  pybind11::class_<Context>(m, "Context")
      .def(pybind11::init<>())
      .def("CheckUsed", &Context::CheckUsed)
      .def("SetUsed", &Context::SetUsed)
      .def("Reset", &Context::Reset)
      .def("GetType", &Context::GetType)
      .def("GetMetaValue", &Context::GetMetaValue)
      .def("SetMetaValue", &Context::SetMetaValue)
      .def("GetMetaKeys", &Context::GetMetaKeys)
      .def("GetSerializationType", &Context::GetSerializationType)
      .def("SetSerializationType", &Context::SetSerializationType)
      .def("ToString", &Context::ToString);

  pybind11::class_<ContextRef>(m, "ContextRef")
      .def(pybind11::init<>())
      .def(pybind11::init<const Context&>())
      .def(pybind11::init<const Context*>())
      .def(pybind11::init<const std::shared_ptr<Context>&>())
      .def("__bool__", &ContextRef::operator bool)
      .def("CheckUsed", &ContextRef::CheckUsed)
      .def("SetUsed", &ContextRef::SetUsed)
      .def("GetType", &ContextRef::GetType)
      .def("GetMetaValue", &ContextRef::GetMetaValue)
      .def("SetMetaValue", &ContextRef::SetMetaValue)
      .def("GetMetaKeys", &ContextRef::GetMetaKeys)
      .def("GetSerializationType", &ContextRef::GetSerializationType)
      .def("SetSerializationType", &ContextRef::SetSerializationType)
      .def("ToString", &ContextRef::ToString);
}

inline bool PyRegisterPublishType(
    aimrt::channel::PublisherRef& publisher_ref,
    const std::shared_ptr<const PyTypeSupport>& msg_type_support) {
  static std::vector<std::shared_ptr<const PyTypeSupport>> py_ts_vec;
  py_ts_vec.emplace_back(msg_type_support);

  return publisher_ref.RegisterPublishType(msg_type_support->NativeHandle());
}

inline bool PyRegisterPublishType(
    aimrt::channel::PublisherRef& publisher_ref,
    const std::shared_ptr<const PyRos2TypeSupport>& py_ros2_type_support) {
  static std::vector<std::shared_ptr<const PyRos2TypeSupport>> py_ros2_ts_vec;
  py_ros2_ts_vec.emplace_back(py_ros2_type_support);

  return publisher_ref.RegisterPublishType(py_ros2_type_support->NativeHandle());
}

inline void PyPublishWithSerializationType(
    aimrt::channel::PublisherRef& publisher_ref,
    std::string_view msg_type,
    std::string_view serialization_type,
    const std::string& msg_buf) {
  aimrt::channel::Context ctx;
  ctx.SetSerializationType(serialization_type);
  publisher_ref.Publish(msg_type, ctx, static_cast<const void*>(&msg_buf));
}

inline void PyPublishWithCtx(
    aimrt::channel::PublisherRef& publisher_ref,
    std::string_view msg_type,
    const aimrt::channel::ContextRef& ctx_ref,
    const std::string& msg_buf) {
  publisher_ref.Publish(msg_type, ctx_ref, static_cast<const void*>(&msg_buf));
}

inline std::unique_ptr<void, destroy_ros_message_function*>
create_from_py(pybind11::object pymessage) {
  typedef void* create_ros_message_function(void);

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

inline void PyPublishRos2MessageWithCtx(
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

inline void ExportPublisherRef(pybind11::object m) {
  using aimrt::channel::PublisherRef;

  using PyTsPtr = std::shared_ptr<const PyTypeSupport>;
  using PyRos2TsPtr = std::shared_ptr<const PyRos2TypeSupport>;

  pybind11::class_<PublisherRef>(std::move(m), "PublisherRef")
      .def(pybind11::init<>())
      .def("__bool__", &PublisherRef::operator bool)
      .def("RegisterPublishType",
           pybind11::overload_cast<PublisherRef&, const PyTsPtr&>(&PyRegisterPublishType))
      .def("RegisterPublishType",
           pybind11::overload_cast<PublisherRef&, const PyRos2TsPtr&>(&PyRegisterPublishType))
      .def("PublishWithSerializationType", &PyPublishWithSerializationType)
      .def("PublishWithCtx", &PyPublishWithCtx)
      .def("PublishRos2MessageWithCtx", &PyPublishRos2MessageWithCtx)
      .def("GetTopic", &PublisherRef::GetTopic)
      .def("MergeSubscribeContextToPublishContext", &PublisherRef::MergeSubscribeContextToPublishContext);
}

inline bool PySubscribeWithCtx(
    aimrt::channel::SubscriberRef& subscriber_ref,
    const std::shared_ptr<const PyTypeSupport>& msg_type_support,
    std::function<void(aimrt::channel::ContextRef, const pybind11::bytes&)>&& callback) {
  static std::vector<std::shared_ptr<const PyTypeSupport>> py_ts_vec;
  py_ts_vec.emplace_back(msg_type_support);

  return subscriber_ref.Subscribe(
      msg_type_support->NativeHandle(),
      [callback{std::move(callback)}](
          const aimrt_channel_context_base_t* ctx_ptr,
          const void* msg_ptr,
          aimrt_function_base_t* release_callback_base) {
        aimrt::channel::SubscriberReleaseCallback release_callback(release_callback_base);

        const std::string& msg_buf = *static_cast<const std::string*>(msg_ptr);
        auto ctx_ref = aimrt::channel::ContextRef(ctx_ptr);

        pybind11::gil_scoped_acquire acquire;

        auto msg_buf_bytes = pybind11::bytes(msg_buf);
        callback(ctx_ref, msg_buf_bytes);
        msg_buf_bytes.release();

        pybind11::gil_scoped_release release;

        release_callback();
      });
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

inline bool PySubscribeRos2MessageWithCtx(
    aimrt::channel::SubscriberRef& subscriber_ref,
    const std::shared_ptr<const PyRos2TypeSupport>& py_ros2_type_support,
    pybind11::object pyclass,
    std::function<void(aimrt::channel::ContextRef, pybind11::object)>&& callback) {
  static std::vector<std::shared_ptr<const PyRos2TypeSupport>> py_ros2_ts_vec;
  py_ros2_ts_vec.emplace_back(py_ros2_type_support);

  return subscriber_ref.Subscribe(
      py_ros2_type_support->NativeHandle(),
      [callback{std::move(callback)}, pyclass{pyclass}](
          const aimrt_channel_context_base_t* ctx_ptr,
          const void* msg_ptr,
          aimrt_function_base_t* release_callback_base) {
        aimrt::channel::SubscriberReleaseCallback release_callback(release_callback_base);

        pybind11::gil_scoped_acquire acquire;
        auto msg_obj = convert_to_py(const_cast<void*>(msg_ptr), pyclass);
        if (!msg_obj) {
          throw py::error_already_set();
        }

        auto ctx_ref = aimrt::channel::ContextRef(ctx_ptr);

        callback(ctx_ref, msg_obj);
        pybind11::gil_scoped_release release;

        release_callback();
      });
}

inline void ExportSubscriberRef(pybind11::object m) {
  using aimrt::channel::SubscriberRef;

  pybind11::class_<SubscriberRef>(std::move(m), "SubscriberRef")
      .def(pybind11::init<>())
      .def("__bool__", &SubscriberRef::operator bool)
      .def("SubscribeWithCtx", &PySubscribeWithCtx)
      .def("SubscribeRos2MessageWithCtx", &PySubscribeRos2MessageWithCtx)
      .def("GetTopic", &SubscriberRef::GetTopic);
}

inline void ExportChannelHandleRef(pybind11::object m) {
  using aimrt::channel::ChannelHandleRef;

  pybind11::class_<ChannelHandleRef>(std::move(m), "ChannelHandleRef")
      .def(pybind11::init<>())
      .def("__bool__", &ChannelHandleRef::operator bool)
      .def("GetPublisher", &ChannelHandleRef::GetPublisher)
      .def("GetSubscriber", &ChannelHandleRef::GetSubscriber)
      .def("MergeSubscribeContextToPublishContext", &ChannelHandleRef::MergeSubscribeContextToPublishContext);
}
}  // namespace aimrt::runtime::python_runtime
