// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <utility>

#include "aimrt_module_cpp_interface/channel/channel_context.h"
#include "aimrt_module_cpp_interface/channel/channel_handle.h"
#include "python_runtime/export_type_support.h"

#include "pybind11/pybind11.h"

namespace aimrt::runtime::python_runtime {

inline void ExportContext(const pybind11::object& m) {
  using aimrt::channel::Context;
  using aimrt::channel::ContextRef;

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

inline void PyPublish(
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

inline void ExportPublisherRef(pybind11::object m) {
  using aimrt::channel::PublisherRef;

  pybind11::class_<PublisherRef>(std::move(m), "PublisherRef")
      .def(pybind11::init<>())
      .def("__bool__", &PublisherRef::operator bool)
      .def("RegisterPublishType", &PyRegisterPublishType)
      .def("Publish", &PyPublish)
      .def("PublishWithCtx", &PyPublishWithCtx)
      .def("GetTopic", &PublisherRef::GetTopic)
      .def("MergeSubscribeContextToPublishContext", &PublisherRef::MergeSubscribeContextToPublishContext);
}

inline bool PySubscribe(
    aimrt::channel::SubscriberRef& subscriber_ref,
    const std::shared_ptr<const PyTypeSupport>& msg_type_support,
    std::function<void(std::string_view, const pybind11::bytes&)>&& callback) {
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
        callback(ctx_ref.GetSerializationType(), msg_buf_bytes);
        msg_buf_bytes.release();

        pybind11::gil_scoped_release release;

        release_callback();
      });
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

inline void ExportSubscriberRef(pybind11::object m) {
  using aimrt::channel::SubscriberRef;

  pybind11::class_<SubscriberRef>(std::move(m), "SubscriberRef")
      .def(pybind11::init<>())
      .def("__bool__", &SubscriberRef::operator bool)
      .def("Subscribe", &PySubscribe)
      .def("SubscribeWithCtx", &PySubscribeWithCtx)
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
