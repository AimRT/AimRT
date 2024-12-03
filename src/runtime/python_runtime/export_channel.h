// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "pybind11/pybind11.h"

#include <utility>

#include "aimrt_module_cpp_interface/channel/channel_context.h"
#include "aimrt_module_cpp_interface/channel/channel_handle.h"

#include "python_runtime/export_pb_type_support.h"

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
      .def(pybind11::init<const Context&>(), pybind11::keep_alive<1, 2>())
      .def(pybind11::init<const Context*>(), pybind11::keep_alive<1, 2>())
      .def(pybind11::init<const std::shared_ptr<Context>&>(), pybind11::keep_alive<1, 2>())
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

inline bool PbRegisterPublishType(
    aimrt::channel::PublisherRef& publisher_ref,
    const std::shared_ptr<const PyPbTypeSupport>& msg_type_support) {
  static std::vector<std::shared_ptr<const PyPbTypeSupport>> py_pb_ts_vec;
  py_pb_ts_vec.emplace_back(msg_type_support);

  return publisher_ref.RegisterPublishType(msg_type_support->NativeHandle());
}

inline void PbPublishWithCtx(
    aimrt::channel::PublisherRef& publisher_ref,
    std::string_view msg_type,
    const aimrt::channel::ContextRef& ctx_ref,
    const std::string& msg_buf) {
  publisher_ref.Publish(msg_type, ctx_ref, static_cast<const void*>(&msg_buf));
}

inline bool PbSubscribeWithCtx(
    aimrt::channel::SubscriberRef& subscriber_ref,
    const std::shared_ptr<const PyPbTypeSupport>& msg_type_support,
    std::function<void(aimrt::channel::ContextRef, const pybind11::bytes&)>&& callback) {
  static std::vector<std::shared_ptr<const PyPbTypeSupport>> py_ts_vec;
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

inline void ExportPublisherRef(pybind11::object m) {
  using aimrt::channel::PublisherRef;

  pybind11::class_<PublisherRef>(std::move(m), "PublisherRef")
      .def(pybind11::init<>())
      .def("__bool__", &PublisherRef::operator bool)
      .def("GetTopic", &PublisherRef::GetTopic)
      .def("MergeSubscribeContextToPublishContext", &PublisherRef::MergeSubscribeContextToPublishContext)
      .def("PbPublishWithCtx", &PbPublishWithCtx)
      .def("PbRegisterPublishType", &PbRegisterPublishType);
}

inline void ExportSubscriberRef(pybind11::object m) {
  using aimrt::channel::SubscriberRef;

  pybind11::class_<SubscriberRef>(std::move(m), "SubscriberRef")
      .def(pybind11::init<>())
      .def("__bool__", &SubscriberRef::operator bool)
      .def("GetTopic", &SubscriberRef::GetTopic)
      .def("PbSubscribeWithCtx", &PbSubscribeWithCtx);
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

#ifdef AIMRT_BUILD_WITH_ROS2

#endif  // AIMRT_BUILD_WITH_ROS2

}  // namespace aimrt::runtime::python_runtime
