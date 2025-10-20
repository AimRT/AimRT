// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <any>
#include <string>
#include <string_view>
#include <utility>

#include "aimrt_module_cpp_interface/channel/channel_context.h"
#include "aimrt_module_cpp_interface/channel/channel_handle.h"
#include "aimrt_module_cpp_interface/context/channel_context.h"
#include "aimrt_module_cpp_interface/context/details/concepts.h"
#include "aimrt_module_cpp_interface/context/details/type_support.h"
#include "aimrt_module_cpp_interface/context/op_base.h"
#include "aimrt_module_cpp_interface/context/res/channel.h"

namespace aimrt::context {

class OpPub : public OpBase {
 public:
  using OpBase::OpBase;

  OpPub(Context& ctx, std::source_location loc) noexcept : OpBase(ctx, loc) {}

  template <concepts::DirectlySupportedType T>
  [[nodiscard]] res::Publisher<T> Init(std::string_view topic_name);

  template <class T>
  void Publish(const res::Channel<T>& ch, const T& msg);

  template <class T>
  void Publish(const res::Channel<T>& ch, aimrt::channel::ContextRef ch_ctx, const T& msg);

 private:
  template <class T>
  std::pair<res::Publisher<T>, ChannelContext&> DoInit(std::string_view topic_name);

  template <concepts::DirectlySupportedType T>
  static typename Context::PublishFunction<T> CreatePublishFunction();
};

template <concepts::DirectlySupportedType T>
res::Publisher<T> OpPub::Init(std::string_view topic_name) {
  auto [channel, channel_ctx] = DoInit<T>(topic_name);
  channel_ctx.pub_f = CreatePublishFunction<T>();
  return channel;
}

template <class T>
void OpPub::Publish(const res::Channel<T>& ch, const T& msg) {
  aimrt::channel::Context ctx;
  Publish(ch, ctx, msg);
}

template <concepts::DirectlySupportedType T>
typename Context::PublishFunction<T> OpPub::CreatePublishFunction() {
  return [](aimrt::channel::PublisherRef publisher, aimrt::channel::ContextRef ctx_ref, const T& msg) {
    aimrt::channel::Publish(publisher, ctx_ref, msg);
  };
}

}  // namespace aimrt::context
