// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <any>
#include <memory>
#include <string>
#include <string_view>
#include <utility>

#include "aimrt_module_cpp_interface/channel/channel_context.h"
#include "aimrt_module_cpp_interface/channel/channel_handle.h"
#include "aimrt_module_cpp_interface/co/sync_wait.h"
#include "aimrt_module_cpp_interface/co/task.h"
#include "aimrt_module_cpp_interface/context/channel_context.h"
#include "aimrt_module_cpp_interface/context/details/concepts.h"
#include "aimrt_module_cpp_interface/context/details/thread_context.h"
#include "aimrt_module_cpp_interface/context/details/type_support.h"
#include "aimrt_module_cpp_interface/context/op_base.h"
#include "aimrt_module_cpp_interface/context/res/channel.h"
#include "aimrt_module_cpp_interface/executor/executor.h"

namespace aimrt::context {

class OpSub : public OpBase {
 public:
  using OpBase::OpBase;

  OpSub(Context& ctx, std::source_location loc) noexcept : OpBase(ctx, loc) {}

  template <concepts::DirectlySupportedType T>
  [[nodiscard]] res::Channel<T> Init(std::string_view topic_name);

  template <class T, concepts::SupportedSubscriber<T> TCallback>
  void SubscribeInline(const res::Channel<T>& ch, TCallback callback);

  template <class T, concepts::SupportedSubscriber<T> TCallback>
  void SubscribeOn(const res::Channel<T>& ch, aimrt::executor::ExecutorRef exe, TCallback callback);

 private:
  template <class T>
  std::pair<res::Channel<T>, ChannelContext&> DoInit(std::string_view topic_name);

  template <concepts::DirectlySupportedType T>
  static typename Context::SubscribeFunction<T> CreateSubscribeFunction();

  template <class T, concepts::SupportedSubscriber<T> TCallback>
  void DoSubscribe(const res::Channel<T>& ch, TCallback callback, aimrt::executor::ExecutorRef exe);

  template <concepts::DirectlySupportedType T, class F>
  static bool RawSubscribe(
      aimrt::channel::SubscriberRef subscriber,
      std::weak_ptr<Context> ctx_weak,
      aimrt::executor::ExecutorRef exe,
      F&& callback);

  template <class T, concepts::SupportedSubscriber<T> F>
  static auto StandardizeSubscriber(F&& cb);
};

template <concepts::DirectlySupportedType T>
res::Channel<T> OpSub::Init(std::string_view topic_name) {
  auto [channel, channel_ctx] = DoInit<T>(topic_name);
  channel_ctx.sub_f = CreateSubscribeFunction<T>();
  return channel;
}

}  // namespace aimrt::context
