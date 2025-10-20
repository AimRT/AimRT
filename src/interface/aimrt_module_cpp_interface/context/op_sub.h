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
  [[nodiscard]] res::Subscriber<T> Init(std::string_view topic_name);

  template <class T, concepts::SupportedSubscriber<T> TCallback>
  void SubscribeInline(const res::Subscriber<T>& ch, TCallback callback);

  template <class T, concepts::SupportedSubscriber<T> TCallback>
  void SubscribeOn(const res::Subscriber<T>& ch, aimrt::executor::ExecutorRef exe, TCallback callback);

 private:
  template <class T>
  std::pair<res::Subscriber<T>, ChannelContext&> DoInit(std::string_view topic_name);

  template <concepts::DirectlySupportedType T>
  static typename Context::SubscribeFunction<T> CreateSubscribeFunction();

  template <class T, concepts::SupportedSubscriber<T> TCallback>
  void DoSubscribe(const res::Subscriber<T>& ch, TCallback callback, aimrt::executor::ExecutorRef exe);

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
res::Subscriber<T> OpSub::Init(std::string_view topic_name) {
  auto [subscriber, channel_ctx] = DoInit<T>(topic_name);
  channel_ctx.sub_f = CreateSubscribeFunction<T>();
  return subscriber;
}

}  // namespace aimrt::context
