// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_cpp_interface/channel/channel_context.h"
#include "aimrt_module_cpp_interface/context/details/concepts.h"
#include "aimrt_module_cpp_interface/context/res/details/base.h"
#include "aimrt_module_cpp_interface/executor/executor.h"
namespace aimrt::context::res {

template <class T>
class Channel : public details::Base {
 public:
  using MessageType = T;
  using details::Base::Base;
};

template <class T>
class Publisher : public Channel<T> {
 public:
  void Publish(const T& msg) const;
  void Publish(aimrt::channel::ContextRef ch_ctx, const T& msg) const;
};

template <class T>
class Subscriber : public Channel<T> {
 public:
  template <concepts::SupportedSubscriber<T> TCallback>
  void SubscribeOn(
      const aimrt::executor::ExecutorRef& exe, TCallback callback) const;

  template <concepts::SupportedSubscriber<T> TCallback>
  void SubscribeInline(TCallback callback) const;
};

}  // namespace aimrt::context::res
