// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_cpp_interface/channel/channel_context.h"
#include "aimrt_module_cpp_interface/context/details/concepts.h"
#include "aimrt_module_cpp_interface/context/res/base.h"
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
  void Publish(const T& msg, std::source_location loc = std::source_location::current()) const;
  void Publish(aimrt::channel::ContextRef ch_ctx, const T& msg, std::source_location loc = std::source_location::current()) const;
};

template <class T>
class Subscriber : public Channel<T> {
 public:
  template <concepts::SupportedSubscriber<T> TCallback>
  void SubscribeOn(
      const aimrt::executor::ExecutorRef& exe, TCallback callback, std::source_location loc = std::source_location::current()) const;

  template <concepts::SupportedSubscriber<T> TCallback>
  void SubscribeInline(TCallback callback, std::source_location loc = std::source_location::current()) const;
};

}  // namespace aimrt::context::res
