// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once
#include "aimrt_module_cpp_interface/context/context.h"  // NOLINT(misc-unused-include)
#include "context/details/concepts.h"
#include "context/res/channel.h"
#include "executor/executor.h"

namespace aimrt::context::init {

// Use Context symbol to satisfy include-what-you-use analysis
using AimrtInitContextType = aimrt::context::Context;

template <concepts::DirectlySupportedType T>
context::res::Publisher<T> CreatePublisher(const std::string_view& topic_name, std::source_location loc = std::source_location::current()) {
  return context::details::ExpectContext(loc)->pub(loc).Init<T>(topic_name);
}

template <concepts::DirectlySupportedType T>
context::res::Subscriber<T> CreateSubscriber(const std::string_view& topic_name, std::source_location loc = std::source_location::current()) {
  return context::details::ExpectContext(loc)->sub(loc).Init<T>(topic_name);
}

template <concepts::DirectlySupportedType T, concepts::SupportedSubscriber<T> TCallback>
context::res::Subscriber<T> CreateSubscriber(const std::string_view& topic_name, TCallback&& callback, std::source_location loc = std::source_location::current()) {
  auto sub = CreateSubscriber<T>(topic_name, loc);
  sub.SubscribeInline(std::forward<TCallback>(callback));
  return sub;
}

template <concepts::DirectlySupportedType T, concepts::SupportedSubscriber<T> TCallback>
context::res::Subscriber<T> CreateSubscriber(const std::string_view& topic_name, const aimrt::executor::ExecutorRef& exe, TCallback&& callback, std::source_location loc = std::source_location::current()) {
  auto sub = CreateSubscriber<T>(topic_name, loc);
  sub.SubscribeOn(exe, std::forward<TCallback>(callback));
  return sub;
}

inline aimrt::executor::ExecutorRef CreateExecutor(const std::string_view& name, std::source_location loc = std::source_location::current()) {
  return context::details::ExpectContext(loc)->CreateExecutor(name, loc);
}

// Compatibility aliases for existing call sites
template <concepts::DirectlySupportedType T>
inline context::res::Subscriber<T> Subscriber(const std::string_view& topic_name, std::source_location loc = std::source_location::current()) {
  return CreateSubscriber<T>(topic_name, loc);
}

template <concepts::DirectlySupportedType T, concepts::SupportedSubscriber<T> TCallback>
inline context::res::Subscriber<T> Subscriber(const std::string_view& topic_name, TCallback&& callback, std::source_location loc = std::source_location::current()) {
  return CreateSubscriber<T>(topic_name, std::forward<TCallback>(callback), loc);
}

template <concepts::DirectlySupportedType T, concepts::SupportedSubscriber<T> TCallback>
inline context::res::Subscriber<T> Subscriber(const std::string_view& topic_name, const aimrt::executor::ExecutorRef& exe, TCallback&& callback, std::source_location loc = std::source_location::current()) {
  return CreateSubscriber<T>(topic_name, exe, std::forward<TCallback>(callback), loc);
}

inline aimrt::executor::ExecutorRef GetExecutor(const std::string_view& name, std::source_location loc = std::source_location::current()) {
  return CreateExecutor(name, loc);
}

}  // namespace aimrt::context::init