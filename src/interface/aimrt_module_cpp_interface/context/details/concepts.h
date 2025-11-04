// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <concepts>
#include <functional>
#include <memory>
#include <type_traits>
#include <utility>

#include "aimrt_module_cpp_interface/channel/channel_context.h"
#include "aimrt_module_cpp_interface/context/res/service.h"

namespace aimrt::context::concepts {

template <class R, class F, class... Args>
concept CallableR =
    std::invocable<F, Args...> &&
    (std::is_void_v<R> || std::convertible_to<std::invoke_result_t<F, Args...>, R>);

template <class F, class Signature>
struct FunctionMatcher;

template <class F, class R, class... Args>
struct FunctionMatcher<F, R(Args...)> {
  static constexpr bool value = CallableR<R, F, Args...>;
};

template <class F, class Signature>
concept Function = FunctionMatcher<F, Signature>::value;

template <class F, class T>
concept SubscriberFunction = Function<F, void(std::shared_ptr<const T>)>;

template <class F, class T>
concept SubscriberFunctionDeref = Function<F, void(const T &)>;

template <class F, class T>
concept SubscriberFunctionWithCtx = Function<F, void(aimrt::channel::ContextRef, std::shared_ptr<const T>)>;

template <class F, class T>
concept SubscriberFunctionDerefWithCtx = Function<F, void(aimrt::channel::ContextRef, const T &)>;

template <class F, class T>
concept SupportedSubscriber =
    SubscriberFunction<F, T> ||
    SubscriberFunctionDeref<F, T> ||
    SubscriberFunctionWithCtx<F, T> ||
    SubscriberFunctionDerefWithCtx<F, T>;

template <class F, class Q, class P>
concept RawClient =
    Function<F, aimrt::co::Task<aimrt::rpc::Status>(aimrt::rpc::ContextRef, const Q &, P &)>;

}  // namespace aimrt::context::concepts
