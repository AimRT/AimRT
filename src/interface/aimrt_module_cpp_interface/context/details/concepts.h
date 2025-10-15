// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <aimrt_module_protobuf_interface/util/protobuf_tools.h>

#ifdef AIMRT_BUILD_WITH_ROS2
  #include <rosidl_generator_traits/message_traits.hpp>
  #include <rosidl_runtime_cpp/traits.hpp>
#endif

#include <concepts>
#include <functional>
#include <memory>
#include <type_traits>
#include <utility>

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
concept SubscriberFunctionDeref = Function<F, void(const T&)>;


template <class F, class T>
concept SupportedSubscriber =
    SubscriberFunction<F, T> || SubscriberFunctionDeref<F, T>;

#ifdef AIMRT_BUILD_WITH_ROS2
template <class T>
concept RosMessage = rosidl_generator_traits::is_message<T>::value;
#else
template <class T>
concept RosMessage = false;
#endif

template <class T>
concept Protobuf = std::derived_from<T, google::protobuf::Message>;

template <class T>
concept DirectlySupportedType = RosMessage<T> || Protobuf<T>;

}  // namespace aimrt::context::concepts
