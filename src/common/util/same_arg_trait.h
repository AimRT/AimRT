// Copyright (c) 2024 The AimRT Authors.
// AimRT is licensed under Mulan PSL v2.

#pragma once

#include <concepts>
#include <tuple>

namespace aimrt::common::util {
template <typename T>
struct function_args;

template <typename R, typename... Args>
struct function_args<R(Args...)> {
  using type = std::tuple<Args...>;
};

template <typename R, typename C, typename... Args>
struct function_args<R (C::*)(Args...)> {
  using type = std::tuple<Args...>;
};

template <typename R, typename C, typename... Args>
struct function_args<R (C::*)(Args...) const> {
  using type = std::tuple<Args...>;
};

template <typename F>
struct function_args {
  using type = typename function_args<decltype(&F::operator())>::type;
};

template <typename F>
using function_args_t = typename function_args<F>::type;

template <typename F1, typename F2>
concept SameArguments = std::same_as<function_args_t<F1>, function_args_t<F2>>;

}  // namespace aimrt::common::util
