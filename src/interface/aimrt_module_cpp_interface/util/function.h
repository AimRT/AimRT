// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstddef>
#include <functional>
#include <new>
#include <type_traits>
#include <utility>

#include "aimrt_module_c_interface/util/function_base.h"

namespace aimrt::util {

template <typename>
class Function;

template <typename>
struct InvokerTraitsHelper;

template <typename R, typename... Args>
struct InvokerTraitsHelper<R (*)(void*, Args...)> {
  static constexpr size_t kArgCount = sizeof...(Args);
  using ReturnType = R;
  using ArgsTuple = std::tuple<Args...>;
};

template <typename T>
concept DecayedType = std::is_same_v<std::decay_t<T>, T>;

// TODO: 参数类型也要是退化的（纯C Style，不可有引用、stl容器等）
template <typename T>
concept FunctionCStyleOps =
    std::is_same_v<void (*)(void*, void*), decltype(T::relocator)> &&
    std::is_same_v<void (*)(void*), decltype(T::destroyer)> &&
    DecayedType<typename InvokerTraitsHelper<decltype(T::invoker)>::ReturnType>;

/**
 * @brief Function
 * @note 由定义好的C Style类型ops构造，可以直接使用NativeHandle与C互调
 *
 * @tparam Ops
 */
template <FunctionCStyleOps Ops>
class Function<Ops> {
 public:
  using OpsType = Ops;
  using InvokerType = decltype(OpsType::invoker);

 private:
  using InvokerTypeHelper = InvokerTraitsHelper<decltype(OpsType::invoker)>;
  using R = typename InvokerTypeHelper::ReturnType;
  using ArgsTuple = typename InvokerTypeHelper::ArgsTuple;
  using Indices = std::make_index_sequence<InvokerTypeHelper::kArgCount>;

 public:
  Function() { base_.ops = nullptr; }
  Function(std::nullptr_t) { base_.ops = nullptr; }

  ~Function() {
    if (base_.ops)
      static_cast<const OpsType*>(base_.ops)->destroyer(&(base_.object_buf));
  }

  Function(Function&& function) noexcept {
    base_.ops = std::exchange(function.base_.ops, nullptr);
    if (base_.ops)
      static_cast<const OpsType*>(base_.ops)
          ->relocator(&(function.base_.object_buf), &(base_.object_buf));
  }

  Function(aimrt_function_base_t* function_base) {
    if (function_base == nullptr) [[unlikely]] {
      base_.ops = nullptr;
      return;
    }

    base_.ops = std::exchange(function_base->ops, nullptr);
    if (base_.ops)
      static_cast<const OpsType*>(base_.ops)
          ->relocator(&(function_base->object_buf), &(base_.object_buf));
  }

  Function& operator=(Function&& function) noexcept {
    if (&function != this) {
      this->~Function();
      new (this) Function(std::move(function));
    }
    return *this;
  }

  template <class T, size_t... Idx>
  static constexpr bool CheckImplicitlyConvertible(
      std::index_sequence<Idx...>) {
    return std::is_invocable_r_v<R, T, std::tuple_element_t<Idx, ArgsTuple>...> &&
           !std::is_same_v<std::decay_t<T>, Function>;
  }

  template <class T,
            class = std::enable_if_t<CheckImplicitlyConvertible<T>(Indices{})>>
  Function(T&& action) {
    if constexpr (std::is_assignable<T, std::nullptr_t>::value) {
      if (action == nullptr) {
        base_.ops = nullptr;
        return;
      }
    }

    ConstructorImpl<T>(std::forward<T>(action), Indices{});
  }

  template <class T,
            class = std::enable_if_t<CheckImplicitlyConvertible<T>(Indices{})>>
  Function& operator=(T&& action) {
    this->~Function();
    new (this) Function(std::forward<T>(action));
    return *this;
  }

  Function& operator=(std::nullptr_t) {
    if (const auto* ops = std::exchange(base_.ops, nullptr)) {
      static_cast<const OpsType*>(ops)->destroyer(&(base_.object_buf));
    }
    return *this;
  }

  // TODO：加上Args的类型限制，参数类型也要是退化的（纯C Style，不可有引用、stl容器等）
  template <typename... Args>
  R operator()(Args... args) const noexcept {
    return static_cast<const OpsType*>(base_.ops)->invoker(&(base_.object_buf), args...);
  }

  explicit operator bool() const { return (base_.ops != nullptr); }

  aimrt_function_base_t* NativeHandle() { return &base_; }

 private:
  template <class T, size_t... Idx>
  void ConstructorImpl(T&& action, std::index_sequence<Idx...>) {
    using Decayed = std::decay_t<T>;

    if constexpr (sizeof(Decayed) <= sizeof(base_.object_buf)) {
      static constexpr OpsType kOps = {
          .invoker = [](void* object, std::tuple_element_t<Idx, ArgsTuple>... args) noexcept -> R {
            if constexpr (std::is_void_v<R>) {
              std::invoke(*static_cast<Decayed*>(object), args...);
            } else {
              return std::invoke(*static_cast<Decayed*>(object), args...);
            }
          },
          .relocator = [](void* from, void* to) noexcept {
                new (to) Decayed(std::move(*static_cast<Decayed*>(from)));
                static_cast<Decayed*>(from)->~Decayed(); },
          .destroyer = [](void* object) noexcept { static_cast<Decayed*>(object)->~Decayed(); }};

      base_.ops = &kOps;
      new (&(base_.object_buf)) Decayed(std::forward<T>(action));
    } else {
      using Stored = Decayed*;

      static constexpr OpsType kOps = {
          .invoker = [](void* object, std::tuple_element_t<Idx, ArgsTuple>... args) noexcept -> R {
            if constexpr (std::is_void_v<R>) {
              std::invoke(**static_cast<Stored*>(object), args...);
            } else {
              return std::invoke(**static_cast<Stored*>(object), args...);
            }
          },
          .relocator = [](void* from, void* to) noexcept { new (to) Stored(*static_cast<Stored*>(from)); },
          .destroyer = [](void* object) noexcept { delete *static_cast<Stored*>(object); }};

      base_.ops = &kOps;
      new (&(base_.object_buf)) Stored(new Decayed(std::forward<T>(action)));
    }
  }

 private:
  mutable aimrt_function_base_t base_;
};

namespace details {

template <class>
struct FunctionTypeDeducer;

template <class T>
using FunctionSignature = typename FunctionTypeDeducer<T>::Type;

template <class R, class Class, class... Args>
struct FunctionTypeDeducer<R (Class::*)(Args...)> {
  using Type = R(Args...);
};

template <class R, class Class, class... Args>
struct FunctionTypeDeducer<R (Class::*)(Args...)&> {
  using Type = R(Args...);
};

template <class R, class Class, class... Args>
struct FunctionTypeDeducer<R (Class::*)(Args...) const> {
  using Type = R(Args...);
};

template <class R, class Class, class... Args>
struct FunctionTypeDeducer<R (Class::*)(Args...) const&> {
  using Type = R(Args...);
};

}  // namespace details

template <class R, class... Args>
Function(R (*)(Args...)) -> Function<R(Args...)>;

template <class F, class Signature =
                       details::FunctionSignature<decltype(&F::operator())>>
Function(F) -> Function<Signature>;

template <class T>
bool operator==(const Function<T>& f, std::nullptr_t) {
  return !f;
}

template <class T>
bool operator==(std::nullptr_t, const Function<T>& f) {
  return !f;
}

template <class T>
bool operator!=(const Function<T>& f, std::nullptr_t) {
  return !(f == nullptr);
}

template <class T>
bool operator!=(std::nullptr_t, const Function<T>& f) {
  return !(f == nullptr);
}

}  // namespace aimrt::util
