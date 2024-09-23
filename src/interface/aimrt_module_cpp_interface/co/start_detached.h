// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#ifdef AIMRT_EXECUTOR_USE_STDEXEC

  #include <stdexec/execution.hpp>

namespace aimrt::co {

inline constexpr auto& StartDetached = stdexec::start_detached;

}

#else

  #include <unifex/async_scope.hpp>
  #include <unifex/sync_wait.hpp>
  #include <unifex/then.hpp>

namespace aimrt::co {

/**
 * @brief Detach executes a coroutine. Use a global async_scope
 *
 * @tparam Sender
 * @param sender
 */
template <typename Sender>
  requires unifex::sender<Sender>
inline void StartDetached(Sender&& sender) {
  struct AsyncScopeDeleter {
    void operator()(unifex::async_scope* p) {
      unifex::sync_wait(p->cleanup());
      delete p;
    }
  };
  static std::unique_ptr<unifex::async_scope, AsyncScopeDeleter> scope_ptr{
      new unifex::async_scope()};
  scope_ptr->spawn((Sender &&) sender);
}

}  // namespace aimrt::co

#endif
