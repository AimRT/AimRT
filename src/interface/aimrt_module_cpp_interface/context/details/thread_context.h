// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <aimrt_module_cpp_interface/executor/executor.h>
#include <aimrt_module_cpp_interface/rpc/rpc_context.h>
#include <util/exception.h>

#include <memory>
#include <optional>
#include <source_location>

namespace aimrt::context {
class Context;
}  // namespace aimrt::context

namespace aimrt::context::details {

struct ThreadContext {
  std::weak_ptr<Context> ctx_ptr;
  aimrt::executor::ExecutorRef exe;
  std::optional<aimrt::rpc::ContextRef> active_rpc_context;
};

inline thread_local ThreadContext g_thread_ctx;

inline std::shared_ptr<Context> GetCurrentContext(const std::source_location& call_loc = std::source_location::current()) {
  const std::shared_ptr ctx_ptr = g_thread_ctx.ctx_ptr.lock();

  if (ctx_ptr == nullptr) [[unlikely]]
    AIMRT_ASSERT(ctx_ptr != nullptr, "Broken context !");
  return ctx_ptr;
}

}  // namespace aimrt::context::details
