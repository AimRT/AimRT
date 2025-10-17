// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <aimrt_module_cpp_interface/executor/executor.h>
#include <aimrt_module_cpp_interface/rpc/rpc_context.h>
#include <util/exception.h>

#include <memory>
#include <optional>

namespace aimrt::context {
class Context;
}  // namespace aimrt::context

namespace aimrt::context::details {

struct ThreadContext {
  std::weak_ptr<Context> ctx_ptr;
  aimrt::executor::ExecutorRef exe;
  std::optional<aimrt::rpc::ContextRef> active_rpc_context;
};

extern thread_local ThreadContext g_thread_ctx;

inline std::shared_ptr<Context> ExpectContext(const std::source_location& call_loc)
{
  const std::shared_ptr ctx_ptr = g_thread_ctx.ctx_ptr.lock();

  if (ctx_ptr != nullptr) [[likely]]
    return ctx_ptr;

  AIMRT_ASSERT(ctx_ptr != nullptr, "Broken context !");

}

}  // namespace aimrt::context::details
