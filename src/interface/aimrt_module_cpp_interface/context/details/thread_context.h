// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <aimrt_module_cpp_interface/executor/executor.h>
#include <aimrt_module_cpp_interface/rpc/rpc_context.h>

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

inline thread_local ThreadContext g_thread_ctx{};

}  // namespace aimrt::context::details
