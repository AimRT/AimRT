// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <functional>

#include "aimrt_module_cpp_interface/co/task.h"
#include "aimrt_module_cpp_interface/rpc/rpc_context.h"
#include "aimrt_module_cpp_interface/rpc/rpc_status.h"

namespace aimrt::rpc {

using CoRpcHandle = std::function<co::Task<Status>(ContextRef, const void*, void*)>;
using CoRpcFilter = std::function<co::Task<Status>(ContextRef, const void*, void*, const CoRpcHandle&)>;

class CoFilterManager {
 public:
  CoFilterManager()
      : final_filter_([](ContextRef ctx_ref, const void* req, void* rsp, const CoRpcHandle& h) -> aimrt::co::Task<Status> {
          return h(ctx_ref, req, rsp);
        }) {}
  ~CoFilterManager() = default;

  CoFilterManager(const CoFilterManager&) = delete;
  CoFilterManager& operator=(const CoFilterManager&) = delete;

  void RegisterFilter(CoRpcFilter&& filter) {
    final_filter_ =
        [final_filter{std::move(final_filter_)}, cur_filter{std::move(filter)}](
            ContextRef ctx_ref, const void* req, void* rsp, const CoRpcHandle& h) -> aimrt::co::Task<Status> {
      co_return co_await cur_filter(
          ctx_ref, req, rsp,
          [&final_filter, &h](ContextRef ctx_ref, const void* req, void* rsp) -> aimrt::co::Task<Status> {
            return final_filter(ctx_ref, req, rsp, h);
          });
    };
  }

  aimrt::co::Task<Status> InvokeRpc(const CoRpcHandle& h, ContextRef ctx_ref, const void* req, void* rsp) const {
    return final_filter_(ctx_ref, req, rsp, h);
  }

  void Clear() {
    final_filter_ = CoRpcFilter();
  }

 private:
  CoRpcFilter final_filter_;
};

}  // namespace aimrt::rpc
