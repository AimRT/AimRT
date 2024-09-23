// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_cpp_interface/rpc/rpc_co_filter.h"
#include "normal_rpc_co_server_module/global.h"

namespace aimrt::examples::cpp::ros2_rpc::normal_rpc_co_server_module {

inline co::Task<aimrt::rpc::Status> TimeCostLogServerFilter(
    aimrt::rpc::ContextRef ctx, const void* req_ptr, void* rsp_ptr,
    const aimrt::rpc::CoRpcHandle& next) {
  auto begin_time = std::chrono::steady_clock::now();
  const auto& status = co_await next(ctx, req_ptr, rsp_ptr);
  auto end_time = std::chrono::steady_clock::now();

  AIMRT_INFO("Svr rpc time cost {} us",
             std::chrono::duration_cast<std::chrono::microseconds>(end_time - begin_time).count());

  co_return status;
}

}  // namespace aimrt::examples::cpp::ros2_rpc::normal_rpc_co_server_module
