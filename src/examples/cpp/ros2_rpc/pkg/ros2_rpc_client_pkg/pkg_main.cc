// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <cstring>

#include "aimrt_pkg_c_interface/pkg_macro.h"
#include "benchmark_rpc_client_module/benchmark_rpc_client_module.h"
#include "normal_rpc_async_client_module/normal_rpc_async_client_module.h"
#include "normal_rpc_co_client_module/normal_rpc_co_client_module.h"
#include "normal_rpc_future_client_module/normal_rpc_future_client_module.h"
#include "normal_rpc_sync_client_module/normal_rpc_sync_client_module.h"

using namespace aimrt::examples::cpp::ros2_rpc;

static std::tuple<std::string_view, std::function<aimrt::ModuleBase*()>> aimrt_module_register_array[]{
    {"NormalRpcCoClientModule", []() -> aimrt::ModuleBase* {
       return new normal_rpc_co_client_module::NormalRpcCoClientModule();
     }},
    {"NormalRpcAsyncClientModule", []() -> aimrt::ModuleBase* {
       return new normal_rpc_async_client_module::NormalRpcAsyncClientModule();
     }},
    {"NormalRpcSyncClientModule", []() -> aimrt::ModuleBase* {
       return new normal_rpc_sync_client_module::NormalRpcSyncClientModule();
     }},
    {"NormalRpcFutureClientModule", []() -> aimrt::ModuleBase* {
       return new normal_rpc_future_client_module::NormalRpcFutureClientModule();
     }},
    {"BenchmarkRpcClientModule", []() -> aimrt::ModuleBase* {
       return new benchmark_rpc_client_module::BenchmarkRpcClientModule();
     }}};

AIMRT_PKG_MAIN(aimrt_module_register_array)
