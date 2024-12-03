// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <cstring>

#include "aimrt_pkg_c_interface/pkg_macro.h"
#include "normal_rpc_async_server_module/normal_rpc_async_server_module.h"
#include "normal_rpc_co_server_module/normal_rpc_co_server_module.h"
#include "normal_rpc_sync_server_module/normal_rpc_sync_server_module.h"
#include "proxy_rpc_co_module/proxy_rpc_co_module.h"
using namespace aimrt::examples::cpp::pb_rpc;

static std::tuple<std::string_view, std::function<aimrt::ModuleBase*()>> aimrt_module_register_array[]{
    {"NormalRpcCoServerModule", []() -> aimrt::ModuleBase* {
       return new normal_rpc_co_server_module::NormalRpcCoServerModule();
     }},
    {"ProxyRpcCoModule", []() -> aimrt::ModuleBase* {
       return new proxy_rpc_co_module::ProxyRpcCoModule();
     }},
    {"NormalRpcAsyncServerModule", []() -> aimrt::ModuleBase* {
       return new normal_rpc_async_server_module::NormalRpcAsyncServerModule();
     }},
    {"NormalRpcSyncServerModule", []() -> aimrt::ModuleBase* {
       return new normal_rpc_sync_server_module::NormalRpcSyncServerModule();
     }}};

AIMRT_PKG_MAIN(aimrt_module_register_array)
