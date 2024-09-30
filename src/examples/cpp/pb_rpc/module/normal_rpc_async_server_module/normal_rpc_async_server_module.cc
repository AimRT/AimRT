// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "normal_rpc_async_server_module/normal_rpc_async_server_module.h"
#include "normal_rpc_async_server_module/global.h"

namespace aimrt::examples::cpp::pb_rpc::normal_rpc_async_server_module {

bool NormalRpcAsyncServerModule::Initialize(aimrt::CoreRef core) {
  core_ = core;

  SetLogger(core_.GetLogger());

  try {
    // Create service
    service_ptr_ = std::make_shared<ExampleServiceAsyncServiceImpl>();

    // Register service
    bool ret = core_.GetRpcHandle().RegisterService(service_ptr_.get());
    AIMRT_CHECK_ERROR_THROW(ret, "Register service failed.");

    AIMRT_INFO("Register service succeeded.");

  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool NormalRpcAsyncServerModule::Start() { return true; }

void NormalRpcAsyncServerModule::Shutdown() {}

}  // namespace aimrt::examples::cpp::pb_rpc::normal_rpc_async_server_module
