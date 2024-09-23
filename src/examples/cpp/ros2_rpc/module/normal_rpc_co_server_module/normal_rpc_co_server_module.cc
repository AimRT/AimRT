// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "normal_rpc_co_server_module/normal_rpc_co_server_module.h"
#include "normal_rpc_co_server_module/filter.h"
#include "normal_rpc_co_server_module/global.h"

namespace aimrt::examples::cpp::ros2_rpc::normal_rpc_co_server_module {

bool NormalRpcCoServerModule::Initialize(aimrt::CoreRef core) {
  core_ = core;

  SetLogger(core_.GetLogger());

  try {
    // Create service
    service_ptr_ = std::make_shared<RosTestRpcServiceImpl>();

    // Register filter
    service_ptr_->RegisterFilter(TimeCostLogServerFilter);

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

bool NormalRpcCoServerModule::Start() { return true; }

void NormalRpcCoServerModule::Shutdown() {}

}  // namespace aimrt::examples::cpp::ros2_rpc::normal_rpc_co_server_module
