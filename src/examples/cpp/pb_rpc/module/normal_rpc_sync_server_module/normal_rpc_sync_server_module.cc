// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "normal_rpc_sync_server_module/normal_rpc_sync_server_module.h"
#include "normal_rpc_sync_server_module/global.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::examples::cpp::pb_rpc::normal_rpc_sync_server_module {

bool NormalRpcSyncServerModule::Initialize(aimrt::CoreRef core) {
  core_ = core;

  SetLogger(core_.GetLogger());

  try {
    // Read cfg
    std::string file_path = std::string(core_.GetConfigurator().GetConfigFilePath());
    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(file_path);
      if (cfg_node["service_name"]) {
        service_name_ = cfg_node["service_name"].as<std::string>();
      }
    }

    // Create service
    service_ptr_ = std::make_shared<ExampleServiceSyncServiceImpl>();

    // Register service
    bool ret = false;
    if (service_name_.empty()) {
      ret = core_.GetRpcHandle().RegisterService(service_ptr_.get());
    } else {
      ret = core_.GetRpcHandle().RegisterService(service_name_, service_ptr_.get());
    }

    AIMRT_CHECK_ERROR_THROW(ret, "Register service failed.");

    AIMRT_INFO("Register service succeeded.");

  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool NormalRpcSyncServerModule::Start() { return true; }

void NormalRpcSyncServerModule::Shutdown() {}

}  // namespace aimrt::examples::cpp::pb_rpc::normal_rpc_sync_server_module
