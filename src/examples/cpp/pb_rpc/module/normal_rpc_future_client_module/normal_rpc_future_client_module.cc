// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "normal_rpc_future_client_module/normal_rpc_future_client_module.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::examples::cpp::pb_rpc::normal_rpc_future_client_module {

bool NormalRpcFutureClientModule::Initialize(aimrt::CoreRef core) {
  core_ = core;

  try {
    // Read cfg
    std::string file_path = std::string(core_.GetConfigurator().GetConfigFilePath());
    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(file_path);
      rpc_frq_ = cfg_node["rpc_frq"].as<double>();

      if (cfg_node["service_name"]) {
        service_name_ = cfg_node["service_name"].as<std::string>();
      }
    }

    // Get executor handle
    executor_ = core_.GetExecutorManager().GetExecutor("work_thread_pool");
    AIMRT_CHECK_ERROR_THROW(executor_, "Get executor 'work_thread_pool' failed.");

    // Get rpc handle
    auto rpc_handle = core_.GetRpcHandle();
    AIMRT_CHECK_ERROR_THROW(rpc_handle, "Get rpc handle failed.");

    // Register rpc client
    bool ret = false;
    if (service_name_.empty()) {
      ret = aimrt::protocols::example::RegisterExampleServiceClientFunc(rpc_handle);
    } else {
      ret = aimrt::protocols::example::RegisterExampleServiceClientFunc(rpc_handle, service_name_);
    }

    AIMRT_CHECK_ERROR_THROW(ret, "Register client failed.");

    // Create rpc proxy
    proxy_ = std::make_shared<aimrt::protocols::example::ExampleServiceFutureProxy>(rpc_handle);

    if (!service_name_.empty()) {
      proxy_->SetServiceName(service_name_);
    }

  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool NormalRpcFutureClientModule::Start() {
  run_flag_ = true;
  try {
    executor_.Execute(std::bind(&NormalRpcFutureClientModule::MainLoop, this));
  } catch (const std::exception& e) {
    AIMRT_ERROR("Start failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Start succeeded.");
  return true;
}

void NormalRpcFutureClientModule::Shutdown() {
  try {
    if (run_flag_) {
      run_flag_ = false;
      stop_sig_.get_future().wait();
    }
  } catch (const std::exception& e) {
    AIMRT_ERROR("Shutdown failed, {}", e.what());
    return;
  }

  AIMRT_INFO("Shutdown succeeded.");
}

// Main loop
void NormalRpcFutureClientModule::MainLoop() {
  try {
    AIMRT_INFO("Start MainLoop.");

    uint32_t count = 0;
    while (run_flag_) {
      // Sleep
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<uint32_t>(1000 / rpc_frq_)));

      count++;
      AIMRT_INFO("Loop count : {} -------------------------", count);

      // Create req and rsp
      aimrt::protocols::example::GetFooDataReq req;
      aimrt::protocols::example::GetFooDataRsp rsp;
      req.set_msg("hello world foo, count " + std::to_string(count));

      // Create ctx
      auto ctx_ptr = proxy_->NewContextSharedPtr();
      ctx_ptr->SetTimeout(std::chrono::seconds(3));

      AIMRT_INFO("Client start new rpc call. req: {}", aimrt::Pb2CompactJson(req));

      // Call rpc
      auto status_future = proxy_->GetFooData(ctx_ptr, req, rsp);
      auto status = status_future.get();

      // Check result
      if (status.OK()) {
        AIMRT_INFO("Client get rpc ret, status: {}, rsp: {}", status.ToString(),
                   aimrt::Pb2CompactJson(rsp));
      } else {
        AIMRT_WARN("Client get rpc error ret, status: {}", status.ToString());
      }
    }

    AIMRT_INFO("Exit MainLoop.");
  } catch (const std::exception& e) {
    AIMRT_ERROR("Exit MainLoop with exception, {}", e.what());
  }

  stop_sig_.set_value();
}

}  // namespace aimrt::examples::cpp::pb_rpc::normal_rpc_future_client_module
