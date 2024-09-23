// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "normal_rpc_async_client_module/normal_rpc_async_client_module.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::examples::cpp::protobuf_rpc::normal_rpc_async_client_module {

bool NormalRpcAsyncClientModule::Initialize(aimrt::CoreRef core) {
  core_ = core;

  try {
    // Read cfg
    std::string file_path = std::string(core_.GetConfigurator().GetConfigFilePath());
    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(file_path);
      rpc_frq_ = cfg_node["rpc_frq"].as<double>();
    }

    // Get executor handle
    executor_ = core_.GetExecutorManager().GetExecutor("work_thread_pool");
    AIMRT_CHECK_ERROR_THROW(executor_ && executor_.SupportTimerSchedule(),
                            "Get executor 'work_thread_pool' failed.");

    // Get rpc handle
    auto rpc_handle = core_.GetRpcHandle();
    AIMRT_CHECK_ERROR_THROW(rpc_handle, "Get rpc handle failed.");

    // Register rpc client
    bool ret = aimrt::protocols::example::RegisterExampleServiceClientFunc(rpc_handle);
    AIMRT_CHECK_ERROR_THROW(ret, "Register client failed.");

  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool NormalRpcAsyncClientModule::Start() {
  try {
    executor_.Execute(std::bind(&NormalRpcAsyncClientModule::MainLoopFunc, this));
  } catch (const std::exception& e) {
    AIMRT_ERROR("Start failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Start succeeded.");
  return true;
}

void NormalRpcAsyncClientModule::Shutdown() {
  run_flag_ = false;
  stop_sig_.get_future().wait();

  AIMRT_INFO("Shutdown succeeded.");
}

// Main loop
void NormalRpcAsyncClientModule::MainLoopFunc() {
  // Check
  if (!run_flag_) {
    stop_sig_.set_value();
    AIMRT_INFO("Exit MainLoop.");
    return;
  }

  count_++;
  AIMRT_INFO("Loop count : {} -------------------------", count_);

  // Create proxy
  aimrt::protocols::example::ExampleServiceAsyncProxy proxy(core_.GetRpcHandle());

  // Create req and rsp
  auto req_ptr = std::make_shared<aimrt::protocols::example::GetFooDataReq>();
  auto rsp_ptr = std::make_shared<aimrt::protocols::example::GetFooDataRsp>();
  req_ptr->set_msg("hello world foo, count " + std::to_string(count_));

  // Create ctx
  auto ctx_ptr = proxy.NewContextSharedPtr();
  ctx_ptr->SetTimeout(std::chrono::seconds(3));

  AIMRT_INFO("Client start new rpc call. req: {}", aimrt::Pb2CompactJson(*req_ptr));

  // Call rpc
  proxy.GetFooData(
      ctx_ptr, *req_ptr, *rsp_ptr,
      [this, ctx_ptr, req_ptr, rsp_ptr](aimrt::rpc::Status status) {
        // Check result
        if (status.OK()) {
          AIMRT_INFO("Client get rpc ret, status: {}, rsp: {}", status.ToString(),
                     aimrt::Pb2CompactJson(*rsp_ptr));
        } else {
          AIMRT_WARN("Client get rpc error ret, status: {}", status.ToString());
        }
      });

  // Sleep
  executor_.ExecuteAfter(
      std::chrono::milliseconds(static_cast<uint32_t>(1000 / rpc_frq_)),
      std::bind(&NormalRpcAsyncClientModule::MainLoopFunc, this));
}

}  // namespace aimrt::examples::cpp::protobuf_rpc::normal_rpc_async_client_module
