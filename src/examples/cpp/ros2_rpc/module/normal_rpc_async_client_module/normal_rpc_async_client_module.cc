// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "normal_rpc_async_client_module/normal_rpc_async_client_module.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::examples::cpp::ros2_rpc::normal_rpc_async_client_module {

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
    bool ret = example_ros2::srv::RegisterRosTestRpcClientFunc(rpc_handle);
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
    run_flag_ = true;
    executor_.Execute(std::bind(&NormalRpcAsyncClientModule::MainLoopFunc, this));
  } catch (const std::exception& e) {
    AIMRT_ERROR("Start failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Start succeeded.");
  return true;
}

void NormalRpcAsyncClientModule::Shutdown() {
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
void NormalRpcAsyncClientModule::MainLoopFunc() {
  if (!run_flag_) {
    stop_sig_.set_value();

    AIMRT_INFO("Exit MainLoop.");

    return;
  }

  count_++;
  AIMRT_INFO("Loop count : {} -------------------------", count_);

  example_ros2::srv::RosTestRpcAsyncProxy proxy(core_.GetRpcHandle());

  // call rpc
  auto req_ptr = std::make_shared<example_ros2::srv::RosTestRpc_Request>();
  auto rsp_ptr = std::make_shared<example_ros2::srv::RosTestRpc_Response>();
  req_ptr->data = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

  auto ctx_ptr = proxy.NewContextSharedPtr();
  ctx_ptr->SetTimeout(std::chrono::seconds(3));

  AIMRT_INFO("start new rpc call. req:\n{}", example_ros2::srv::to_yaml(*req_ptr));

  proxy.RosTestRpc(
      ctx_ptr, *req_ptr, *rsp_ptr,
      [this, ctx_ptr, req_ptr, rsp_ptr](aimrt::rpc::Status status) {
        if (status.OK()) {
          AIMRT_INFO("Client get rpc ret, status: {}, rsp: {}", status.ToString(),
                     example_ros2::srv::to_yaml(*rsp_ptr));
        } else {
          AIMRT_WARN("Client get rpc error ret, status: {}", status.ToString());
        }
      });

  executor_.ExecuteAfter(
      std::chrono::milliseconds(static_cast<uint32_t>(1000 / rpc_frq_)),
      std::bind(&NormalRpcAsyncClientModule::MainLoopFunc, this));
}

}  // namespace aimrt::examples::cpp::ros2_rpc::normal_rpc_async_client_module
