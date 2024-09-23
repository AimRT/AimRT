// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "normal_rpc_co_client_module/normal_rpc_co_client_module.h"
#include "aimrt_module_cpp_interface/co/aimrt_context.h"
#include "aimrt_module_cpp_interface/co/inline_scheduler.h"
#include "aimrt_module_cpp_interface/co/on.h"
#include "aimrt_module_cpp_interface/co/schedule.h"
#include "aimrt_module_cpp_interface/co/sync_wait.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::examples::cpp::ros2_rpc::normal_rpc_co_client_module {

bool NormalRpcCoClientModule::Initialize(aimrt::CoreRef core) {
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

    // Create rpc proxy
    proxy_ = std::make_shared<example_ros2::srv::RosTestRpcCoProxy>(rpc_handle);

    // Register filter
    proxy_->RegisterFilter([this](aimrt::rpc::ContextRef ctx,
                                  const void* req_ptr, void* rsp_ptr,
                                  const aimrt::rpc::CoRpcHandle& next)
                               -> co::Task<aimrt::rpc::Status> {
      // timecost count
      auto begin_time = std::chrono::steady_clock::now();
      const auto& status = co_await next(ctx, req_ptr, rsp_ptr);
      auto end_time = std::chrono::steady_clock::now();

      AIMRT_INFO("Client rpc time cost {} us",
                 std::chrono::duration_cast<std::chrono::microseconds>(end_time - begin_time).count());

      co_return status;
    });

  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool NormalRpcCoClientModule::Start() {
  try {
    scope_.spawn(co::On(co::InlineScheduler(), MainLoop()));
  } catch (const std::exception& e) {
    AIMRT_ERROR("Start failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Start succeeded.");
  return true;
}

void NormalRpcCoClientModule::Shutdown() {
  try {
    run_flag_ = false;
    co::SyncWait(scope_.complete());
  } catch (const std::exception& e) {
    AIMRT_ERROR("Shutdown failed, {}", e.what());
    return;
  }

  AIMRT_INFO("Shutdown succeeded.");
}

// Main loop
co::Task<void> NormalRpcCoClientModule::MainLoop() {
  try {
    AIMRT_INFO("Start MainLoop.");

    co::AimRTScheduler work_thread_pool_scheduler(executor_);

    co_await co::Schedule(work_thread_pool_scheduler);

    uint32_t count = 0;
    while (run_flag_) {
      co_await co::ScheduleAfter(
          work_thread_pool_scheduler,
          std::chrono::milliseconds(static_cast<uint32_t>(1000 / rpc_frq_)));
      count++;
      AIMRT_INFO("Loop count : {} -------------------------", count);

      // call rpc
      example_ros2::srv::RosTestRpc_Request req;
      example_ros2::srv::RosTestRpc_Response rsp;
      req.data = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

      AIMRT_INFO("start new rpc call. req:\n{}", example_ros2::srv::to_yaml(req));

      auto ctx_ptr = proxy_->NewContextSharedPtr();
      ctx_ptr->SetTimeout(std::chrono::seconds(3));

      auto status = co_await proxy_->RosTestRpc(ctx_ptr, req, rsp);

      AIMRT_INFO("Get rpc ret, status: {}. rsp:\n{}",
                 status.ToString(), example_ros2::srv::to_yaml(rsp));

      AIMRT_CHECK_WARN(status, "Call GetFooData failed, status: {}", status.ToString());
    }

    AIMRT_INFO("Exit MainLoop.");
  } catch (const std::exception& e) {
    AIMRT_ERROR("Exit MainLoop with exception, {}", e.what());
  }

  co_return;
}

}  // namespace aimrt::examples::cpp::ros2_rpc::normal_rpc_co_client_module
