// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "rpc_client_module/rpc_client_module.h"
#include "aimrt_module_cpp_interface/co/aimrt_context.h"
#include "aimrt_module_cpp_interface/co/async_scope.h"
#include "aimrt_module_cpp_interface/co/inline_scheduler.h"
#include "aimrt_module_cpp_interface/co/on.h"
#include "aimrt_module_cpp_interface/co/schedule.h"
#include "aimrt_module_cpp_interface/co/sync_wait.h"
#include "rpc/rpc_context.h"
#include "yaml-cpp/yaml.h"

namespace aimrt::examples::cpp::context::RpcClientModule {

bool RpcClientModule::Initialize(aimrt::CoreRef core) {
  ctx_ptr_ = std::make_shared<aimrt::context::Context>(core);
  ctx_ptr_->LetMe();
  try {
    auto cfg_path = ctx_ptr_->GetConfigFilePath();
    if (!cfg_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(cfg_path));
      rpc_frq_ = cfg_node["rpc_frq"].as<double>();
    }

    time_executor_ = ctx_ptr_->CreateExecutor("time_schedule_executor");

    client_ = ctx_ptr_->CreateClient<aimrt::protocols::example::ExampleServiceCoClient>();

  } catch (const std::exception& e) {
    AIMRT_ERROR("RpcClientModule init failed: {}", e.what());
    return false;
  }

  return true;
}

bool RpcClientModule::Start() {
  try {
    // 在后台启动 RPC 任务，绑定到 time_executor 避免线程切换
    auto scheduler = aimrt::co::AimRTScheduler(time_executor_);
    scope_.spawn(aimrt::co::On(scheduler, RpcTask()));

    AIMRT_INFO("RpcClientModule start succeeded.");
  } catch (const std::exception& e) {
    AIMRT_ERROR("RpcClientModule start failed: {}", e.what());
    return false;
  }
  return true;
}

aimrt::co::Task<void> RpcClientModule::RpcTask() {
  ctx_ptr_->LetMe();

  try {
    AIMRT_INFO("Start RPC loop.");

    while (ctx_ptr_->Running()) {
      aimrt::protocols::example::GetFooDataReq req;
      aimrt::protocols::example::GetFooDataRsp rsp;
      req.set_msg("hello world");
      auto status = co_await client_.GetFooData(req, rsp);
      if (status.OK()) {
        AIMRT_INFO("RPC call succeeded, response: {}", rsp.msg());
      } else {
        AIMRT_WARN("RPC call failed, status: {}", status.ToString());
      }

      co_await aimrt::co::ScheduleAfter(aimrt::co::AimRTScheduler(time_executor_), std::chrono::milliseconds(static_cast<uint32_t>(1000 / rpc_frq_)));
    }

    AIMRT_INFO("Exit RPC loop.");
  } catch (const std::exception& e) {
    AIMRT_ERROR("RpcTask failed: {}", e.what());
  }

  co_return;
}

void RpcClientModule::Shutdown() {
  try {
    run_flag_ = false;
    ctx_ptr_->StopRunning();

    // 等待协程完成
    aimrt::co::SyncWait(scope_.complete());

    AIMRT_INFO("RpcClientModule shutdown succeeded.");
  } catch (const std::exception& e) {
    AIMRT_ERROR("RpcClientModule shutdown failed: {}", e.what());
  }
}

}  // namespace aimrt::examples::cpp::context::RpcClientModule
