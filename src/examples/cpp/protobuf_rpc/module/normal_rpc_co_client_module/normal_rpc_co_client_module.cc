// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "normal_rpc_co_client_module/normal_rpc_co_client_module.h"
#include "aimrt_module_cpp_interface/co/aimrt_context.h"
#include "aimrt_module_cpp_interface/co/inline_scheduler.h"
#include "aimrt_module_cpp_interface/co/on.h"
#include "aimrt_module_cpp_interface/co/schedule.h"
#include "aimrt_module_cpp_interface/co/sync_wait.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::examples::cpp::protobuf_rpc::normal_rpc_co_client_module {

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
    bool ret = aimrt::protocols::example::RegisterExampleServiceClientFunc(rpc_handle);
    AIMRT_CHECK_ERROR_THROW(ret, "Register client failed.");

    // Create rpc proxy
    proxy_ = std::make_shared<aimrt::protocols::example::ExampleServiceCoProxy>(rpc_handle);

    // Register filter
    proxy_->RegisterFilter([this](aimrt::rpc::ContextRef ctx,
                                  const void* req_ptr, void* rsp_ptr,
                                  const aimrt::rpc::CoRpcHandle& next)
                               -> co::Task<aimrt::rpc::Status> {
      // debuglog
      AIMRT_INFO("Client start new rpc call. context: {}, req: {}",
                 ctx.ToString(), aimrt::Pb2CompactJson(*static_cast<const google::protobuf::Message*>(req_ptr)));
      const auto& status = co_await next(ctx, req_ptr, rsp_ptr);
      if (status.OK()) {
        AIMRT_INFO("Client get rpc ret, status: {}, rsp: {}", status.ToString(),
                   aimrt::Pb2CompactJson(*static_cast<const google::protobuf::Message*>(rsp_ptr)));
      } else {
        AIMRT_WARN("Client get rpc error ret, status: {}", status.ToString());
      }

      co_return status;
    });

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

    uint32_t count = 0;
    while (run_flag_) {
      // Sleep
      co_await co::ScheduleAfter(
          work_thread_pool_scheduler,
          std::chrono::milliseconds(static_cast<uint32_t>(1000 / rpc_frq_)));
      count++;
      AIMRT_INFO("Loop count : {} -------------------------", count);

      // Create req and rsp
      aimrt::protocols::example::GetFooDataReq req;
      aimrt::protocols::example::GetFooDataRsp rsp;
      req.set_msg("hello world foo, count " + std::to_string(count));

      // Create ctx
      auto ctx_ptr = proxy_->NewContextSharedPtr();
      ctx_ptr->SetTimeout(std::chrono::seconds(3));

      // Call rpc
      auto status = co_await proxy_->GetFooData(ctx_ptr, req, rsp);

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

  co_return;
}

}  // namespace aimrt::examples::cpp::protobuf_rpc::normal_rpc_co_client_module
