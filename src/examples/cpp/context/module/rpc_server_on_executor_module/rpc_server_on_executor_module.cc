// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "rpc_server_on_executor_module/rpc_server_on_executor_module.h"
#include "aimrt_module_cpp_interface/co/aimrt_context.h"
#include "aimrt_module_cpp_interface/co/async_scope.h"
#include "aimrt_module_cpp_interface/co/inline_scheduler.h"
#include "aimrt_module_cpp_interface/co/on.h"
#include "aimrt_module_cpp_interface/co/schedule.h"
#include "aimrt_module_cpp_interface/co/sync_wait.h"
#include "rpc.aimrt_rpc.pb.h"
#include "rpc.pb.h"
#include "rpc/rpc_context.h"
#include "yaml-cpp/yaml.h"

namespace aimrt::examples::cpp::context::RpcServerOnExecutorModule {

bool RpcServerOnExecutorModule::Initialize(aimrt::CoreRef core) {
  ctx_ptr_ = std::make_shared<aimrt::context::Context>(core);
  ctx_ptr_->LetMe();
  try {
    auto cfg_path = ctx_ptr_->GetConfigFilePath();
    if (!cfg_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(cfg_path));
    }

    work_executor_ = ctx_ptr_->CreateExecutor("work_thread_pool");

    auto tx = ctx_ptr_->CreateServer<aimrt::protocols::example::ExampleServiceCoServer>();

    tx->GetBarData.ServeOn(work_executor_,[this](aimrt::rpc::ContextRef ctx, const aimrt::protocols::example::GetBarDataReq& req, aimrt::protocols::example::GetBarDataRsp& rsp) {
      AIMRT_INFO("RpcServerOnExecutorModule handle GetBarData server inline rpc call. context: {}, req: {}, return rsp: {}",
                 ctx.ToString(), aimrt::Pb2CompactJson(req), aimrt::Pb2CompactJson(rsp));
      rsp.set_msg("echo " + req.msg());
      return aimrt::rpc::Status();
    });

    tx->GetFooData.ServeOn(work_executor_,[this](aimrt::rpc::ContextRef ctx, const aimrt::protocols::example::GetFooDataReq& req, aimrt::protocols::example::GetFooDataRsp& rsp) {
      AIMRT_INFO("RpcServerOnExecutorModule handle GetFooData server inline rpc call. context: {}, req: {}, return rsp: {}",
                 ctx.ToString(), aimrt::Pb2CompactJson(req), aimrt::Pb2CompactJson(rsp));
      rsp.set_msg("echo " + req.msg());
      return aimrt::rpc::Status();
    });

  } catch (const std::exception& e) {
    AIMRT_ERROR("RpcServerOnExecutorModule init failed: {}", e.what());
    return false;
  }

  return true;
}

bool RpcServerOnExecutorModule::Start() {
  ctx_ptr_->LetMe();
  return true;
}

void RpcServerOnExecutorModule::Shutdown() {
  try {
    run_flag_ = false;
    ctx_ptr_->StopRunning();

    // 等待协程完成
    aimrt::co::SyncWait(scope_.complete());

    AIMRT_INFO("RpcServerOnExecutorModule shutdown succeeded.");
  } catch (const std::exception& e) {
    AIMRT_ERROR("RpcServerOnExecutorModule shutdown failed: {}", e.what());
  }
}

}  // namespace aimrt::examples::cpp::context::RpcServerOnExecutorModule
