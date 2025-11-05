// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "rpc_server_inline_module.h"
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

namespace aimrt::examples::cpp::context::RpcServerInlineModule {

bool RpcServerInlineModule::Initialize(aimrt::CoreRef core) {
  ctx_ptr_ = std::make_shared<aimrt::context::Context>(core);
  ctx_ptr_->LetMe();
  try {
    auto cfg_path = ctx_ptr_->GetConfigFilePath();
    if (!cfg_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(cfg_path));
    }

    time_executor_ = ctx_ptr_->CreateExecutor("time_schedule_executor");

    auto tx = ctx_ptr_->CreateServer<aimrt::protocols::example::ExampleServiceCoServer>();

    tx->GetBarData.ServeInline([this](aimrt::rpc::ContextRef ctx, const aimrt::protocols::example::GetBarDataReq& req, aimrt::protocols::example::GetBarDataRsp& rsp) {
      AIMRT_INFO("RpcServerInlineModule handle GetBarData server inline rpc call. context: {}, req: {}, return rsp: {}",
                 ctx.ToString(), aimrt::Pb2CompactJson(req), aimrt::Pb2CompactJson(rsp));
      rsp.set_msg("echo " + req.msg());
      return aimrt::rpc::Status();
    });

    tx->GetFooData.ServeInline([this](aimrt::rpc::ContextRef ctx, const aimrt::protocols::example::GetFooDataReq& req, aimrt::protocols::example::GetFooDataRsp& rsp) {
      AIMRT_INFO("RpcServerInlineModule handle GetFooData server inline rpc call. context: {}, req: {}, return rsp: {}",
                 ctx.ToString(), aimrt::Pb2CompactJson(req), aimrt::Pb2CompactJson(rsp));
      rsp.set_msg("echo " + req.msg());
      return aimrt::rpc::Status();
    });

  } catch (const std::exception& e) {
    AIMRT_ERROR("RpcServerInlineModule init failed: {}", e.what());
    return false;
  }

  return true;
}

bool RpcServerInlineModule::Start() {
  ctx_ptr_->LetMe();
  return true;
}

void RpcServerInlineModule::Shutdown() {
  try {
    run_flag_ = false;
    ctx_ptr_->StopRunning();

    // 等待协程完成
    aimrt::co::SyncWait(scope_.complete());

    AIMRT_INFO("RpcServerInlineModule shutdown succeeded.");
  } catch (const std::exception& e) {
    AIMRT_ERROR("RpcServerInlineModule shutdown failed: {}", e.what());
  }
}

}  // namespace aimrt::examples::cpp::context::RpcServerInlineModule
