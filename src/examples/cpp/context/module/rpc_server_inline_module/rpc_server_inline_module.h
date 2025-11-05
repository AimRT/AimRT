// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <memory>
#include <string>

#include "aimrt_module_cpp_interface.h"
#include "aimrt_module_cpp_interface/co/async_scope.h"
#include "aimrt_module_protobuf_interface/aimrt_module_protobuf_interface.h"

#include "rpc.aimrt_rpc.pb.h"

namespace aimrt::examples::cpp::context::RpcServerInlineModule {

class RpcServerInlineModule : public aimrt::ModuleBase {
 public:
  RpcServerInlineModule() = default;
  ~RpcServerInlineModule() override = default;

  ModuleInfo Info() const override { return ModuleInfo{.name = "RpcServerInlineModule"}; }

  bool Initialize(aimrt::CoreRef core) override;
  bool Start() override;
  void Shutdown() override;

 private:
  aimrt::logger::LoggerRef GetLogger() const { return ctx_ptr_->GetLogger(); }


 private:
  std::shared_ptr<aimrt::context::Context> ctx_ptr_;
  aimrt::protocols::example::ExampleServiceCoServer server_;
  aimrt::executor::ExecutorRef time_executor_;
  aimrt::co::AsyncScope scope_;

  std::string service_name_;
  double rpc_frq_;
  bool run_flag_ = true;
};

}  // namespace aimrt::examples::cpp::context::RpcServerInlineModule
