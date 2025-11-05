// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <memory>
#include <string>

#include "aimrt_module_cpp_interface.h"
#include "aimrt_module_cpp_interface/co/async_scope.h"
#include "aimrt_module_protobuf_interface/aimrt_module_protobuf_interface.h"

#include "rpc.aimrt_rpc.pb.h"

namespace aimrt::examples::cpp::context::RpcClientModule {

class RpcClientModule : public aimrt::ModuleBase {
 public:
  RpcClientModule() = default;
  ~RpcClientModule() override = default;

  ModuleInfo Info() const override { return ModuleInfo{.name = "RpcClientModule"}; }

  bool Initialize(aimrt::CoreRef core) override;
  bool Start() override;
  void Shutdown() override;

 private:
  aimrt::logger::LoggerRef GetLogger() const { return ctx_ptr_->GetLogger(); }

  aimrt::co::Task<void> RpcTask();

 private:
  std::shared_ptr<aimrt::context::Context> ctx_ptr_;
  aimrt::protocols::example::ExampleServiceCoClient client_;
  aimrt::executor::ExecutorRef time_executor_;
  aimrt::co::AsyncScope scope_;

  std::string service_name_;
  double rpc_frq_;
};

}  // namespace aimrt::examples::cpp::context::RpcClientModule
