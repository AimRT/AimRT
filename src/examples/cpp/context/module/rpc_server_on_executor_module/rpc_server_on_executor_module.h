// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <memory>
#include <string>

#include "aimrt_module_cpp_interface.h"
#include "aimrt_module_cpp_interface/co/async_scope.h"
#include "aimrt_module_protobuf_interface/aimrt_module_protobuf_interface.h"

#include "rpc.aimrt_rpc.pb.h"

namespace aimrt::examples::cpp::context::RpcServerOnExecutorModule {

class RpcServerOnExecutorModule : public aimrt::ModuleBase {
 public:
  RpcServerOnExecutorModule() = default;
  ~RpcServerOnExecutorModule() override = default;

  ModuleInfo Info() const override { return ModuleInfo{.name = "RpcServerOnExecutorModule"}; }

  bool Initialize(aimrt::CoreRef core) override;
  bool Start() override;
  void Shutdown() override;

 private:
  aimrt::logger::LoggerRef GetLogger() const { return ctx_ptr_->GetLogger(); }

 private:
  std::shared_ptr<aimrt::context::Context> ctx_ptr_;
  aimrt::executor::ExecutorRef work_executor_;

  std::string service_name_;
  double rpc_frq_;
};

}  // namespace aimrt::examples::cpp::context::RpcServerOnExecutorModule
