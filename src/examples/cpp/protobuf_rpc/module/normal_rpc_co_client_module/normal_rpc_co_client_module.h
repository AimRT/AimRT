// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <memory>

#include "aimrt_module_cpp_interface/co/async_scope.h"
#include "aimrt_module_cpp_interface/co/task.h"
#include "aimrt_module_cpp_interface/module_base.h"
#include "aimrt_module_cpp_interface/rpc/rpc_co_filter.h"

#include "rpc.aimrt_rpc.pb.h"

namespace aimrt::examples::cpp::protobuf_rpc::normal_rpc_co_client_module {

class NormalRpcCoClientModule : public aimrt::ModuleBase {
 public:
  NormalRpcCoClientModule() = default;
  ~NormalRpcCoClientModule() override = default;

  ModuleInfo Info() const override {
    return ModuleInfo{.name = "NormalRpcCoClientModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;

  bool Start() override;

  void Shutdown() override;

 private:
  auto GetLogger() { return core_.GetLogger(); }

  co::Task<void> MainLoop();

 private:
  aimrt::CoreRef core_;
  aimrt::executor::ExecutorRef executor_;

  co::AsyncScope scope_;
  std::atomic_bool run_flag_ = true;

  double rpc_frq_ = 1.0;
  std::shared_ptr<aimrt::protocols::example::ExampleServiceCoProxy> proxy_;
};

}  // namespace aimrt::examples::cpp::protobuf_rpc::normal_rpc_co_client_module
