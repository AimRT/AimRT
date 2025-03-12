// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <future>
#include <memory>

#include "aimrt_module_cpp_interface/module_base.h"

#include "RosTestRpc.aimrt_rpc.srv.h"

namespace aimrt::examples::cpp::ros2_rpc::normal_rpc_future_client_module {

class NormalRpcFutureClientModule : public aimrt::ModuleBase {
 public:
  NormalRpcFutureClientModule() = default;
  ~NormalRpcFutureClientModule() override = default;

  ModuleInfo Info() const override {
    return ModuleInfo{.name = "NormalRpcFutureClientModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;

  bool Start() override;

  void Shutdown() override;

 private:
  auto GetLogger() { return core_.GetLogger(); }

  void MainLoop();

 private:
  aimrt::CoreRef core_;
  aimrt::executor::ExecutorRef executor_;

  std::atomic_bool run_flag_ = false;
  std::promise<void> stop_sig_;

  double rpc_frq_ = 1.0;
};

}  // namespace aimrt::examples::cpp::ros2_rpc::normal_rpc_future_client_module
