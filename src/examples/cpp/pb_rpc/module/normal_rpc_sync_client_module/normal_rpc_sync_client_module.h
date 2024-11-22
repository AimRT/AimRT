// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <future>
#include <memory>

#include "aimrt_module_cpp_interface/module_base.h"

#include "rpc.aimrt_rpc.pb.h"

namespace aimrt::examples::cpp::pb_rpc::normal_rpc_sync_client_module {

class NormalRpcSyncClientModule : public aimrt::ModuleBase {
 public:
  NormalRpcSyncClientModule() = default;
  ~NormalRpcSyncClientModule() override = default;

  ModuleInfo Info() const override {
    return ModuleInfo{.name = "NormalRpcSyncClientModule"};
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

  std::atomic_bool run_flag_ = true;
  std::promise<void> stop_sig_;

  double rpc_frq_ = 1.0;
  std::string service_name_;

  std::shared_ptr<aimrt::protocols::example::ExampleServiceSyncProxy> proxy_;
};

}  // namespace aimrt::examples::cpp::pb_rpc::normal_rpc_sync_client_module
