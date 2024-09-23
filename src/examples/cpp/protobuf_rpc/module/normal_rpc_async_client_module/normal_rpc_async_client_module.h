// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <future>
#include <memory>

#include "aimrt_module_cpp_interface/module_base.h"

#include "rpc.aimrt_rpc.pb.h"

namespace aimrt::examples::cpp::protobuf_rpc::normal_rpc_async_client_module {

class NormalRpcAsyncClientModule : public aimrt::ModuleBase {
 public:
  NormalRpcAsyncClientModule() = default;
  ~NormalRpcAsyncClientModule() override = default;

  ModuleInfo Info() const override {
    return ModuleInfo{.name = "NormalRpcAsyncClientModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;

  bool Start() override;

  void Shutdown() override;

 private:
  auto GetLogger() { return core_.GetLogger(); }

  void MainLoopFunc();

 private:
  aimrt::CoreRef core_;
  aimrt::executor::ExecutorRef executor_;

  uint32_t count_ = 0;
  std::atomic_bool run_flag_ = true;
  std::promise<void> stop_sig_;

  double rpc_frq_ = 1.0;
};

}  // namespace aimrt::examples::cpp::protobuf_rpc::normal_rpc_async_client_module
