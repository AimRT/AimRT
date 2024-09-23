// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <memory>

#include "aimrt_module_cpp_interface/module_base.h"
#include "normal_rpc_async_server_module/service.h"

namespace aimrt::examples::cpp::ros2_rpc::normal_rpc_async_server_module {

class NormalRpcAsyncServerModule : public aimrt::ModuleBase {
 public:
  NormalRpcAsyncServerModule() = default;
  ~NormalRpcAsyncServerModule() override = default;

  ModuleInfo Info() const override {
    return ModuleInfo{.name = "NormalRpcAsyncServerModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;

  bool Start() override;

  void Shutdown() override;

 private:
  aimrt::CoreRef core_;
  std::shared_ptr<RosTestRpcAsyncServiceImpl> service_ptr_;
};

}  // namespace aimrt::examples::cpp::ros2_rpc::normal_rpc_async_server_module
