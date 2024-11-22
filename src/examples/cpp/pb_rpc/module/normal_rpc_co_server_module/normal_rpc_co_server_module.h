// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <memory>

#include "aimrt_module_cpp_interface/module_base.h"
#include "normal_rpc_co_server_module/service.h"

namespace aimrt::examples::cpp::pb_rpc::normal_rpc_co_server_module {

class NormalRpcCoServerModule : public aimrt::ModuleBase {
 public:
  NormalRpcCoServerModule() = default;
  ~NormalRpcCoServerModule() override = default;

  ModuleInfo Info() const override {
    return ModuleInfo{.name = "NormalRpcCoServerModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;

  bool Start() override;

  void Shutdown() override;

 private:
  aimrt::CoreRef core_;
  std::shared_ptr<ExampleServiceImpl> service_ptr_;

  std::string service_name_;
};

}  // namespace aimrt::examples::cpp::pb_rpc::normal_rpc_co_server_module
