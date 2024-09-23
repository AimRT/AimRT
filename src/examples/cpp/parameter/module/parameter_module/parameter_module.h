// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <future>

#include "aimrt_module_cpp_interface/module_base.h"

namespace aimrt::examples::cpp::parameter::parameter_module {

class ParameterModule : public aimrt::ModuleBase {
 public:
  ParameterModule() = default;
  ~ParameterModule() override = default;

  ModuleInfo Info() const override {
    return ModuleInfo{.name = "ParameterModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;

  bool Start() override;

  void Shutdown() override;

 private:
  auto GetLogger() { return core_.GetLogger(); }

  void SetParameterLoop();
  void GetParameterLoop();

 private:
  aimrt::CoreRef core_;
  aimrt::executor::ExecutorRef work_executor_;
  aimrt::parameter::ParameterHandleRef parameter_handle_;

  std::atomic_bool run_flag_ = true;
  std::promise<void> set_loop_stop_sig_;
  std::promise<void> get_loop_stop_sig_;
};

}  // namespace aimrt::examples::cpp::parameter::parameter_module
