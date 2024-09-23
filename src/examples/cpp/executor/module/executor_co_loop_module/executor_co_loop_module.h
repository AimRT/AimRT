// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>

#include "aimrt_module_cpp_interface/co/aimrt_context.h"
#include "aimrt_module_cpp_interface/co/async_scope.h"
#include "aimrt_module_cpp_interface/co/task.h"
#include "aimrt_module_cpp_interface/module_base.h"

namespace aimrt::examples::cpp::executor::executor_co_loop_module {

class ExecutorCoLoopModule : public aimrt::ModuleBase {
 public:
  ExecutorCoLoopModule() = default;
  ~ExecutorCoLoopModule() override = default;

  ModuleInfo Info() const override {
    return ModuleInfo{.name = "ExecutorCoLoopModule"};
  }

  bool Initialize(aimrt::CoreRef aimrt_ptr) override;

  bool Start() override;

  void Shutdown() override;

 private:
  auto GetLogger() { return core_.GetLogger(); }

  co::Task<void> MainLoop();

 private:
  aimrt::CoreRef core_;
  aimrt::executor::ExecutorRef time_schedule_executor_;

  co::AsyncScope scope_;
  std::atomic_bool run_flag_ = true;
};

}  // namespace aimrt::examples::cpp::executor::executor_co_loop_module
