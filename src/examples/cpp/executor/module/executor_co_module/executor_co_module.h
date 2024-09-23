// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>

#include "aimrt_module_cpp_interface/co/aimrt_context.h"
#include "aimrt_module_cpp_interface/co/async_scope.h"
#include "aimrt_module_cpp_interface/co/task.h"
#include "aimrt_module_cpp_interface/module_base.h"

namespace aimrt::examples::cpp::executor::executor_co_module {

class ExecutorCoModule : public aimrt::ModuleBase {
 public:
  ExecutorCoModule() = default;
  ~ExecutorCoModule() override = default;

  ModuleInfo Info() const override {
    return ModuleInfo{.name = "ExecutorCoModule"};
  }

  bool Initialize(aimrt::CoreRef aimrt_ptr) override;

  bool Start() override;

  void Shutdown() override;

 private:
  auto GetLogger() { return core_.GetLogger(); }

  co::Task<void> SimpleExecuteDemo();
  co::Task<void> ThreadSafeDemo();
  co::Task<void> TimeScheduleDemo();

 private:
  aimrt::CoreRef core_;

  co::AimRTContext ctx_;

  co::AsyncScope scope_;
  std::atomic_bool run_flag_ = true;
};

}  // namespace aimrt::examples::cpp::executor::executor_co_module
