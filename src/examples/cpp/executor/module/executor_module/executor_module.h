// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>

#include "aimrt_module_cpp_interface/module_base.h"

namespace aimrt::examples::cpp::executor::executor_module {

class ExecutorModule : public aimrt::ModuleBase {
 public:
  ExecutorModule() = default;
  ~ExecutorModule() override = default;

  ModuleInfo Info() const override {
    return ModuleInfo{.name = "ExecutorModule"};
  }

  bool Initialize(aimrt::CoreRef aimrt_ptr) override;

  bool Start() override;

  void Shutdown() override;

 private:
  auto GetLogger() { return core_.GetLogger(); }

  void SimpleExecuteDemo();
  void ThreadSafeDemo();
  void TimeScheduleDemo();

 private:
  aimrt::CoreRef core_;

  aimrt::executor::ExecutorRef work_executor_;
  aimrt::executor::ExecutorRef thread_safe_executor_;

  std::atomic_bool run_flag_ = true;
  uint32_t loop_count_ = 0;
  aimrt::executor::ExecutorRef time_schedule_executor_;
};

}  // namespace aimrt::examples::cpp::executor::executor_module
