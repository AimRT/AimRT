// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>

#include "aimrt_module_cpp_interface/context/context.h"
#include "aimrt_module_cpp_interface/executor/executor.h"
#include "aimrt_module_cpp_interface/logger/logger.h"
#include "aimrt_module_cpp_interface/module_base.h"

namespace aimrt::examples::cpp::context::executor_module {

class ExecutorModule : public aimrt::ModuleBase {
 public:
  ExecutorModule() = default;
  ~ExecutorModule() override = default;

  ModuleInfo Info() const override {
    return ModuleInfo{.name = "ContextExecutorModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;

  bool Start() override;

  void Shutdown() override;

 private:
  aimrt::logger::LoggerRef GetLogger() { return core_.GetLogger(); }

  void TimeScheduleDemo();

 private:
  aimrt::CoreRef core_;

  aimrt::executor::ExecutorRef work_executor_;
  aimrt::executor::ExecutorRef thread_safe_executor_;
  aimrt::executor::ExecutorRef time_schedule_executor_;

  std::atomic_bool run_flag_{true};
  std::atomic_uint32_t loop_count_{0};
};

}  // namespace aimrt::examples::cpp::context::executor_module
