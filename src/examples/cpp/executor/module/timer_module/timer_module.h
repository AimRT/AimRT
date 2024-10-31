// Copyright (c) 2024 The AimRT Authors.
// AimRT is licensed under Mulan PSL v2.

#pragma once

#include "aimrt_module_cpp_interface/executor/executor.h"
#include "aimrt_module_cpp_interface/executor/timer.h"
#include "aimrt_module_cpp_interface/module_base.h"

namespace aimrt::examples::cpp::executor::timer_module {

class TimerModule : public aimrt::ModuleBase {
 public:
  TimerModule() = default;
  ~TimerModule() override = default;

  [[nodiscard]] ModuleInfo Info() const override {
    return ModuleInfo{.name = "TimerModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;

  bool Start() override;

  void Shutdown() override;

 private:
  auto GetLogger() { return core_.GetLogger(); }

 private:
  aimrt::CoreRef core_;

  aimrt::executor::ExecutorRef timer_executor_;
  std::shared_ptr<aimrt::executor::Timer> timer_;
};
}  // namespace aimrt::examples::cpp::executor::timer_module
