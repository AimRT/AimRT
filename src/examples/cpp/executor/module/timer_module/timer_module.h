// Copyright (c) 2024 The AimRT Authors.
// AimRT is licensed under Mulan PSL v2.

#pragma once

#include "aimrt_module_cpp_interface/executor/executor.h"
#include "aimrt_module_cpp_interface/module_base.h"

namespace aimrt::examples::cpp::executor::timer_module {

class WallTimer {
 public:
  WallTimer(aimrt::executor::ExecutorRef executor, std::chrono::nanoseconds interval, std::function<void()>&& task)
      : executor_(executor), interval_(interval), task_(std::move(task)) {
    next_time_ = std::chrono::system_clock::now();
    Start();
  }

 private:
  void Start() {
    executor_.ExecuteAt(next_time_, [this]() {
      task_();

      next_time_ += interval_;
      Start();
    });
  }

 private:
  aimrt::executor::ExecutorRef executor_;
  std::chrono::nanoseconds interval_;
  std::function<void()> task_;
  std::chrono::system_clock::time_point next_time_;
};

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
  std::shared_ptr<WallTimer> timer_;
};
}  // namespace aimrt::examples::cpp::executor::timer_module
