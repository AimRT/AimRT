// Copyright (c) 2024 The AimRT Authors.
// AimRT is licensed under Mulan PSL v2.

#pragma once

#include <chrono>
#include <functional>

#include "aimrt_module_cpp_interface/executor/executor.h"
#include "exception.h"

namespace aimrt::executor {

class Timer {
 public:
  Timer(ExecutorRef executor, std::chrono::nanoseconds period, std::function<void()>&& task, bool auto_start = true)
      : executor_(executor), period_(period), task_(std::move(task)) {
    AIMRT_ASSERT(executor_, "Executor is null.");
    AIMRT_ASSERT(executor_.SupportTimerSchedule(), "Executor does not support timer scheduling.");
    AIMRT_ASSERT(period_ >= std::chrono::nanoseconds::zero(), "Timer period must not be negative.");

    next_call_time_ = executor_.Now();
    if (auto_start) {
      Start();
    }
  }

  ~Timer() { Cancel(); }

  Timer(const Timer&) = delete;
  Timer& operator=(const Timer&) = delete;

  void Start() {
    executor_.ExecuteAt(next_call_time_, [this]() {
      if (cancelled_) {
        return;
      }

      task_();

      next_call_time_ += period_;

      // If the next call time is before the current time, we need to skip some times.
      auto now = executor_.Now();
      if (next_call_time_ < now) {
        if (period_ == std::chrono::nanoseconds::zero()) {
          next_call_time_ = now;
        } else {
          auto now_ahead = now - next_call_time_;
          auto skip_count = 1 + (now_ahead - std::chrono::nanoseconds(1)) / period_;
          next_call_time_ += skip_count * period_;
        }
      }

      Start();
    });
  }

  void Cancel() { cancelled_ = true; }

 private:
  ExecutorRef executor_;
  std::chrono::nanoseconds period_;
  std::function<void()> task_;
  std::chrono::system_clock::time_point next_call_time_;
  bool cancelled_ = false;
};
}  // namespace aimrt::executor
