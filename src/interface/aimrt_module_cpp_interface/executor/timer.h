// Copyright (c) 2024 The AimRT Authors.
// AimRT is licensed under Mulan PSL v2.

#pragma once

#include <chrono>
#include <memory>

#include "aimrt_module_cpp_interface/executor/executor.h"
#include "util/same_arg_trait.h"

namespace aimrt::executor {

class TimerBase {
 public:
  TimerBase(ExecutorRef executor, std::chrono::nanoseconds period)
      : executor_(executor), period_(period) {
    AIMRT_ASSERT(executor_, "Executor is null.");
    AIMRT_ASSERT(executor_.SupportTimerSchedule(), "Executor does not support timer scheduling.");
    AIMRT_ASSERT(period_ >= std::chrono::nanoseconds::zero(), "Timer period must not be negative.");
  }

  virtual ~TimerBase() = default;

  TimerBase(const TimerBase&) = delete;
  TimerBase& operator=(const TimerBase&) = delete;

  virtual void Start() = 0;

  virtual void Reset() = 0;

  virtual void ExecuteTask() = 0;

  void Cancel() { cancelled_ = true; }

  [[nodiscard]] bool IsCancelled() const { return cancelled_; }

  [[nodiscard]] std::chrono::nanoseconds Period() const { return period_; }

  [[nodiscard]] std::chrono::system_clock::time_point NextCallTime() const {
    return next_call_time_;
  }

  [[nodiscard]] std::chrono::nanoseconds TimeUntilNextCall() const {
    return next_call_time_ - executor_.Now();
  }

  [[nodiscard]] ExecutorRef Executor() const { return executor_; }

 protected:
  ExecutorRef executor_;
  std::chrono::nanoseconds period_;
  std::chrono::system_clock::time_point next_call_time_;
  bool cancelled_ = false;
};

template <typename TaskType>
  requires(common::util::SameArguments<TaskType, std::function<void()>> ||
           common::util::SameArguments<TaskType, std::function<void(TimerBase&)>> ||
           common::util::SameArguments<TaskType, std::function<void(const TimerBase&)>>)
class Timer : public TimerBase {
 public:
  Timer(ExecutorRef executor, std::chrono::nanoseconds period, TaskType&& task, bool auto_start = true)
      : TimerBase(executor, period), task_(std::move(task)) {
    if (auto_start) {
      Start();
    } else {
      Cancel();
    }
  }

  ~Timer() { Cancel(); }

  Timer(const Timer&) = delete;
  Timer& operator=(const Timer&) = delete;

  void Start() override {
    cancelled_ = false;
    next_call_time_ = executor_.Now();
    ExecuteLoop();
  }

  void Reset() override {
    cancelled_ = false;
    next_call_time_ = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
        executor_.Now() + period_);
    ExecuteLoop();
  }

  void ExecuteTask() override {
    if constexpr (std::is_invocable_v<TaskType>) {
      task_();
    } else {
      task_(*this);
    }
  }

  [[nodiscard]] const TaskType& Task() const { return task_; }

 private:
  void ExecuteLoop() {
    if (IsCancelled()) {
      return;
    }

    executor_.ExecuteAt(next_call_time_, [this, planned_time = next_call_time_]() {
      if (IsCancelled()) {
        return;
      }

      // Skip current execution if timer was reset or restarted
      if (planned_time != next_call_time_) {
        return;
      }

      ExecuteTask();

      auto now = executor_.Now();
      next_call_time_ = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
          next_call_time_ + period_);

      // If now is ahead of the next call time, skip some times.
      if (next_call_time_ < now) {
        if (period_ == std::chrono::nanoseconds::zero()) {
          next_call_time_ = now;
        } else {
          auto now_ahead = now - next_call_time_;
          auto skip_count = 1 + ((now_ahead - std::chrono::nanoseconds(1)) / period_);
          next_call_time_ = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
              next_call_time_ + (skip_count * period_));
        }
      }

      ExecuteLoop();
    });
  }

 private:
  TaskType task_;
};

template <typename TaskType>
std::shared_ptr<TimerBase> CreateTimer(ExecutorRef executor, std::chrono::nanoseconds period,
                                       TaskType&& task, bool auto_start = true) {
  return std::make_shared<Timer<TaskType>>(executor, period, std::forward<TaskType>(task), auto_start);
}

}  // namespace aimrt::executor
