// Copyright (c) 2024 The AimRT Authors.
// AimRT is licensed under Mulan PSL v2.

#pragma once

#include <chrono>
#include <type_traits>

#include "aimrt_module_cpp_interface/executor/executor.h"
#include "exception.h"

namespace aimrt::executor {

template <typename T>
struct function_args;

template <typename R, typename... Args>
struct function_args<R(Args...)> {
  using type = std::tuple<Args...>;
};

template <typename R, typename C, typename... Args>
struct function_args<R (C::*)(Args...)> {
  using type = std::tuple<Args...>;
};

template <typename R, typename C, typename... Args>
struct function_args<R (C::*)(Args...) const> {
  using type = std::tuple<Args...>;
};

template <typename F>
struct function_args {
  using type = typename function_args<decltype(&F::operator())>::type;
};

template <typename F>
using function_args_t = typename function_args<F>::type;

template <typename F1, typename F2>
concept SameArguments = std::same_as<function_args_t<F1>, function_args_t<F2>>;

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

  void Cancel() { cancelled_ = true; }

  void Reset(std::chrono::nanoseconds period) {
    period_ = period;
    cancelled_ = false;
    next_call_time_ = executor_.Now() + period_;
  }

  void Reset() { Reset(period_); }

  virtual void ExecuteCallback() = 0;

  [[nodiscard]] bool IsCancelled() const { return cancelled_; }

  [[nodiscard]] std::chrono::nanoseconds Period() const { return period_; }

  [[nodiscard]] std::chrono::system_clock::time_point NextCallTime() const { return next_call_time_; }

  [[nodiscard]] std::chrono::nanoseconds TimeUntilNextCall() const { return next_call_time_ - executor_.Now(); }

  [[nodiscard]] ExecutorRef Executor() const { return executor_; }

 protected:
  ExecutorRef executor_;
  std::chrono::nanoseconds period_;
  std::chrono::system_clock::time_point next_call_time_;
  bool cancelled_ = false;
};

template <typename TaskType>
  requires(SameArguments<TaskType, std::function<void()>> ||
           SameArguments<TaskType, std::function<void(TimerBase&)>>)
class Timer : public TimerBase {
 public:
  Timer(ExecutorRef executor, std::chrono::nanoseconds period, TaskType&& task, bool auto_start = true)
      : TimerBase(executor, period), task_(std::move(task)) {
    if (auto_start) {
      Start();
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

  void ExecuteCallback() override {
    if constexpr (std::is_invocable_v<TaskType>) {
      task_();
    } else {
      task_(*this);
    }
  }

  [[nodiscard]] const TaskType& Task() const { return task_; }

 private:
  void ExecuteLoop() {
    executor_.ExecuteAt(next_call_time_, [this]() {
      if (cancelled_) {
        return;
      }

      auto now = executor_.Now();

      // The executor need to ensure all tasks are executed later than the current time.
      // Because we need this to handle the case when the timer is reset.
      if (next_call_time_ <= now) {
        ExecuteCallback();
        next_call_time_ += period_;
      }

      // If now is ahead of the next call time, skip some times.
      if (next_call_time_ < now) {
        if (period_ == std::chrono::nanoseconds::zero()) {
          next_call_time_ = now;
        } else {
          auto now_ahead = now - next_call_time_;
          auto skip_count = 1 + ((now_ahead - std::chrono::nanoseconds(1)) / period_);
          next_call_time_ += skip_count * period_;
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
