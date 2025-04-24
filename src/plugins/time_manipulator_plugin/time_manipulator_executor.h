// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <memory>
#include <shared_mutex>
#include <thread>
#include <unordered_map>

#include "aimrt_module_cpp_interface/executor/executor.h"
#include "core/executor/executor_base.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::plugins::time_manipulator_plugin {

class TimeManipulatorExecutor : public aimrt::runtime::core::executor::ExecutorBase {
 public:
  struct Options {
    std::string bind_executor;
    std::chrono::nanoseconds dt = std::chrono::microseconds(100000);  // 100 ms
    double init_ratio = 1.0;
    std::vector<size_t> wheel_size = {100, 360};  // 1 h
    std::string thread_sched_policy;
    std::vector<uint32_t> thread_bind_cpu;
    bool use_system_clock = false;
  };

  enum class State : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

 public:
  TimeManipulatorExecutor() = default;
  ~TimeManipulatorExecutor() override = default;

  void Initialize(std::string_view name, YAML::Node options_node) override;
  void Start() override;
  void Shutdown() override;

  std::string_view Type() const noexcept override { return "time_manipulator"; }
  std::string_view Name() const noexcept override { return name_; }

  bool ThreadSafe() const noexcept override { return thread_safe_; }
  bool IsInCurrentExecutor() const noexcept override;
  bool SupportTimerSchedule() const noexcept override { return true; }

  void Execute(aimrt::executor::Task&& task) noexcept override;

  std::chrono::system_clock::time_point Now() const noexcept override;
  void ExecuteAt(std::chrono::system_clock::time_point tp, aimrt::executor::Task&& task) noexcept override;

  State GetState() const { return state_.load(); }

  void RegisterGetExecutorFunc(
      const std::function<aimrt::executor::ExecutorRef(std::string_view)>& get_executor_func);

  void SetTimeRatio(double ratio);
  double GetTimeRatio() const;

 private:
  void TimerLoop();

  struct TaskWithTimestamp {
    uint64_t tick_count;  // Time tick from start_time
    aimrt::executor::Task task;
  };

  using TaskList = std::list<TaskWithTimestamp>;

  struct TimingWheelTool {
    uint64_t current_pos;
    uint64_t scale;
    std::vector<TaskList> wheel;
    std::function<void()> borrow_func;

    TaskList Tick() {
      TaskList task_list;
      task_list.swap(wheel[current_pos]);

      ++current_pos;
      if (current_pos == wheel.size()) [[unlikely]] {
        current_pos = 0;
        borrow_func();
      }

      return task_list;
    }
  };

 private:
  std::string name_;
  Options options_;
  std::atomic<State> state_ = State::kPreInit;

  std::function<aimrt::executor::ExecutorRef(std::string_view)> get_executor_func_;
  executor::ExecutorRef bind_executor_ref_;
  bool thread_safe_ = true;

  uint64_t dt_count_;

  mutable std::shared_mutex ratio_mutex_;
  bool ratio_direction_ = true;
  uint32_t real_ratio_ = 1;

  uint64_t start_time_point_ = 0;
  std::atomic_bool start_flag_ = false;

  mutable std::shared_mutex tick_mutex_;
  uint64_t current_tick_count_ = 0;
  std::vector<TimingWheelTool> timing_wheel_vec_;
  uint64_t timing_task_map_pos_ = 0;
  std::unordered_map<uint64_t, TaskList> timing_task_map_;

  std::unique_ptr<std::thread> timer_thread_ptr_;
};

}  // namespace aimrt::plugins::time_manipulator_plugin
