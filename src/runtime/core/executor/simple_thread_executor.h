// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>

#include "core/executor/executor_base.h"
#include "util/log_util.h"

namespace aimrt::runtime::core::executor {

class SimpleThreadExecutor : public ExecutorBase {
 public:
  struct Options {
    std::string thread_sched_policy;
    std::vector<uint32_t> thread_bind_cpu;
    uint32_t queue_threshold = 10000;
  };

  enum class State : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

 public:
  SimpleThreadExecutor()
      : logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}
  ~SimpleThreadExecutor() override = default;

  void Initialize(std::string_view name, YAML::Node options_node) override;
  void Start() override;
  void Shutdown() override;

  std::string_view Type() const noexcept override { return "simple_thread"; }
  std::string_view Name() const noexcept override { return name_; }

  bool ThreadSafe() const noexcept override { return true; }
  bool IsInCurrentExecutor() const noexcept override {
    return std::this_thread::get_id() == thread_id_;
  }
  bool SupportTimerSchedule() const noexcept override { return false; }

  void Execute(aimrt::executor::Task&& task) noexcept override;

  std::chrono::system_clock::time_point Now() const noexcept override {
    return std::chrono::system_clock::now();
  }
  void ExecuteAt(std::chrono::system_clock::time_point tp, aimrt::executor::Task&& task) noexcept override;

  size_t CurrentTaskNum() noexcept override { return queue_task_num_.load(); }

  State GetState() const { return state_.load(); }

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) { logger_ptr_ = logger_ptr; }
  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

 private:
  std::string name_;
  Options options_;
  std::atomic<State> state_ = State::kPreInit;
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  uint32_t queue_threshold_;
  uint32_t queue_warn_threshold_;
  std::atomic_uint32_t queue_task_num_ = 0;

  std::thread::id thread_id_;

  std::mutex mutex_;
  std::condition_variable cond_;
  std::queue<aimrt::executor::Task> queue_;
  std::unique_ptr<std::thread> thread_ptr_;
};

}  // namespace aimrt::runtime::core::executor