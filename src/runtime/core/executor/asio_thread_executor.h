// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <list>
#include <memory>
#include <set>
#include <string>
#include <thread>

#include "core/executor/executor_base.h"
#include "util/log_util.h"

#include "yaml-cpp/yaml.h"

#include "asio.hpp"

namespace aimrt::runtime::core::executor {

class AsioThreadExecutor : public ExecutorBase {
 public:
  struct Options {
    uint32_t thread_num = 1;
    std::string thread_sched_policy;
    std::vector<uint32_t> thread_bind_cpu;
    std::chrono::nanoseconds timeout_alarm_threshold_us = std::chrono::seconds(1);
    uint32_t queue_threshold = 10000;
  };

  enum class State : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

 public:
  AsioThreadExecutor()
      : logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}
  ~AsioThreadExecutor() override = default;

  void Initialize(std::string_view name, YAML::Node options_node) override;
  void Start() override;
  void Shutdown() override;

  std::string_view Type() const noexcept override { return "asio_thread"; }
  std::string_view Name() const noexcept override { return name_; }

  bool ThreadSafe() const noexcept override { return (options_.thread_num == 1); }
  bool IsInCurrentExecutor() const noexcept override;
  bool SupportTimerSchedule() const noexcept override { return true; }

  void Execute(aimrt::executor::Task&& task) noexcept override;

  std::chrono::system_clock::time_point Now() const noexcept override {
    return std::chrono::system_clock::now();
  }
  void ExecuteAt(std::chrono::system_clock::time_point tp, aimrt::executor::Task&& task) noexcept override;

  size_t CurrentTaskNum() noexcept override { return queue_task_num_.load(); }

  State GetState() const { return state_.load(); }

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) { logger_ptr_ = logger_ptr; }
  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

  asio::io_context* IOCTX() { return io_ptr_.get(); }

 private:
  std::string name_;
  Options options_;
  std::atomic<State> state_ = State::kPreInit;
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  uint32_t queue_threshold_;
  uint32_t queue_warn_threshold_;
  std::atomic_uint32_t queue_task_num_ = 0;

  std::unique_ptr<asio::io_context> io_ptr_;
  std::unique_ptr<
      asio::executor_work_guard<asio::io_context::executor_type>>
      work_guard_ptr_;

  std::vector<std::thread::id> thread_id_vec_;
  std::list<std::thread> threads_;
};

}  // namespace aimrt::runtime::core::executor
