// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <chrono>
#include <string>

#include "aimrt_module_cpp_interface/executor/executor.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::runtime::core::executor {

class ExecutorBase {
 public:
  ExecutorBase() = default;
  virtual ~ExecutorBase() = default;

  ExecutorBase(const ExecutorBase&) = delete;
  ExecutorBase& operator=(const ExecutorBase&) = delete;

  virtual void Initialize(std::string_view name, YAML::Node options_node) = 0;
  virtual void Start() = 0;
  virtual void Shutdown() = 0;

  virtual std::list<std::pair<std::string, std::string>> GenInitializationReport() const noexcept { return {}; }

  virtual std::string_view Type() const noexcept = 0;  // It should always return the same value
  virtual std::string_view Name() const noexcept = 0;  // It should always return the same value

  virtual bool ThreadSafe() const noexcept = 0;            // It should always return the same value
  virtual bool SupportTimerSchedule() const noexcept = 0;  // It should always return the same value

  /**
   * @brief Check if the caller is executing on this executor
   * @note
   * 1. This method will only be called after 'Initialize' and before 'Shutdown'.
   * 2. If return true, then it must be inside this executor, otherwise it may not be.
   *
   * @return Check result
   */
  virtual bool IsInCurrentExecutor() const noexcept = 0;

  /**
   * @brief Execute a task
   * @note
   * 1. This method will only be called after 'Initialize' and before 'Shutdown'.
   * 2. The Executor can define the actual behavior.
   *
   * @param task
   */
  virtual void Execute(aimrt::executor::Task&& task) noexcept = 0;

  /**
   * @brief Get the time point in this executor
   * @note
   * 1. This method will only be called after 'Initialize' and before 'Shutdown'.
   * 2. The Executor can define the actual behavior.
   *
   * @return std::chrono::system_clock::time_point
   */
  virtual std::chrono::system_clock::time_point Now() const noexcept = 0;

  /**
   * @brief Execute a task at a spectial time point in this executor
   * @note
   * 1. This method will only be called after 'Initialize' and before 'Shutdown'.
   * 2. The Executor can define the actual behavior.
   *
   * @param tp
   * @param task
   */
  virtual void ExecuteAt(std::chrono::system_clock::time_point tp, aimrt::executor::Task&& task) noexcept = 0;

  /**
   * @brief Get current number of tasks in the executor
   * @note
   * 1. This method will only be called after 'Initialize' and before 'Shutdown'.
   * 2. Executor should return the number of unexecuted tasks.
   * 3. This is an optional interface that may return 0 when the executor does not implement it.
   *
   * @return size_t
   */
  virtual size_t CurrentTaskNum() noexcept { return 0; }
};

}  // namespace aimrt::runtime::core::executor
