// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "util/log_util.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::runtime::core::executor {

class MainThreadExecutor {
 public:
  struct Options {
    std::string name = "aimrt_main";
    std::string thread_sched_policy;
    std::vector<uint32_t> thread_bind_cpu;
  };

  enum class State : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

 public:
  MainThreadExecutor()
      : logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}
  ~MainThreadExecutor() = default;

  void Initialize(YAML::Node options_node);
  void Start();
  void Shutdown();

  std::string_view Name() const { return name_; }

  State GetState() const { return state_.load(); }

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) { logger_ptr_ = logger_ptr; }
  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

 private:
  Options options_;
  std::atomic<State> state_ = State::kPreInit;
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  std::string name_;
  std::thread::id main_thread_id_;
};

}  // namespace aimrt::runtime::core::executor
