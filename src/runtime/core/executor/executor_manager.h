// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "aimrt_module_cpp_interface/executor/executor.h"
#include "core/executor/executor_proxy.h"
#include "core/util/module_detail_info.h"
#include "util/log_util.h"
#include "util/string_util.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::runtime::core::executor {

class ExecutorManager {
 public:
  struct Options {
    struct ExecutorOptions {
      std::string name;
      std::string type;
      YAML::Node options;
    };
    std::vector<ExecutorOptions> executors_options;
  };

  enum class State : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

  using ExecutorGenFunc = std::function<std::unique_ptr<ExecutorBase>()>;

 public:
  ExecutorManager()
      : logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}
  ~ExecutorManager() = default;

  ExecutorManager(const ExecutorManager&) = delete;
  ExecutorManager& operator=(const ExecutorManager&) = delete;

  void Initialize(YAML::Node options_node);
  void Start();
  void Shutdown();

  void RegisterExecutorGenFunc(std::string_view type,
                               ExecutorGenFunc&& executor_gen_func);

  const ExecutorManagerProxy& GetExecutorManagerProxy(const util::ModuleDetailInfo& module_info);
  const ExecutorManagerProxy& GetExecutorManagerProxy(std::string_view module_name = "core") {
    return GetExecutorManagerProxy(
        util::ModuleDetailInfo{.name = std::string(module_name), .pkg_path = "core"});
  }

  State GetState() const { return state_.load(); }

  std::list<std::pair<std::string, std::string>> GenInitializationReport() const;

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) { logger_ptr_ = logger_ptr; }
  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

  aimrt::executor::ExecutorRef GetExecutor(std::string_view executor_name);

  void SetUsedExecutorName(std::string_view executor_name) {
    used_executor_names_.emplace(executor_name);
  }

  const std::vector<std::unique_ptr<ExecutorBase>>& GetAllExecutors() const {
    return executor_vec_;
  }

  const std::unordered_map<std::string, ExecutorGenFunc>& GetExecutorGenFuncMap() const {
    return executor_gen_func_map_;
  }

 private:
  void RegisterAsioExecutorGenFunc();
  void RegisterTBBExecutorGenFunc();
  void RegisterSimpleThreadExecutorGenFunc();
  void RegisterTImeWheelExecutorGenFunc();

 private:
  Options options_;
  std::atomic<State> state_ = State::kPreInit;
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  std::unordered_map<std::string, ExecutorGenFunc> executor_gen_func_map_;

  std::set<std::string> used_executor_names_;

  std::vector<std::unique_ptr<ExecutorBase>> executor_vec_;

  std::unordered_map<
      std::string,
      std::unique_ptr<ExecutorProxy>,
      aimrt::common::util::StringHash,
      std::equal_to<>>
      executor_proxy_map_;

  std::unordered_map<
      std::string,
      std::unique_ptr<ExecutorManagerProxy>>
      executor_manager_proxy_map_;
};

}  // namespace aimrt::runtime::core::executor
