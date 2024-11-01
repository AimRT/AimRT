// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <mutex>
#include <atomic>
#include <memory>
#include <unordered_map>
#include <vector>

#include "aimrt_module_cpp_interface/executor/executor.h"
#include "core/logger/logger_proxy.h"
#include "core/util/module_detail_info.h"
#include "util/log_util.h"
#include "util/string_util.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::runtime::core::logger {

class LoggerManager {
 public:
  struct Options {
    aimrt_log_level_t core_lvl = aimrt_log_level_t::AIMRT_LOG_LEVEL_INFO;
    aimrt_log_level_t default_module_lvl = aimrt_log_level_t::AIMRT_LOG_LEVEL_INFO;

    struct BackendOptions {
      std::string type;
      YAML::Node options;
    };
    std::vector<BackendOptions> backends_options;
  };

  enum class State : uint32_t {
    kPreInit = 0,
    kInit,
    kStart,
    kShutdown,
  };

  using LoggerBackendGenFunc = std::function<std::unique_ptr<LoggerBackendBase>()>;

 public:
  LoggerManager()
      : logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}
  ~LoggerManager() = default;

  LoggerManager(const LoggerManager&) = delete;
  LoggerManager& operator=(const LoggerManager&) = delete;

  void Initialize(YAML::Node options_node);
  void Start();
  void Shutdown();

  void RegisterGetExecutorFunc(
      const std::function<aimrt::executor::ExecutorRef(std::string_view)>& get_executor_func);

  void RegisterLoggerBackendGenFunc(
      std::string_view type,
      LoggerBackendGenFunc&& logger_backend_gen_func);

  const LoggerProxy& GetLoggerProxy(const util::ModuleDetailInfo& module_info);
  const LoggerProxy& GetLoggerProxy(std::string_view logger_name = "core");

  std::unordered_map<std::string, aimrt_log_level_t> GetAllLoggerLevels() const;
  void SetLoggerLevels(const std::unordered_map<std::string, aimrt_log_level_t>& logger_lvls);

  State GetState() const { return state_.load(); }

  std::list<std::pair<std::string, std::string>> GenInitializationReport() const;

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) { logger_ptr_ = logger_ptr; }
  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

  const std::vector<std::unique_ptr<LoggerBackendBase>>& GetUsedLoggerBackend() const;

 private:
  void RegisterConsoleLoggerBackendGenFunc();
  void RegisterRotateFileLoggerBackendGenFunc();

 private:
  Options options_;
  std::atomic<State> state_ = State::kPreInit;
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  std::function<aimrt::executor::ExecutorRef(std::string_view)> get_executor_func_;

  std::unordered_map<std::string, LoggerBackendGenFunc> logger_backend_gen_func_map_;

  std::vector<std::unique_ptr<LoggerBackendBase>> logger_backend_vec_;

  std::unordered_map<
      std::string,
      std::unique_ptr<LoggerProxy>,
      aimrt::common::util::StringHash,
      std::equal_to<>>
      logger_proxy_map_;

  mutable std::mutex logger_proxy_map_mutex_;  // 互斥锁，声明为 mutable
};

}  // namespace aimrt::runtime::core::logger
