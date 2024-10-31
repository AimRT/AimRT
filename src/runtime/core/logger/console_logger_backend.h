// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <shared_mutex>
#include <unordered_map>

#include "aimrt_module_cpp_interface/executor/executor.h"
#include "core/logger/formatter.h"
#include "core/logger/logger_backend_base.h"
#include "util/string_util.h"

namespace aimrt::runtime::core::logger {

class ConsoleLoggerBackend : public LoggerBackendBase {
 public:
  struct Options {
    bool print_color = true;
    std::string module_filter = "(.*)";
    std::string log_executor_name = "";
    std::string pattern;
  };

 public:
  ConsoleLoggerBackend() = default;
  ~ConsoleLoggerBackend() override = default;

  std::string_view Type() const noexcept override { return "console"; }

  void Initialize(YAML::Node options_node) override;
  void Start() override {}
  void Shutdown() override { run_flag_.store(false); }

  void RegisterGetExecutorFunc(
      const std::function<aimrt::executor::ExecutorRef(std::string_view)>& get_executor_func) {
    get_executor_func_ = get_executor_func;
  }

  bool AllowDuplicates() const noexcept override { return false; }

  void Log(const LogDataWrapper& log_data_wrapper) noexcept override;

 private:
  bool CheckLog(const LogDataWrapper& log_data_wrapper);

 private:
  Options options_;
  std::function<aimrt::executor::ExecutorRef(std::string_view)> get_executor_func_;
  aimrt::executor::ExecutorRef log_executor_;
  std::atomic_bool run_flag_ = false;

  std::shared_mutex module_filter_map_mutex_;
  std::unordered_map<
      std::string, bool, aimrt::common::util::StringHash, std::equal_to<>>
      module_filter_map_;

  LogFormatter formatter_;
  std::string pattern_ = "[%c.%f][%l][%t][%n][%g:%R:%C @%F]%v";
};

}  // namespace aimrt::runtime::core::logger
