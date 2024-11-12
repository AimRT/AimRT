// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <fstream>
#include <shared_mutex>
#include <unordered_map>
#include "aimrt_module_cpp_interface/executor/executor.h"
#include "aimrt_module_cpp_interface/executor/timer.h"
#include "core/logger/formatter.h"
#include "core/logger/logger_backend_base.h"
#include "util/string_util.h"

namespace aimrt::runtime::core::logger {

class RotateFileLoggerBackend : public LoggerBackendBase {
 public:
  struct Options {
    std::string path = "./log";
    std::string filename = "aimrt.log";
    uint32_t max_file_size_m = 16;
    uint32_t max_file_num = 100;
    std::string module_filter = "(.*)";
    std::string log_executor_name = "";
    std::string pattern;
    uint32_t flush_interval_ms = 1000;
  };

 public:
  RotateFileLoggerBackend() = default;
  ~RotateFileLoggerBackend() override;

  std::string_view Type() const noexcept override { return "rotate_file"; }

  void Initialize(YAML::Node options_node) override;
  void Start() override {}
  void Shutdown() override { run_flag_.store(false); }

  void RegisterGetExecutorFunc(
      const std::function<aimrt::executor::ExecutorRef(std::string_view)>& get_executor_func) {
    get_executor_func_ = get_executor_func;
  }

  bool AllowDuplicates() const noexcept override { return true; }

  void Log(const LogDataWrapper& log_data_wrapper) noexcept override;

 private:
  bool OpenNewFile();
  void CleanLogFile();
  uint32_t GetNextIndex();
  bool CheckLog(const LogDataWrapper& log_data_wrapper);

 private:
  Options options_;
  std::function<aimrt::executor::ExecutorRef(std::string_view)> get_executor_func_;
  executor::ExecutorRef log_executor_;

  std::string base_file_name_;
  std::ofstream ofs_;

  std::atomic_bool run_flag_ = false;

  std::shared_mutex module_filter_map_mutex_;
  std::unordered_map<
      std::string, bool, aimrt::common::util::StringHash, std::equal_to<>>
      module_filter_map_;
  LogFormatter formatter_;
  std::string pattern_ = "[%c.%f][%l][%t][%n][%g:%R:%C @%F]%v";

  std::shared_ptr<aimrt::executor::TimerBase> flush_timer_;
};

}  // namespace aimrt::runtime::core::logger
