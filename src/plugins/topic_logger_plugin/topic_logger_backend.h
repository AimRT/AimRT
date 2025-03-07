// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <mutex>
#include <queue>
#include <shared_mutex>
#include <unordered_map>

#include "aimrt_module_cpp_interface/channel/channel_handle.h"
#include "aimrt_module_cpp_interface/executor/executor.h"
#include "aimrt_module_cpp_interface/executor/timer.h"
#include "core/logger/logger_backend_base.h"
#include "topic_logger.pb.h"
#include "util/string_util.h"

namespace aimrt::plugins::topic_logger_plugin {

class TopicLoggerBackend : public runtime::core::logger::LoggerBackendBase {
 public:
  struct Options {
    std::string module_filter = "(.*)";  // default: match all modules
    uint32_t interval_ms = 100;          // default: 100ms
    std::string timer_executor_name;
    std::string topic_name;
  };

 public:
  TopicLoggerBackend() = default;
  ~TopicLoggerBackend() override = default;

  std::string_view Type() const noexcept override { return "topic_logger"; }

  void Initialize(YAML::Node options_node) override;
  void Start() override {}
  void Shutdown() override {
    run_flag_.store(false);
  }

  void RegisterGetExecutorFunc(
      const std::function<aimrt::executor::ExecutorRef(std::string_view)>& get_executor_func) {
    get_executor_func_ = get_executor_func;
  }

  void RegisterGetPublisherRefFunc(
      const std::function<aimrt::channel::PublisherRef(std::string_view)>& get_publisher_ref_func) {
    get_publisher_ref_func_ = get_publisher_ref_func;
  }

  void RegisterLogPublisher();

  void StartupPulisher() {
    publish_flag_ = true;
    timer_ptr->Reset();
  }

  void StopPulisher() {
    run_flag_.store(false);
    publish_flag_ = false;
    timer_ptr->Cancel();
    timer_ptr->SyncWait();
  }

  bool AllowDuplicates() const noexcept override { return true; }

  void Log(const runtime::core::logger::LogDataWrapper& log_data_wrapper) noexcept override;

 private:
  bool CheckLog(const runtime::core::logger::LogDataWrapper& log_data_wrapper);

 private:
  Options options_;

  std::function<aimrt::executor::ExecutorRef(std::string_view)> get_executor_func_;
  aimrt::executor::ExecutorRef timer_executor_;
  std::shared_ptr<aimrt::executor::TimerBase> timer_ptr;

  std::function<aimrt::channel::PublisherRef(std::string_view)> get_publisher_ref_func_;
  aimrt::channel::PublisherRef log_publisher_;

  std::atomic_bool run_flag_ = false;
  bool publish_flag_ = false;

  std::shared_mutex module_filter_map_mutex_;
  std::unordered_map<
      std::string, bool, aimrt::common::util::StringHash, std::equal_to<>>
      module_filter_map_;

  std::mutex mutex_;
  std::queue<aimrt::protocols::topic_logger::SingleLogData> queue_;
};

}  // namespace aimrt::plugins::topic_logger_plugin