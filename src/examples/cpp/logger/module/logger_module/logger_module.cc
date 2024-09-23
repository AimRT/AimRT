// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "logger_module/logger_module.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::examples::cpp::logger::logger_module {

bool LoggerModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;

  // Get executor handle
  executor_ = core_.GetExecutorManager().GetExecutor("work_executor");
  AIMRT_HL_CHECK_ERROR_THROW(
      core_.GetLogger(), executor_, "Get executor 'work_thread_pool' failed.");

  AIMRT_HL_INFO(core_.GetLogger(), "Init succeeded.");

  return true;
}

bool LoggerModule::Start() {
  executor_.Execute([this]() {
    // Create log handle for current scope
    auto GetLogger = [this]() { return core_.GetLogger(); };

    std::string s = "abc";
    int n = 0;

    while (run_flag_.load()) {
      ++n;

      AIMRT_TRACE("Test trace log, s = {}, n = {}.", s, n);
      AIMRT_DEBUG("Test debug log, s = {}, n = {}.", s, n);
      AIMRT_INFO("Test info log, s = {}, n = {}.", s, n);
      AIMRT_WARN("Test warn log, s = {}, n = {}.", s, n);
      AIMRT_ERROR("Test error log, s = {}, n = {}.", s, n);
      AIMRT_FATAL("Test fatal log, s = {}, n = {}.", s, n);

      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    stop_sig_.set_value();
  });

  AIMRT_HL_INFO(core_.GetLogger(), "Start succeeded.");

  return true;
}

void LoggerModule::Shutdown() {
  run_flag_ = false;
  stop_sig_.get_future().wait();
  AIMRT_HL_INFO(core_.GetLogger(), "Shutdown succeeded.");
}

}  // namespace aimrt::examples::cpp::logger::logger_module
