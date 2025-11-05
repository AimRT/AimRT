// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "logger_module.h"
#include "yaml-cpp/yaml.h"

namespace aimrt::examples::cpp::context::logger_module {

bool LoggerModule::Initialize(aimrt::CoreRef core) {
  ctx_ptr_ = std::make_shared<aimrt::context::Context>(core);
  ctx_ptr_->LetMe();

  // Get executor handle
  executor_ = ctx_ptr_->CreateExecutor("work_executor");

  return true;
}

bool LoggerModule::Start() {
  executor_.Execute([this]() {
    ctx_ptr_->LetMe();

    std::string s = "abc";
    int n = 0;

    while (aimrt::context::Running()) {
      ++n;

      aimrt::context::Log().Trace("Test trace log, s = {}, n = {}.", s, n);
      aimrt::context::Log().Debug("Test debug log, s = {}, n = {}.", s, n);
      aimrt::context::Log().Info("Test info log, s = {}, n = {}.", s, n);
      aimrt::context::Log().Warn("Test warn log, s = {}, n = {}.", s, n);
      aimrt::context::Log().Error("Test error log, s = {}, n = {}.", s, n);
      aimrt::context::Log().Fatal("Test fatal log, s = {}, n = {}.", s, n);

      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  });

  aimrt::context::Log().Info("Start succeeded.");

  return true;
}

void LoggerModule::Shutdown() {
  ctx_ptr_->StopRunning();
  aimrt::context::Log().Info("Shutdown succeeded.");
}

}  // namespace aimrt::examples::cpp::context::logger_module
