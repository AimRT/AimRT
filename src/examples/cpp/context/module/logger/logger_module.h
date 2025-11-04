// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <future>
#include "aimrt_module_cpp_interface/context/context.h"
#include "aimrt_module_cpp_interface/module_base.h"

namespace aimrt::examples::cpp::context::logger_module {

class LoggerModule : public aimrt::ModuleBase {
 public:
  LoggerModule() = default;
  ~LoggerModule() override = default;

  ModuleInfo Info() const override {
    return ModuleInfo{.name = "ContextLoggerModule"};
  }

  bool Initialize(aimrt::CoreRef aimrt_ptr) override;

  bool Start() override;

  void Shutdown() override;

 private:
  std::shared_ptr<aimrt::context::Context> ctx_ptr_;
  aimrt::executor::ExecutorRef executor_;
};

}  // namespace aimrt::examples::cpp::context::logger_module
