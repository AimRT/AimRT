// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <future>

#include "aimrt_module_cpp_interface/module_base.h"

namespace aimrt::examples::cpp::logger::logger_module {

class LoggerModule : public aimrt::ModuleBase {
 public:
  LoggerModule() = default;
  ~LoggerModule() override = default;

  ModuleInfo Info() const override {
    return ModuleInfo{.name = "LoggerModule"};
  }

  bool Initialize(aimrt::CoreRef aimrt_ptr) override;

  bool Start() override;

  void Shutdown() override;

 private:
  aimrt::CoreRef core_;
  aimrt::executor::ExecutorRef executor_;

  std::atomic_bool run_flag_ = true;
  std::promise<void> stop_sig_;
};

}  // namespace aimrt::examples::cpp::logger::logger_module
