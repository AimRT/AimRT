// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <future>

#include "aimrt_module_cpp_interface/module_base.h"

namespace aimrt::examples::cpp::logger::logger_bench_module {

class LoggerBenchModule : public aimrt::ModuleBase {
 public:
  LoggerBenchModule() = default;
  ~LoggerBenchModule() override = default;

  ModuleInfo Info() const override {
    return ModuleInfo{.name = "LoggerBenchModule"};
  }

  bool Initialize(aimrt::CoreRef aimrt_ptr) override;

  bool Start() override;

  void Shutdown() override;

 private:
  auto GetLogger() { return core_.GetLogger(); }

 private:
  aimrt::CoreRef core_;

  std::vector<uint32_t> log_data_size_vec_;
  uint32_t log_bench_num_ = 10000;

  std::promise<void> log_loop_stop_sig_;
};

}  // namespace aimrt::examples::cpp::logger::logger_bench_module
