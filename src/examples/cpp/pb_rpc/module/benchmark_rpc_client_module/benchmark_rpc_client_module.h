// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <memory>

#include "aimrt_module_cpp_interface/co/async_scope.h"
#include "aimrt_module_cpp_interface/co/task.h"
#include "aimrt_module_cpp_interface/module_base.h"

#include "rpc.aimrt_rpc.pb.h"

namespace aimrt::examples::cpp::pb_rpc::benchmark_rpc_client_module {

class BenchmarkRpcClientModule : public aimrt::ModuleBase {
 public:
  BenchmarkRpcClientModule() = default;
  ~BenchmarkRpcClientModule() override = default;

  ModuleInfo Info() const override {
    return ModuleInfo{.name = "BenchmarkRpcClientModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;

  bool Start() override;

  void Shutdown() override;

 private:
  auto GetLogger() { return core_.GetLogger(); }

  co::Task<void> MainLoop();

  co::Task<void> WaitForServiceServer();

  struct BenchPlan {
    enum class PerfMod : uint8_t {
      kBench,
      kFixedFreq
    };

    PerfMod mode = PerfMod::kBench;

    uint32_t freq;
    uint32_t msg_size;
    uint32_t parallel;
    uint32_t msg_count;
  };

  co::Task<void> StartSinglePlan(uint32_t plan_id, BenchPlan plan);
  co::Task<void> StartFixedFreqPlan(uint32_t parallel_id, BenchPlan plan, std::vector<uint32_t>& perf_data);
  co::Task<void> StartBenchPlan(uint32_t parallel_id, BenchPlan plan, std::vector<uint32_t>& perf_data);

 private:
  aimrt::CoreRef core_;

  co::AsyncScope scope_;
  std::atomic_bool run_flag_ = true;

  std::shared_ptr<aimrt::protocols::example::ExampleServiceCoProxy> proxy_;

  aimrt::executor::ExecutorRef client_statistics_executor_;  // name: client_statistics_executor
  std::vector<aimrt::executor::ExecutorRef> executor_vec_;   // name: client_executor_x

  // cfg
  uint32_t max_parallel_;
  std::vector<BenchPlan> bench_plans_;
};

}  // namespace aimrt::examples::cpp::pb_rpc::benchmark_rpc_client_module
