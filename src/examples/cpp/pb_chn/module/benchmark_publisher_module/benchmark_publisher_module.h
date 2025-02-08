// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <future>

#include "aimrt_module_cpp_interface/module_base.h"

namespace aimrt::examples::cpp::pb_chn::benchmark_publisher_module {

class BenchmarkPublisherModule : public aimrt::ModuleBase {
 public:
  BenchmarkPublisherModule() = default;
  ~BenchmarkPublisherModule() override = default;

  ModuleInfo Info() const override {
    return ModuleInfo{.name = "BenchmarkPublisherModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;

  bool Start() override;

  void Shutdown() override;

 private:
  auto GetLogger() { return core_.GetLogger(); }

  void MainLoop();

  struct BenchPlan {
    enum class PerfMod : uint8_t {
      kMultiTopic,
      kParallel
    };

    PerfMod mode = PerfMod::kMultiTopic;
    uint32_t channel_frq;
    uint32_t msg_size;
    uint32_t topic_number = 1;
    uint32_t parallel_number = 1;
    uint32_t msg_count;
  };

  void StartSinglePlan(uint32_t plan_id, BenchPlan plan);
  void StartMultiTopicPlan(uint32_t plan_id, BenchPlan plan);
  void StartParallelPlan(uint32_t plan_id, BenchPlan plan);

 private:
  aimrt::CoreRef core_;

  std::atomic_bool run_flag_ = true;
  std::promise<void> stop_sig_;

  aimrt::executor::ExecutorRef publish_control_executor_;  // name: publish_control_executor
  aimrt::channel::PublisherRef signal_publisher_;          // topic name: benchmark_signal

  std::vector<aimrt::executor::ExecutorRef> executor_vec_;
  std::vector<aimrt::channel::PublisherRef> publisher_vec_;

  // cfg
  uint32_t max_topic_number_;
  uint32_t max_parallel_number_;
  std::vector<BenchPlan> bench_plans_;
};

}  // namespace aimrt::examples::cpp::pb_chn::benchmark_publisher_module
