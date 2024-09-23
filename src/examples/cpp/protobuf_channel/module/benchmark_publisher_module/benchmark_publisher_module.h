// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <future>

#include "aimrt_module_cpp_interface/module_base.h"

namespace aimrt::examples::cpp::protobuf_channel::benchmark_publisher_module {

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
    uint32_t channel_frq;
    uint32_t msg_size;
    uint32_t topic_number;
    uint32_t msg_count;
  };

  void StartSinglePlan(uint32_t plan_id, BenchPlan plan);

 private:
  aimrt::CoreRef core_;

  std::atomic_bool run_flag_ = true;
  std::promise<void> stop_sig_;

  aimrt::executor::ExecutorRef publish_control_executor_;  // name: publish_control_executor
  aimrt::channel::PublisherRef signal_publisher_;          // topic name: benchmark_signal

  struct PublisherWrapper {
    aimrt::executor::ExecutorRef publish_executor;  // name: publish_executor_x
    aimrt::channel::PublisherRef publisher;         // name: test_topic_x
  };
  std::vector<PublisherWrapper> publisher_wrapper_vec_;

  // cfg
  uint32_t max_topic_number_;
  std::vector<BenchPlan> bench_plans_;
};

}  // namespace aimrt::examples::cpp::protobuf_channel::benchmark_publisher_module
