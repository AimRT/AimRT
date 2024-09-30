// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_cpp_interface/module_base.h"

#include "benchmark.pb.h"

namespace aimrt::examples::cpp::pb_chn::benchmark_subscriber_module {

class BenchmarkSubscriberModule : public aimrt::ModuleBase {
 public:
  BenchmarkSubscriberModule() = default;
  ~BenchmarkSubscriberModule() override = default;

  ModuleInfo Info() const override {
    return ModuleInfo{.name = "BenchmarkSubscriberModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;

  bool Start() override;

  void Shutdown() override;

 private:
  auto GetLogger() const { return core_.GetLogger(); }

  void BenchmarkSignalHandle(
      const std::shared_ptr<const aimrt::protocols::example::BenchmarkSignal>& data);
  void BenchmarkMessageHandle(
      uint32_t topic_index,
      const std::shared_ptr<const aimrt::protocols::example::BenchmarkMessage>& data);

  void Evaluate() const;

 private:
  aimrt::CoreRef core_;

  aimrt::channel::SubscriberRef signal_subscriber_;  // topic name: benchmark_signal

  uint32_t max_topic_number_;
  std::vector<aimrt::channel::SubscriberRef> subscribers_;  // name: test_topic_x

  enum class State : uint32_t {
    kReadyToRun,
    kRunning,
    kEvaluating,
  };
  std::atomic<State> run_state_ = State::kReadyToRun;

  uint32_t cur_bench_plan_id_ = 0;
  uint32_t cur_bench_topic_number_ = 0;
  uint32_t cur_bench_expect_send_num_ = 0;
  uint32_t cur_bench_message_size_ = 0;
  uint32_t cur_bench_send_frequency_ = 0;

  struct TopicRecord {
    std::string topic_name;

    struct MsgRecord {
      bool recv = false;
      uint64_t send_timestamp = 0;
      uint64_t recv_timestamp = 0;
    };
    std::vector<MsgRecord> msg_record_vec;
  };
  std::vector<TopicRecord> topic_record_vec_;
};

}  // namespace aimrt::examples::cpp::pb_chn::benchmark_subscriber_module