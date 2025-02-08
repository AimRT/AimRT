// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "benchmark_publisher_module/benchmark_publisher_module.h"
#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"
#include "util/time_util.h"

#include "yaml-cpp/yaml.h"

#include "benchmark.pb.h"

namespace aimrt::examples::cpp::pb_chn::benchmark_publisher_module {

std::string GenerateRandomString(int min_length, int max_length) {
  static constexpr std::string_view kChars = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
  srand(time(nullptr));

  int length = rand() % (max_length - min_length + 1) + min_length;

  std::string result;
  result.reserve(length);

  for (int i = 0; i < length; ++i) {
    result += kChars[rand() % kChars.length()];
  }

  return result;
}

std::string GenerateRandomString(int length) {
  return GenerateRandomString(length, length);
}

std::vector<uint32_t> GenerateLastSendCounts(uint32_t total, uint32_t concurrency) {
  if (concurrency <= 0) [[unlikely]]
    return {};
  if (concurrency == 1) return {0, total};

  std::vector<uint32_t> last_send_counts(concurrency + 1, 0);
  uint32_t base_count = total / concurrency;

  for (uint32_t i = 1; i < concurrency; ++i) {
    last_send_counts[i] = last_send_counts[i - 1] + base_count;
  }

  last_send_counts[concurrency] = total;

  return last_send_counts;
}

bool BenchmarkPublisherModule::Initialize(aimrt::CoreRef core) {
  core_ = core;

  try {
    // Read cfg
    auto file_path = core_.GetConfigurator().GetConfigFilePath();
    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(file_path));
      max_topic_number_ = cfg_node["max_topic_number"].as<uint32_t>();
      max_parallel_number_ = cfg_node["max_parallel_number"].as<uint32_t>();

      if (cfg_node["bench_plans"] && cfg_node["bench_plans"].IsSequence()) {
        for (const auto& bench_plan_node : cfg_node["bench_plans"]) {
          BenchPlan bench_plan;

          auto perf_mode = bench_plan_node["perf_mode"].as<std::string>();
          if (perf_mode == "multi-topic") {
            bench_plan.mode = BenchPlan::PerfMod::kMultiTopic;
          } else if (perf_mode == "parallel") {
            bench_plan.mode = BenchPlan::PerfMod::kParallel;
          } else {
            throw aimrt::common::util::AimRTException("Unsupport perf mode: " + perf_mode);
          }

          if (bench_plan.mode == BenchPlan::PerfMod::kParallel) {
            bench_plan.parallel_number = bench_plan_node["parallel_number"].as<uint32_t>();
            AIMRT_CHECK_ERROR_THROW(
                bench_plan.parallel_number <= max_parallel_number_,
                "Bench plan parallel number({}) is greater than max parallel number({})",
                bench_plan.parallel_number, max_parallel_number_);
          }

          if (bench_plan.mode == BenchPlan::PerfMod::kMultiTopic) {
            bench_plan.topic_number = bench_plan_node["topic_number"].as<uint32_t>();
            AIMRT_CHECK_ERROR_THROW(
                bench_plan.topic_number <= max_topic_number_,
                "Bench plan topic number({}) is greater than max topic number({})",
                bench_plan.topic_number, max_topic_number_);
          }

          bench_plan.channel_frq = bench_plan_node["channel_frq"].as<uint32_t>();
          bench_plan.msg_size = bench_plan_node["msg_size"].as<uint32_t>();
          bench_plan.msg_count = bench_plan_node["msg_count"].as<uint32_t>();

          bench_plans_.emplace_back(bench_plan);
        }
      }
    }

    // controller
    publish_control_executor_ = core_.GetExecutorManager().GetExecutor("publish_control_executor");
    AIMRT_CHECK_ERROR_THROW(publish_control_executor_, "Get executor 'publish_control_executor' failed.");

    signal_publisher_ = core_.GetChannelHandle().GetPublisher("benchmark_signal");
    AIMRT_CHECK_ERROR_THROW(signal_publisher_, "Get publisher for topic 'benchmark_signal' failed.");

    bool ret = aimrt::channel::RegisterPublishType<aimrt::protocols::example::BenchmarkSignal>(signal_publisher_);
    AIMRT_CHECK_ERROR_THROW(ret, "Register publish type failed.");

    for (uint32_t ii = 0; ii < std::max(max_parallel_number_, max_topic_number_); ++ii) {
      auto executor_name = "publish_executor_" + std::to_string(ii);
      auto executor = core_.GetExecutorManager().GetExecutor(executor_name);
      AIMRT_CHECK_ERROR_THROW(executor, "Get executor '{}' failed.", executor_name);

      executor_vec_.emplace_back(executor);
    }

    for (uint32_t ii = 0; ii < max_topic_number_; ++ii) {
      auto topic_name = "test_topic_" + std::to_string(ii);
      auto publisher = core_.GetChannelHandle().GetPublisher(topic_name);
      AIMRT_CHECK_ERROR_THROW(publisher, "Get publisher for topic '{}' failed.", topic_name);

      bool ret = aimrt::channel::RegisterPublishType<aimrt::protocols::example::BenchmarkMessage>(publisher);
      AIMRT_CHECK_ERROR_THROW(ret, "Register publish type failed.");

      publisher_vec_.emplace_back(publisher);
    }

  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool BenchmarkPublisherModule::Start() {
  try {
    publish_control_executor_.Execute(std::bind(&BenchmarkPublisherModule::MainLoop, this));
  } catch (const std::exception& e) {
    AIMRT_ERROR("Start failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Start succeeded.");
  return true;
}

void BenchmarkPublisherModule::Shutdown() {
  try {
    run_flag_ = false;

    stop_sig_.get_future().wait();

  } catch (const std::exception& e) {
    AIMRT_ERROR("Shutdown failed, {}", e.what());
    return;
  }

  AIMRT_INFO("Shutdown succeeded.");
}

// Main loop
void BenchmarkPublisherModule::MainLoop() {
  try {
    AIMRT_INFO("Start Bench.");

    // warm up
    for (size_t ii = 0; ii < 10; ++ii) {
      aimrt::protocols::example::BenchmarkSignal begin_signal;
      begin_signal.set_status(aimrt::protocols::example::BenchmarkStatus::WarmUp);
      aimrt::channel::Publish(signal_publisher_, begin_signal);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    for (size_t ii = 0; ii < bench_plans_.size(); ++ii) {
      if (!run_flag_.load()) break;

      StartSinglePlan(ii, bench_plans_[ii]);

      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    AIMRT_INFO("Bench completed.");

  } catch (const std::exception& e) {
    AIMRT_ERROR("Exit MainLoop with exception, {}", e.what());
  }

  stop_sig_.set_value();
}

void BenchmarkPublisherModule::StartSinglePlan(uint32_t plan_id, BenchPlan plan) {
  if (plan.mode == BenchPlan::PerfMod::kMultiTopic) {
    StartMultiTopicPlan(plan_id, plan);
  } else {
    StartParallelPlan(plan_id, plan);
  }
}

void BenchmarkPublisherModule::StartMultiTopicPlan(uint32_t plan_id, BenchPlan plan) {
  // publish start signal
  aimrt::protocols::example::BenchmarkSignal begin_signal;
  begin_signal.set_status(aimrt::protocols::example::BenchmarkStatus::Begin);
  begin_signal.set_mode("multi-topic");
  begin_signal.set_bench_plan_id(plan_id);
  begin_signal.set_topic_number(plan.topic_number);
  begin_signal.set_parallel_number(1);
  begin_signal.set_send_num(plan.msg_count);
  begin_signal.set_message_size(plan.msg_size);
  begin_signal.set_send_frequency(plan.channel_frq);

  AIMRT_INFO("Publish benchmark start signal, data: {}", aimrt::Pb2CompactJson(begin_signal));
  aimrt::channel::Publish(signal_publisher_, begin_signal);

  std::this_thread::sleep_for(std::chrono::seconds(1));

  // publish topic
  std::vector<std::future<void>> future_vec;
  for (size_t ii = 0; ii < plan.topic_number; ++ii) {
    auto publish_executor = executor_vec_[ii];
    auto publisher = publisher_vec_[ii];

    std::promise<void> task_promise;
    future_vec.emplace_back(task_promise.get_future());
    publish_executor.Execute(
        [this, publisher, plan, task_promise{std::move(task_promise)}]() mutable {
          aimrt::protocols::example::BenchmarkMessage msg;
          msg.set_data(GenerateRandomString(plan.msg_size));

          uint32_t send_count = 0;

          uint32_t sleep_ns = static_cast<uint32_t>(1000000000 / plan.channel_frq);
          auto cur_tp = std::chrono::steady_clock::now();

          for (; send_count < plan.msg_count; ++send_count) {
            if (!run_flag_.load()) [[unlikely]]
              break;

            msg.set_seq(send_count);
            msg.set_timestamp(aimrt::common::util::GetCurTimestampNs());

            aimrt::channel::Publish(publisher, msg);

            cur_tp += std::chrono::nanoseconds(sleep_ns);
            std::this_thread::sleep_until(cur_tp);
          }

          task_promise.set_value();
        });
  }

  for (auto& fu : future_vec) fu.wait();

  std::this_thread::sleep_for(std::chrono::seconds(1));

  // publish end signal
  aimrt::protocols::example::BenchmarkSignal end_signal;
  end_signal.set_status(aimrt::protocols::example::BenchmarkStatus::End);
  end_signal.set_bench_plan_id(plan_id);

  AIMRT_INFO("Publish benchmark end signal, data: {}", aimrt::Pb2CompactJson(end_signal));
  aimrt::channel::Publish(signal_publisher_, end_signal);
}

void BenchmarkPublisherModule::StartParallelPlan(uint32_t plan_id, BenchPlan plan) {
  // publish start signal
  aimrt::protocols::example::BenchmarkSignal begin_signal;
  begin_signal.set_status(aimrt::protocols::example::BenchmarkStatus::Begin);
  begin_signal.set_bench_plan_id(plan_id);
  begin_signal.set_mode("parallel");
  begin_signal.set_topic_number(1);
  begin_signal.set_parallel_number(plan.parallel_number);
  begin_signal.set_send_num(plan.msg_count);
  begin_signal.set_message_size(plan.msg_size);
  begin_signal.set_send_frequency(plan.channel_frq);

  AIMRT_INFO("Publish benchmark start signal, data: {}", aimrt::Pb2CompactJson(begin_signal));
  aimrt::channel::Publish(signal_publisher_, begin_signal);

  std::this_thread::sleep_for(std::chrono::seconds(1));

  // publish topic
  std::vector<std::future<void>> future_vec;
  std::vector<uint32_t> last_send_counts = GenerateLastSendCounts(plan.msg_count, plan.parallel_number);

  for (size_t ii = 0; ii < plan.parallel_number; ++ii) {
    auto publish_executor = executor_vec_[ii];
    auto publisher = publisher_vec_[0];

    std::promise<void> task_promise;
    future_vec.emplace_back(task_promise.get_future());
    publish_executor.Execute(
        [this, publisher, plan, last_send_counts, ii, task_promise{std::move(task_promise)}]() mutable {
          aimrt::protocols::example::BenchmarkMessage msg;
          msg.set_data(GenerateRandomString(plan.msg_size));

          uint32_t send_count = 0;

          uint32_t sleep_ns = static_cast<uint32_t>(1000000000 / plan.channel_frq);
          auto cur_tp = std::chrono::steady_clock::now();

          uint32_t messages_to_send = last_send_counts[ii + 1] - last_send_counts[ii];
          for (; send_count < messages_to_send; ++send_count) {
            if (!run_flag_.load()) [[unlikely]]
              break;

            msg.set_seq(last_send_counts[ii] + send_count);
            msg.set_timestamp(aimrt::common::util::GetCurTimestampNs());

            aimrt::channel::Publish(publisher, msg);

            cur_tp += std::chrono::nanoseconds(sleep_ns);
            std::this_thread::sleep_until(cur_tp);
          }

          task_promise.set_value();
        });
  }

  for (auto& fu : future_vec) fu.wait();

  std::this_thread::sleep_for(std::chrono::seconds(1));

  // publish end signal
  aimrt::protocols::example::BenchmarkSignal end_signal;
  end_signal.set_status(aimrt::protocols::example::BenchmarkStatus::End);
  end_signal.set_bench_plan_id(plan_id);

  AIMRT_INFO("Publish benchmark end signal, data: {}", aimrt::Pb2CompactJson(end_signal));
  aimrt::channel::Publish(signal_publisher_, end_signal);
}

}  // namespace aimrt::examples::cpp::pb_chn::benchmark_publisher_module