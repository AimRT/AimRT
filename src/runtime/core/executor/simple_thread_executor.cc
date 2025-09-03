// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/executor/simple_thread_executor.h"
#include "core/util/thread_tools.h"

namespace YAML {
template <>
struct convert<aimrt::runtime::core::executor::SimpleThreadExecutor::Options> {
  using Options = aimrt::runtime::core::executor::SimpleThreadExecutor::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["thread_sched_policy"] = rhs.thread_sched_policy;
    node["thread_bind_cpu"] = rhs.thread_bind_cpu;
    node["queue_threshold"] = rhs.queue_threshold;
    node["threshold_alarm_interval_ms"] = rhs.threshold_alarm_interval_ms;

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    if (node["thread_sched_policy"])
      rhs.thread_sched_policy = node["thread_sched_policy"].as<std::string>();

    if (node["thread_bind_cpu"])
      rhs.thread_bind_cpu = node["thread_bind_cpu"].as<std::vector<uint32_t>>();

    if (node["queue_threshold"])
      rhs.queue_threshold = node["queue_threshold"].as<uint32_t>();

    if (node["threshold_alarm_interval_ms"])
      rhs.threshold_alarm_interval_ms = node["threshold_alarm_interval_ms"].as<int32_t>();

    return true;
  }
};
}  // namespace YAML

namespace aimrt::runtime::core::executor {

void SimpleThreadExecutor::Initialize(std::string_view name, YAML::Node options_node) {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "SimpleThreadExecutor can only be initialized once.");

  name_ = std::string(name);
  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  queue_threshold_ = options_.queue_threshold;
  queue_warn_threshold_ = queue_threshold_ * 0.95;

  thread_ptr_ = std::make_unique<std::thread>([this]() {
    thread_id_ = std::this_thread::get_id();

    try {
      util::SetNameForCurrentThread(name_);
      util::BindCpuForCurrentThread(options_.thread_bind_cpu);
      util::SetCpuSchedForCurrentThread(options_.thread_sched_policy);
    } catch (const std::exception& e) {
      AIMRT_WARN("Set thread policy for simple thread executor '{}' get exception, {}",
                 Name(), e.what());
    }

    while (state_.load() != State::kShutdown) {
      // Multi-producer-single-consumer optimization
      std::queue<aimrt::executor::Task> tmp_queue;

      {
        std::unique_lock<std::mutex> lck(mutex_);
        cond_.wait(lck, [this] { return !queue_.empty() || state_.load() == State::kShutdown; });
        queue_.swap(tmp_queue);
      }

      while (!tmp_queue.empty()) {
        auto& task = tmp_queue.front();

        try {
          task();
          --queue_task_num_;
        } catch (const std::exception& e) {
          AIMRT_FATAL("Simple thread executor run task get exception, {}", e.what());
        }

        tmp_queue.pop();
      }
    }

    // Run once more after shutdown, no need for locks since no more tasks will enter the queue
    while (!queue_.empty()) {
      auto& task = queue_.front();

      try {
        task();
        --queue_task_num_;
      } catch (const std::exception& e) {
        AIMRT_FATAL("Simple thread executor run task get exception, {}", e.what());
      }

      queue_.pop();
    }

    thread_id_ = std::thread::id();
  });

  options_node = options_;
}

void SimpleThreadExecutor::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");
}

void SimpleThreadExecutor::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  {
    std::unique_lock<std::mutex> lck(mutex_);
    cond_.notify_one();
  }

  if (thread_ptr_ && thread_ptr_->joinable())
    thread_ptr_->join();

  thread_ptr_.reset();
}

void SimpleThreadExecutor::Execute(aimrt::executor::Task&& task) noexcept {
  if (state_.load() != State::kInit && state_.load() != State::kStart) [[unlikely]] {
    AIMRT_ERROR("Simple thread executor '{}' can only execute task when state is 'Init' or 'Start'.",
                name_);
    return;
  }

  uint32_t cur_queue_task_num = ++queue_task_num_;

  if (options_.threshold_alarm_interval_ms >= 0 && cur_queue_task_num > queue_threshold_) [[unlikely]] {
    AIMRT_WARN_INTERVAL(static_cast<uint64_t>(options_.threshold_alarm_interval_ms),
                        "The number of tasks in the simple thread executor '{}' has reached the threshold '{}', the task will not be delivered.",
                        name_, queue_threshold_);
    --queue_task_num_;
    return;
  }

  if (options_.threshold_alarm_interval_ms >= 0 && cur_queue_task_num > queue_warn_threshold_) [[unlikely]] {
    AIMRT_WARN_INTERVAL(static_cast<uint64_t>(options_.threshold_alarm_interval_ms),
                        "The number of tasks in the simple thread executor '{}' is about to reach the threshold: '{} / {}'.",
                        name_, cur_queue_task_num, queue_threshold_);
  }

  std::unique_lock<std::mutex> lck(mutex_);
  queue_.emplace(std::move(task));
  cond_.notify_one();
}

std::chrono::system_clock::time_point SimpleThreadExecutor::Now() const noexcept {
  AIMRT_ERROR("Simple thread executor '{}' does not support timer schedule.", Name());
  return std::chrono::system_clock::time_point();
}

void SimpleThreadExecutor::ExecuteAt(
    std::chrono::system_clock::time_point tp, aimrt::executor::Task&& task) noexcept {
  AIMRT_ERROR("Simple thread executor '{}' does not support timer schedule.", Name());
}

}  // namespace aimrt::runtime::core::executor
