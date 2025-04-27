// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/executor/guard_thread_executor.h"
#include "core/util/thread_tools.h"

namespace YAML {
template <>
struct convert<aimrt::runtime::core::executor::GuardThreadExecutor::Options> {
  using Options = aimrt::runtime::core::executor::GuardThreadExecutor::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["name"] = rhs.name;
    node["thread_sched_policy"] = rhs.thread_sched_policy;
    node["thread_bind_cpu"] = rhs.thread_bind_cpu;
    node["queue_threshold"] = rhs.queue_threshold;

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    if (node["name"])
      rhs.name = node["name"].as<std::string>();

    if (node["thread_sched_policy"])
      rhs.thread_sched_policy = node["thread_sched_policy"].as<std::string>();

    if (node["thread_bind_cpu"])
      rhs.thread_bind_cpu = node["thread_bind_cpu"].as<std::vector<uint32_t>>();

    if (node["queue_threshold"])
      rhs.queue_threshold = node["queue_threshold"].as<uint32_t>();

    return true;
  }
};
}  // namespace YAML

namespace aimrt::runtime::core::executor {

void GuardThreadExecutor::Initialize(YAML::Node options_node) {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "GuardThreadExecutor can only be initialized once.");

  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  name_ = options_.name;

  queue_threshold_ = options_.queue_threshold;
  queue_warn_threshold_ = queue_threshold_ * 0.95;

  thread_ptr_ = std::make_unique<std::thread>([this]() {
    thread_id_ = std::this_thread::get_id();

    try {
      util::SetNameForCurrentThread(name_);
      util::BindCpuForCurrentThread(options_.thread_bind_cpu);
      util::SetCpuSchedForCurrentThread(options_.thread_sched_policy);
    } catch (const std::exception& e) {
      AIMRT_WARN("Set thread policy for guard thread executor '{}' get exception, {}",
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
          AIMRT_FATAL("Guard thread executor run task get exception, {}", e.what());
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
        AIMRT_FATAL("Guard thread executor run task get exception, {}", e.what());
      }

      queue_.pop();
    }

    thread_id_ = std::thread::id();
  });

  options_node = options_;

  AIMRT_INFO("Guard thread executor init complete");
}

void GuardThreadExecutor::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");
}

void GuardThreadExecutor::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  {
    std::unique_lock<std::mutex> lck(mutex_);
    cond_.notify_one();
  }

  if (thread_ptr_ && thread_ptr_->joinable())
    thread_ptr_->join();

  thread_ptr_.reset();

  AIMRT_INFO("Guard thread executor shutdown.");
}

void GuardThreadExecutor::Execute(aimrt::executor::Task&& task) {
  if (state_.load() != State::kInit && state_.load() != State::kStart) [[unlikely]] {
    fprintf(stderr, "Guard thread executor can only execute task when state is 'Init' or 'Start'.\n");
    return;
  }

  uint32_t cur_queue_task_num = ++queue_task_num_;

  if (cur_queue_task_num > queue_threshold_) [[unlikely]] {
    fprintf(stderr,
            "The number of tasks in the guard thread executor has reached the threshold '%u', the task will not be delivered.\n",
            queue_threshold_);
    --queue_task_num_;
    return;
  }

  if (cur_queue_task_num > queue_warn_threshold_) [[unlikely]] {
    fprintf(stderr,
            "The number of tasks in the guard thread executor is about to reach the threshold: '%u / %u'.\n",
            cur_queue_task_num, queue_threshold_);
  }

  std::unique_lock<std::mutex> lck(mutex_);
  queue_.emplace(std::move(task));
  cond_.notify_one();
}

}  // namespace aimrt::runtime::core::executor
