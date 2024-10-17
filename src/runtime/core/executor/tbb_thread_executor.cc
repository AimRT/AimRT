// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/executor/tbb_thread_executor.h"
#include "core/util/thread_tools.h"

namespace YAML {
template <>
struct convert<aimrt::runtime::core::executor::TBBThreadExecutor::Options> {
  using Options = aimrt::runtime::core::executor::TBBThreadExecutor::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["thread_num"] = rhs.thread_num;
    node["thread_sched_policy"] = rhs.thread_sched_policy;
    node["thread_bind_cpu"] = rhs.thread_bind_cpu;
    node["queue_threshold"] = rhs.queue_threshold;

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    if (node["thread_num"]) rhs.thread_num = node["thread_num"].as<uint32_t>();
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

void TBBThreadExecutor::Initialize(std::string_view name, YAML::Node options_node) {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "TBBThreadExecutor can only be initialized once.");

  name_ = std::string(name);
  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  queue_threshold_ = options_.queue_threshold;
  queue_warn_threshold_ = queue_threshold_ * 0.95;

  AIMRT_CHECK_ERROR_THROW(
      options_.thread_num > 0,
      "Invalide tbb thread executor options, thread num is zero.");

  thread_id_vec_.resize(options_.thread_num);

  for (uint32_t ii = 0; ii < options_.thread_num; ++ii) {
    threads_.emplace_back([this, ii] {
      ++work_thread_num_;

      thread_id_vec_[ii] = std::this_thread::get_id();

      std::string threadname = name_;
      if (options_.thread_num > 1)
        threadname = threadname + "." + std::to_string(ii);

      try {
        util::SetNameForCurrentThread(threadname);
        util::BindCpuForCurrentThread(options_.thread_bind_cpu);
        util::SetCpuSchedForCurrentThread(options_.thread_sched_policy);
      } catch (const std::exception& e) {
        AIMRT_WARN("Set thread policy for tbb thread executor '{}' get exception, {}",
                   Name(), e.what());
      }

      aimrt::executor::Task task;
      while (true) {
        try {
          while (qu_.try_pop(task)) {
            task();
            --queue_task_num_;
          }
        } catch (const std::exception& e) {
          AIMRT_FATAL("Tbb thread executor '{}' run loop get exception, {}",
                      Name(), e.what());
        }

        if (state_.load() == State::kShutdown) break;

        try {
          qu_.pop(task);
          task();
          --queue_task_num_;
        } catch (const tbb::user_abort& e) {
        } catch (const std::exception& e) {
          AIMRT_FATAL("Tbb thread executor '{}' run loop get exception, {}",
                      Name(), e.what());
        }
      }

      thread_id_vec_[ii] = std::thread::id();

      --work_thread_num_;
    });
  }

  options_node = options_;
}

void TBBThreadExecutor::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");
}

void TBBThreadExecutor::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  while (work_thread_num_.load()) {
    qu_.abort();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  for (auto itr = threads_.begin(); itr != threads_.end();) {
    if (itr->joinable()) itr->join();
    threads_.erase(itr++);
  }
}

bool TBBThreadExecutor::IsInCurrentExecutor() const noexcept {
  try {
    auto finditr = std::find(thread_id_vec_.begin(), thread_id_vec_.end(), std::this_thread::get_id());
    return (finditr != thread_id_vec_.end());
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
  }
  return false;
}

void TBBThreadExecutor::Execute(aimrt::executor::Task&& task) noexcept {
  if (state_.load() != State::kInit && state_.load() != State::kStart) [[unlikely]] {
    fprintf(stderr,
            "Tbb thread executor '%s' can only execute task when state is 'Init' or 'Start'.\n",
            name_.c_str());
    return;
  }

  uint32_t cur_queue_task_num = ++queue_task_num_;

  if (cur_queue_task_num > queue_threshold_) [[unlikely]] {
    fprintf(stderr,
            "The number of tasks in the tbb thread executor '%s' has reached the threshold '%u', the task will not be delivered.\n",
            name_.c_str(), queue_threshold_);
    --queue_task_num_;
    return;
  }

  if (cur_queue_task_num > queue_warn_threshold_) [[unlikely]] {
    fprintf(stderr,
            "The number of tasks in the tbb thread executor '%s' is about to reach the threshold: '%u / %u'.\n",
            name_.c_str(), cur_queue_task_num, queue_threshold_);
  }

  try {
    qu_.emplace(std::move(task));
  } catch (const std::exception& e) {
    fprintf(stderr, "Tbb thread executor '%s' execute task get exception: %s\n", name_.c_str(), e.what());
  }
}

void TBBThreadExecutor::ExecuteAt(
    std::chrono::system_clock::time_point tp, aimrt::executor::Task&& task) noexcept {
  AIMRT_ERROR("Tbb thread executor '{}' does not support timer schedule.", Name());
}

}  // namespace aimrt::runtime::core::executor
