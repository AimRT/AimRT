// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/executor/asio_thread_executor.h"
#include "core/util/thread_tools.h"

namespace YAML {
template <>
struct convert<aimrt::runtime::core::executor::AsioThreadExecutor::Options> {
  using Options = aimrt::runtime::core::executor::AsioThreadExecutor::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["thread_num"] = rhs.thread_num;
    node["thread_sched_policy"] = rhs.thread_sched_policy;
    node["thread_bind_cpu"] = rhs.thread_bind_cpu;
    node["timeout_alarm_threshold_us"] = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            rhs.timeout_alarm_threshold_us)
            .count());
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
    if (node["timeout_alarm_threshold_us"])
      rhs.timeout_alarm_threshold_us = std::chrono::microseconds(
          node["timeout_alarm_threshold_us"].as<uint64_t>());
    if (node["queue_threshold"])
      rhs.queue_threshold = node["queue_threshold"].as<uint32_t>();

    return true;
  }
};
}  // namespace YAML

namespace aimrt::runtime::core::executor {

void AsioThreadExecutor::Initialize(std::string_view name,
                                    YAML::Node options_node) {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "AsioThreadExecutor can only be initialized once.");

  name_ = std::string(name);
  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  queue_threshold_ = options_.queue_threshold;
  queue_warn_threshold_ = queue_threshold_ * 0.95;

  AIMRT_CHECK_ERROR_THROW(
      options_.thread_num > 0,
      "Invalide asio thread executor options, thread num is zero.");

  io_ptr_ = std::make_unique<asio::io_context>(options_.thread_num);
  work_guard_ptr_ = std::make_unique<
      asio::executor_work_guard<asio::io_context::executor_type>>(
      io_ptr_->get_executor());

  thread_id_vec_.resize(options_.thread_num);

  for (uint32_t ii = 0; ii < options_.thread_num; ++ii) {
    threads_.emplace_back([this, ii] {
      thread_id_vec_[ii] = std::this_thread::get_id();

      std::string threadname = name_;
      if (options_.thread_num > 1)
        threadname = threadname + "." + std::to_string(ii);

      try {
        util::SetNameForCurrentThread(threadname);
        util::BindCpuForCurrentThread(options_.thread_bind_cpu);
        util::SetCpuSchedForCurrentThread(options_.thread_sched_policy);
      } catch (const std::exception& e) {
        AIMRT_WARN("Set thread policy for asio thread executor '{}' get exception, {}",
                   Name(), e.what());
      }

      try {
        while (io_ptr_->run_one()) {
          --queue_task_num_;
        }
      } catch (const std::exception& e) {
        AIMRT_FATAL("Asio thread executor '{}' run loop get exception, {}",
                    Name(), e.what());
      }

      thread_id_vec_[ii] = std::thread::id();
    });
  }

  options_node = options_;
}

void AsioThreadExecutor::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");
}

void AsioThreadExecutor::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  if (work_guard_ptr_) work_guard_ptr_->reset();

  for (auto itr = threads_.begin(); itr != threads_.end();) {
    if (itr->joinable()) itr->join();
    threads_.erase(itr++);
  }
}

bool AsioThreadExecutor::IsInCurrentExecutor() const noexcept {
  try {
    auto finditr = std::find(thread_id_vec_.begin(), thread_id_vec_.end(), std::this_thread::get_id());
    return (finditr != thread_id_vec_.end());
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
  }
  return false;
}

void AsioThreadExecutor::Execute(aimrt::executor::Task&& task) noexcept {
  if (state_.load() != State::kInit && state_.load() != State::kStart) [[unlikely]] {
    fprintf(stderr,
            "Asio thread executor '%s' can only execute task when state is 'Init' or 'Start'.\n",
            name_.c_str());
    return;
  }

  uint32_t cur_queue_task_num = ++queue_task_num_;

  if (cur_queue_task_num > queue_threshold_) [[unlikely]] {
    fprintf(stderr,
            "The number of tasks in the asio thread executor '%s' has reached the threshold '%u', the task will not be delivered.\n",
            name_.c_str(), queue_threshold_);
    --queue_task_num_;
    return;
  }

  if (cur_queue_task_num > queue_warn_threshold_) [[unlikely]] {
    fprintf(stderr,
            "The number of tasks in the asio thread executor '%s' is about to reach the threshold: '%u / %u'.\n",
            name_.c_str(), cur_queue_task_num, queue_threshold_);
  }

  try {
    asio::post(*io_ptr_, std::move(task));
  } catch (const std::exception& e) {
    fprintf(stderr, "Asio thread executor '%s' execute Task get exception: %s\n", name_.c_str(), e.what());
  }
}

void AsioThreadExecutor::ExecuteAt(
    std::chrono::system_clock::time_point tp, aimrt::executor::Task&& task) noexcept {
  if (state_.load() != State::kInit && state_.load() != State::kStart) [[unlikely]] {
    fprintf(stderr,
            "Asio thread executor '%s' can only execute task when state is 'Init' or 'Start'.\n",
            name_.c_str());
    return;
  }

  uint32_t cur_queue_task_num = ++queue_task_num_;

  if (cur_queue_task_num > queue_threshold_) [[unlikely]] {
    fprintf(stderr,
            "The number of tasks in the asio thread executor '%s' has reached the threshold '%u', the task will not be delivered.\n",
            name_.c_str(), queue_threshold_);
    --queue_task_num_;
    return;
  }

  if (cur_queue_task_num > queue_warn_threshold_) [[unlikely]] {
    fprintf(stderr,
            "The number of tasks in the asio thread executor '%s' is about to reach the threshold: '%u / %u'.\n",
            name_.c_str(), cur_queue_task_num, queue_threshold_);
  }

  try {
    auto timer_ptr = std::make_shared<asio::system_timer>(*io_ptr_);
    timer_ptr->expires_at(tp);
    timer_ptr->async_wait([this, timer_ptr,
                           task{std::move(task)}](asio::error_code ec) {
      if (ec) [[unlikely]] {
        AIMRT_ERROR("Asio thread executor '{}' timer get err, code '{}', msg: {}",
                    Name(), ec.value(), ec.message());
        return;
      }

      auto dif_time = std::chrono::system_clock::now() - timer_ptr->expiry();

      task();

      AIMRT_CHECK_WARN(
          dif_time <= options_.timeout_alarm_threshold_us,
          "Asio thread executor '{}' timer delay too much, error time value '{}', require '{}'. "
          "Perhaps the CPU load is too high",
          Name(), std::chrono::duration_cast<std::chrono::microseconds>(dif_time),
          options_.timeout_alarm_threshold_us);
    });
  } catch (const std::exception& e) {
    fprintf(stderr, "Asio thread executor '%s' execute Task get exception: %s\n", name_.c_str(), e.what());
  }
}

}  // namespace aimrt::runtime::core::executor
