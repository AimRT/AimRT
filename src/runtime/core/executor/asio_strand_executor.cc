// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/executor/asio_strand_executor.h"

namespace YAML {
template <>
struct convert<aimrt::runtime::core::executor::AsioStrandExecutor::Options> {
  using Options = aimrt::runtime::core::executor::AsioStrandExecutor::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["bind_asio_thread_executor_name"] = rhs.bind_asio_thread_executor_name;
    node["timeout_alarm_threshold_us"] = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            rhs.timeout_alarm_threshold_us)
            .count());
    node["use_system_clock"] = rhs.use_system_clock;

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    if (node["bind_asio_thread_executor_name"])
      rhs.bind_asio_thread_executor_name =
          node["bind_asio_thread_executor_name"].as<std::string>();
    if (node["timeout_alarm_threshold_us"])
      rhs.timeout_alarm_threshold_us = std::chrono::microseconds(
          node["timeout_alarm_threshold_us"].as<uint64_t>());
    if (node["use_system_clock"])
      rhs.use_system_clock = node["use_system_clock"].as<bool>();

    return true;
  }
};
}  // namespace YAML

namespace aimrt::runtime::core::executor {

void AsioStrandExecutor::Initialize(std::string_view name,
                                    YAML::Node options_node) {
  AIMRT_CHECK_ERROR_THROW(get_asio_handle_,
                          "Get asio handle is not set before initialize.");

  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "AsioStrandExecutor can only be initialized once.");

  name_ = std::string(name);

  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  start_sys_tp_ = std::chrono::system_clock::now();
  start_std_tp_ = std::chrono::steady_clock::now();

  AIMRT_CHECK_ERROR_THROW(
      !options_.bind_asio_thread_executor_name.empty(),
      "Invalide bind asio thread executor name, name is empty.");

  auto* io_ptr = get_asio_handle_(options_.bind_asio_thread_executor_name);

  AIMRT_CHECK_ERROR_THROW(
      io_ptr,
      "Invalide bind asio thread executor name, can not get asio io context.");

  strand_ptr_ = std::make_unique<Strand>(asio::make_strand(*io_ptr));

  options_node = options_;
}

void AsioStrandExecutor::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");
}

void AsioStrandExecutor::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;
}

void AsioStrandExecutor::Execute(aimrt::executor::Task&& task) noexcept {
  if (state_.load() != State::kInit && state_.load() != State::kStart) [[unlikely]] {
    fprintf(stderr,
            "Asio strand executor '%s' can only execute task when state is 'Init' or 'Start'.\n",
            name_.c_str());
    return;
  }

  try {
    asio::post(*strand_ptr_, std::move(task));
  } catch (const std::exception& e) {
    fprintf(stderr, "Asio strand executor '%s' execute Task get exception: %s\n", name_.c_str(), e.what());
  }
}

std::chrono::system_clock::time_point AsioStrandExecutor::Now() const noexcept {
  if (!options_.use_system_clock) {
    return start_sys_tp_ +
           std::chrono::duration_cast<std::chrono::system_clock::time_point::duration>(
               std::chrono::steady_clock::now() - start_std_tp_);
  }

  return std::chrono::system_clock::now();
}

void AsioStrandExecutor::ExecuteAt(
    std::chrono::system_clock::time_point tp, aimrt::executor::Task&& task) noexcept {
  if (state_.load() != State::kInit && state_.load() != State::kStart) [[unlikely]] {
    fprintf(stderr,
            "Asio strand executor '%s' can only execute task when state is 'Init' or 'Start'.\n",
            name_.c_str());
    return;
  }

  try {
    if (!options_.use_system_clock) {
      auto timer_ptr = std::make_shared<asio::steady_timer>(*strand_ptr_);
      timer_ptr->expires_after(tp - Now());
      timer_ptr->async_wait([this, timer_ptr,
                             task{std::move(task)}](asio::error_code ec) {
        if (ec) [[unlikely]] {
          AIMRT_ERROR("Asio strand executor '{}' timer get err, code '{}', msg: {}",
                      Name(), ec.value(), ec.message());
          return;
        }

        auto diff_time = std::chrono::steady_clock::now() - timer_ptr->expiry();

        AIMRT_CHECK_WARN(
            diff_time <= options_.timeout_alarm_threshold_us,
            "Asio strand executor '{}' timer delay too much, error time value '{}', require '{}'. "
            "Perhaps the CPU load is too high",
            Name(), std::chrono::duration_cast<std::chrono::microseconds>(diff_time),
            options_.timeout_alarm_threshold_us);

        task();
      });
    } else {
      auto timer_ptr = std::make_shared<asio::system_timer>(*strand_ptr_);
      timer_ptr->expires_at(tp);
      timer_ptr->async_wait([this, timer_ptr,
                             task{std::move(task)}](asio::error_code ec) {
        if (ec) [[unlikely]] {
          AIMRT_ERROR("Asio strand executor '{}' timer get err, code '{}', msg: {}",
                      Name(), ec.value(), ec.message());
          return;
        }

        auto diff_time = std::chrono::system_clock::now() - timer_ptr->expiry();

        AIMRT_CHECK_WARN(
            diff_time <= options_.timeout_alarm_threshold_us,
            "Asio strand executor '{}' timer delay too much, error time value '{}', require '{}'. "
            "Perhaps the CPU load is too high",
            Name(), std::chrono::duration_cast<std::chrono::microseconds>(diff_time),
            options_.timeout_alarm_threshold_us);

        task();
      });
    }
  } catch (const std::exception& e) {
    fprintf(stderr, "Asio strand executor '%s' execute Task get exception: %s\n", name_.c_str(), e.what());
  }
}

void AsioStrandExecutor::RegisterGetAsioHandle(GetAsioHandle&& handle) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");

  get_asio_handle_ = std::move(handle);
}

}  // namespace aimrt::runtime::core::executor