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

  AIMRT_CHECK_ERROR_THROW(
      !options_.bind_asio_thread_executor_name.empty(),
      "Invalide bind asio thread executor name, name is empty.");

  auto* io_ptr = get_asio_handle_(options_.bind_asio_thread_executor_name);

  AIMRT_CHECK_ERROR_THROW(
      io_ptr,
      "Invalide bind asio thread executor name, can not get asio io context.");

  strand_ptr_ = std::make_unique<Strand>(boost::asio::make_strand(*io_ptr));

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
  try {
    boost::asio::post(*strand_ptr_, std::move(task));
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
  }
}

void AsioStrandExecutor::ExecuteAt(
    std::chrono::system_clock::time_point tp, aimrt::executor::Task&& task) noexcept {
  try {
    auto timer_ptr = std::make_shared<boost::asio::system_timer>(*strand_ptr_);
    timer_ptr->expires_at(tp);
    timer_ptr->async_wait([this, timer_ptr,
                           task{std::move(task)}](boost::system::error_code ec) {
      if (ec) [[unlikely]] {
        AIMRT_ERROR("Asio strand executor '{}' timer get err, code '{}', msg: {}",
                    Name(), ec.value(), ec.message());
        return;
      }

      auto dif_time = std::chrono::system_clock::now() - timer_ptr->expiry();

      task();

      AIMRT_CHECK_WARN(
          dif_time <= options_.timeout_alarm_threshold_us,
          "Asio strand executor '{}' timer delay too much, error time value '{}', require '{}'. "
          "Perhaps the CPU load is too high",
          Name(), std::chrono::duration_cast<std::chrono::microseconds>(dif_time),
          options_.timeout_alarm_threshold_us);
    });
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
  }
}

void AsioStrandExecutor::RegisterGetAsioHandle(GetAsioHandle&& handle) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");

  get_asio_handle_ = std::move(handle);
}

}  // namespace aimrt::runtime::core::executor