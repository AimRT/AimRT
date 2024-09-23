// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/executor/main_thread_executor.h"
#include "core/util/thread_tools.h"

namespace YAML {
template <>
struct convert<aimrt::runtime::core::executor::MainThreadExecutor::Options> {
  using Options = aimrt::runtime::core::executor::MainThreadExecutor::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["name"] = rhs.name;
    node["thread_sched_policy"] = rhs.thread_sched_policy;
    node["thread_bind_cpu"] = rhs.thread_bind_cpu;

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

    return true;
  }
};
}  // namespace YAML

namespace aimrt::runtime::core::executor {

void MainThreadExecutor::Initialize(YAML::Node options_node) {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "Main thread executor can only be initialized once.");

  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  name_ = options_.name;

  try {
    util::SetNameForCurrentThread(name_);
    util::BindCpuForCurrentThread(options_.thread_bind_cpu);
    util::SetCpuSchedForCurrentThread(options_.thread_sched_policy);
  } catch (const std::exception& e) {
    AIMRT_WARN("Set thread policy for main thread get exception, {}", e.what());
  }

  main_thread_id_ = std::this_thread::get_id();

  options_node = options_;

  AIMRT_INFO("Main thread executor init complete");
}

void MainThreadExecutor::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Main thread executor can only run when state is 'Init'.");
}

void MainThreadExecutor::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  AIMRT_INFO("Main thread executor shutdown.");
}

}  // namespace aimrt::runtime::core::executor
