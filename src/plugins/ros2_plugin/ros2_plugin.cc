// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "ros2_plugin/ros2_plugin.h"
#include "core/aimrt_core.h"
#include "core/util/thread_tools.h"
#include "irobot_lock_free_events_queue/lock_free_events_queue.hpp"
#include "rcl/logging.h"
#include "rclcpp/version.h"
#if RCLCPP_VERSION_MAJOR == 16
  // Humble: use irobot events_executor
  #include "rclcpp/executors/events_executor/events_executor.hpp"
#elif RCLCPP_VERSION_MAJOR == 28
  // Jazzy: use system events_executor
  #include "rclcpp/experimental/executors/events_executor/events_executor.hpp"
#endif
#include "ros2_plugin/global.h"
#include "ros2_plugin/ros2_channel_backend.h"
#include "ros2_plugin/ros2_rpc_backend.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::ros2_plugin::Ros2Plugin::Options> {
  using Options = aimrt::plugins::ros2_plugin::Ros2Plugin::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["node_name"] = rhs.node_name;
    node["executor_type"] = rhs.executor_type;
    if (rhs.executor_type == "MultiThreaded") {
      node["executor_thread_num"] = rhs.executor_thread_num;
    }
    node["executor_name"] = rhs.executor_name;
    node["executor_sched_policy"] = rhs.executor_sched_policy;
    node["executor_bind_cpus"] = rhs.executor_bind_cpus;
    node["auto_initialize_logging"] = rhs.auto_initialize_logging;

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    rhs.node_name = node["node_name"].as<std::string>();

    if (node["executor_type"])
      rhs.executor_type = node["executor_type"].as<std::string>();

    if (rhs.executor_type == "MultiThreaded" && node["executor_thread_num"])
      rhs.executor_thread_num = node["executor_thread_num"].as<uint32_t>();

    if (node["auto_initialize_logging"]) {
      rhs.auto_initialize_logging = node["auto_initialize_logging"].as<bool>();
    }

    if (node["executor_name"])
      rhs.executor_name = node["executor_name"].as<std::string>();

    if (node["executor_sched_policy"])
      rhs.executor_sched_policy = node["executor_sched_policy"].as<std::string>();

    if (node["executor_bind_cpus"])
      rhs.executor_bind_cpus = node["executor_bind_cpus"].as<std::vector<uint32_t>>();

    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::ros2_plugin {

bool Ros2Plugin::Initialize(runtime::core::AimRTCore* core_ptr) noexcept {
  try {
    core_ptr_ = core_ptr;

    YAML::Node plugin_options_node = core_ptr_->GetPluginManager().GetPluginOptionsNode(Name());

    if (plugin_options_node && !plugin_options_node.IsNull()) {
      options_ = plugin_options_node.as<Options>();
    }

    init_flag_ = true;
    rclcpp::InitOptions op;

    op.auto_initialize_logging(options_.auto_initialize_logging);
    op.shutdown_on_signal = false;

    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr, op);
    }

    ros2_node_ptr_ = std::make_shared<rclcpp::Node>(options_.node_name);

    if (options_.executor_type == "SingleThreaded") {
      ros2_node_executor_ptr_ =
          std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    } else if (options_.executor_type == "StaticSingleThreaded") {
      ros2_node_executor_ptr_ =
          std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
    } else if (options_.executor_type == "MultiThreaded") {
      ros2_node_executor_ptr_ =
          std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
              rclcpp::ExecutorOptions(), options_.executor_thread_num);
    } else if (options_.executor_type == "EventsExecutor") {
#if RCLCPP_VERSION_MAJOR == 16
      // Humble: use irobot events_executor
      ros2_node_executor_ptr_ =
          std::make_shared<rclcpp::executors::EventsExecutor>(
              std::make_unique<LockFreeEventsQueue>());
#elif RCLCPP_VERSION_MAJOR == 28
      // Jazzy: use system events_executor in experimental namespace
      ros2_node_executor_ptr_ =
          std::make_shared<rclcpp::experimental::executors::EventsExecutor>(
              std::make_unique<LockFreeEventsQueue>());
#endif
    } else {
      AIMRT_ERROR_THROW("Invalid ros2 executor_type '{}'.",
                        options_.executor_type);
    }

    ros2_node_executor_ptr_->add_node(ros2_node_ptr_);

    core_ptr->RegisterHookFunc(runtime::core::AimRTCore::State::kPostStart, [this] {
      ros2_thread_ptr_ = std::make_unique<std::thread>(
          [this]() {
            SetExecutorThreadSchedOption();
            ros2_node_executor_ptr_->spin();
          });
    });

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPostInitLog,
                                [this] { SetPluginLogger(); });

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreInitRpc,
                                [this] { RegisterRos2RpcBackend(); });

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreInitChannel,
                                [this] { RegisterRos2ChannelBackend(); });

    plugin_options_node = options_;
    core_ptr_->GetPluginManager().UpdatePluginOptionsNode(Name(), plugin_options_node);

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("Initialize failed, {}", e.what());
  }

  return false;
}

void Ros2Plugin::Shutdown() noexcept {
  try {
    if (!init_flag_) return;

    auto default_context = rclcpp::contexts::get_global_default_context();
    default_context->shutdown("user called rclcpp::shutdown()");
    rclcpp::uninstall_signal_handlers();

    ros2_thread_ptr_->join();
  } catch (const std::exception& e) {
    AIMRT_ERROR("Shutdown failed, {}", e.what());
  }
}

void Ros2Plugin::SetExecutorThreadSchedOption() {
  try {
    aimrt::runtime::core::util::SetNameForCurrentThread(options_.executor_name);
    aimrt::runtime::core::util::BindCpuForCurrentThread(options_.executor_bind_cpus);
    aimrt::runtime::core::util::SetCpuSchedForCurrentThread(options_.executor_sched_policy);
  } catch (const aimrt::common::util::AimRTException& e) {
    AIMRT_WARN("Set thread policy for ros2 executor '{}' get exception, {}",
               Name(), e.what());
  }
}

void Ros2Plugin::SetPluginLogger() {
  SetLogger(aimrt::logger::LoggerRef(
      core_ptr_->GetLoggerManager().GetLoggerProxy().NativeHandle()));
}

void Ros2Plugin::RegisterRos2RpcBackend() {
  std::unique_ptr<runtime::core::rpc::RpcBackendBase> ros2_rpc_backend_ptr =
      std::make_unique<Ros2RpcBackend>();

  static_cast<Ros2RpcBackend*>(ros2_rpc_backend_ptr.get())
      ->RegisterGetExecutorFunc(
          [this](std::string_view executor_name) -> aimrt::executor::ExecutorRef {
            return core_ptr_->GetExecutorManager().GetExecutor(executor_name);
          });

  static_cast<Ros2RpcBackend*>(ros2_rpc_backend_ptr.get())
      ->SetNodePtr(ros2_node_ptr_);

  core_ptr_->GetRpcManager().RegisterRpcBackend(
      std::move(ros2_rpc_backend_ptr));
}

void Ros2Plugin::RegisterRos2ChannelBackend() {
  std::unique_ptr<runtime::core::channel::ChannelBackendBase> ros2_channel_backend_ptr =
      std::make_unique<Ros2ChannelBackend>();

  static_cast<Ros2ChannelBackend*>(ros2_channel_backend_ptr.get())
      ->SetNodePtr(ros2_node_ptr_);

  core_ptr_->GetChannelManager().RegisterChannelBackend(
      std::move(ros2_channel_backend_ptr));
}

}  // namespace aimrt::plugins::ros2_plugin
