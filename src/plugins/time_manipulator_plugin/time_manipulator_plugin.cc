// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "time_manipulator_plugin/time_manipulator_plugin.h"

#include "aimrt_module_cpp_interface/rpc/rpc_handle.h"
#include "core/aimrt_core.h"
#include "time_manipulator_plugin/global.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::time_manipulator_plugin::TimeManipulatorPlugin::Options> {
  using Options = aimrt::plugins::time_manipulator_plugin::TimeManipulatorPlugin::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["service_name"] = rhs.service_name;

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    if (node["service_name"])
      rhs.service_name = node["service_name"].as<std::string>();

    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::time_manipulator_plugin {

bool TimeManipulatorPlugin::Initialize(runtime::core::AimRTCore* core_ptr) noexcept {
  try {
    core_ptr_ = core_ptr;

    YAML::Node plugin_options_node = core_ptr_->GetPluginManager().GetPluginOptionsNode(Name());

    if (plugin_options_node && !plugin_options_node.IsNull()) {
      options_ = plugin_options_node.as<Options>();
    }

    init_flag_ = true;

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPostInitLog,
                                [this] { SetPluginLogger(); });

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreInitExecutor,
                                [this] { RegisterTimeManipulatorExecutor(); });

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreInitModules,
                                [this] { RegisterRpcService(); });

    plugin_options_node = options_;
    core_ptr_->GetPluginManager().UpdatePluginOptionsNode(Name(), plugin_options_node);

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("Initialize failed, {}", e.what());
  }

  return false;
}

void TimeManipulatorPlugin::Shutdown() noexcept {
  try {
    if (!init_flag_) return;

  } catch (const std::exception& e) {
    AIMRT_ERROR("Shutdown failed, {}", e.what());
  }
}

void TimeManipulatorPlugin::SetPluginLogger() {
  SetLogger(aimrt::logger::LoggerRef(
      core_ptr_->GetLoggerManager().GetLoggerProxy().NativeHandle()));
}

void TimeManipulatorPlugin::RegisterTimeManipulatorExecutor() {
  core_ptr_->GetExecutorManager().RegisterExecutorGenFunc(
      "time_manipulator",
      [this]() -> std::unique_ptr<aimrt::runtime::core::executor::ExecutorBase> {
        auto ptr = std::make_unique<TimeManipulatorExecutor>();
        ptr->RegisterGetExecutorFunc(
            [this](std::string_view executor_name) -> aimrt::executor::ExecutorRef {
              return core_ptr_->GetExecutorManager().GetExecutor(executor_name);
            });
        return ptr;
      });
}

void TimeManipulatorPlugin::RegisterRpcService() {
  service_ptr_ = std::make_unique<TimeManipulatorServiceImpl>();

  if (!options_.service_name.empty())
    service_ptr_->SetServiceName(options_.service_name);

  const auto& executor_vec = core_ptr_->GetExecutorManager().GetAllExecutors();

  for (const auto& itr : executor_vec) {
    if (itr->Type() == "time_manipulator") {
      auto* ptr = dynamic_cast<TimeManipulatorExecutor*>(itr.get());
      if (ptr) {
        service_ptr_->RegisterTimeManipulatorExecutor(ptr);
      } else {
        AIMRT_ERROR("Invalid executor pointer");
      }
    }
  }

  auto rpc_handle_ref = aimrt::rpc::RpcHandleRef(
      core_ptr_->GetRpcManager().GetRpcHandleProxy().NativeHandle());

  bool ret = rpc_handle_ref.RegisterService(service_ptr_.get());
  AIMRT_CHECK_ERROR(ret, "Register service failed.");
}

}  // namespace aimrt::plugins::time_manipulator_plugin
