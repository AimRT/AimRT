// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "log_control_plugin/log_control_plugin.h"

#include "aimrt_module_cpp_interface/rpc/rpc_handle.h"
#include "core/aimrt_core.h"
#include "log_control_plugin/global.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::log_control_plugin::LogControlPlugin::Options> {
  using Options = aimrt::plugins::log_control_plugin::LogControlPlugin::Options;

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

namespace aimrt::plugins::log_control_plugin {

bool LogControlPlugin::Initialize(runtime::core::AimRTCore* core_ptr) noexcept {
  try {
    core_ptr_ = core_ptr;

    YAML::Node plugin_options_node = core_ptr_->GetPluginManager().GetPluginOptionsNode(Name());

    if (plugin_options_node && !plugin_options_node.IsNull()) {
      options_ = plugin_options_node.as<Options>();
    }

    init_flag_ = true;

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPostInitLog,
                                [this] { SetPluginLogger(); });

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

void LogControlPlugin::Shutdown() noexcept {
  try {
    if (!init_flag_) return;

  } catch (const std::exception& e) {
    AIMRT_ERROR("Shutdown failed, {}", e.what());
  }
}

void LogControlPlugin::SetPluginLogger() {
  SetLogger(aimrt::logger::LoggerRef(
      core_ptr_->GetLoggerManager().GetLoggerProxy().NativeHandle()));
}

void LogControlPlugin::RegisterRpcService() {
  service_ptr_ = std::make_unique<LogControlServiceImpl>();

  if (!options_.service_name.empty())
    service_ptr_->SetServiceName(options_.service_name);

  service_ptr_->SetLoggerManager(&(core_ptr_->GetLoggerManager()));

  auto rpc_handle_ref = aimrt::rpc::RpcHandleRef(
      core_ptr_->GetRpcManager().GetRpcHandleProxy().NativeHandle());

  bool ret = rpc_handle_ref.RegisterService(service_ptr_.get());
  AIMRT_CHECK_ERROR(ret, "Register service failed.");
}

}  // namespace aimrt::plugins::log_control_plugin
