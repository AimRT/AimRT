// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "zenoh_plugin/zenoh_plugin.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::zenoh_plugin::ZenohPlugin::Options> {
  using Options = aimrt::plugins::zenoh_plugin::ZenohPlugin::Options;

  static Node encode(const Options &rhs) {
    Node node;

    node["native_cfg_path"] = rhs.native_cfg_path;
    node["limit_domain"] = rhs.limit_domain;

    return node;
  }

  static bool decode(const Node &node, Options &rhs) {
    if (!node.IsMap()) return false;

    if (node["native_cfg_path"])
      rhs.native_cfg_path = node["native_cfg_path"].as<std::string>();

    if (node["limit_domain"])
      rhs.limit_domain = '/' + node["limit_domain"].as<std::string>();

    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::zenoh_plugin {

bool ZenohPlugin::Initialize(runtime::core::AimRTCore *core_ptr) noexcept {
  try {
    core_ptr_ = core_ptr;

    YAML::Node plugin_options_node = core_ptr_->GetPluginManager().GetPluginOptionsNode(Name());

    if (plugin_options_node && !plugin_options_node.IsNull()) {
      options_ = plugin_options_node.as<Options>();
    }

    init_flag_ = true;

    // todo remove role
    zenoh_manager_ptr_->Initialize(options_.native_cfg_path);

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPostInitLog,
                                [this] { SetPluginLogger(); });

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreInitChannel,
                                [this] { RegisterZenohChannelBackend(); });

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreInitRpc,
                                [this] { RegisterZenohRpcBackend(); });

    plugin_options_node = options_;

    return true;

  } catch (const std::exception &e) {
    AIMRT_ERROR("Initialize failed, {}", e.what());
  }
  return false;
}

void ZenohPlugin::Shutdown() noexcept {
  try {
    if (!init_flag_) return;

    stop_flag_ = true;

    zenoh_manager_ptr_->Shutdown();

  } catch (const std::exception &e) {
    AIMRT_ERROR("Shutdown failed, {}", e.what());
  }
}

void ZenohPlugin::SetPluginLogger() {
  SetLogger(aimrt::logger::LoggerRef(
      core_ptr_->GetLoggerManager().GetLoggerProxy().NativeHandle()));
}

void ZenohPlugin::RegisterZenohChannelBackend() {
  std::unique_ptr<runtime::core::channel::ChannelBackendBase> zenoh_channel_backend_ptr =
      std::make_unique<ZenohChannelBackend>(zenoh_manager_ptr_, options_.limit_domain);

  core_ptr_->GetChannelManager().RegisterChannelBackend(std::move(zenoh_channel_backend_ptr));
}

void ZenohPlugin::RegisterZenohRpcBackend() {
  std::unique_ptr<runtime::core::rpc::RpcBackendBase> zenoh_rpc_backend_ptr =
      std::make_unique<ZenohRpcBackend>(zenoh_manager_ptr_, options_.limit_domain);

  static_cast<ZenohRpcBackend *>(zenoh_rpc_backend_ptr.get())
      ->RegisterGetExecutorFunc(
          [this](std::string_view executor_name) -> aimrt::executor::ExecutorRef {
            return core_ptr_->GetExecutorManager().GetExecutor(executor_name);
          });

  core_ptr_->GetRpcManager().RegisterRpcBackend(std::move(zenoh_rpc_backend_ptr));
}

}  // namespace aimrt::plugins::zenoh_plugin