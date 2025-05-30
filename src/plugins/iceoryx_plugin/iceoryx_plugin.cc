// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "iceoryx_plugin/iceoryx_plugin.h"
#include "iceoryx_plugin/global.h"
#include "iceoryx_plugin/iceoryx_channel_backend.h"
#if defined(_WIN32)
  #include <windows.h>
#else
  #include <unistd.h>
#endif

namespace YAML {
template <>
struct convert<aimrt::plugins::iceoryx_plugin::IceoryxPlugin::Options> {
  using Options = aimrt::plugins::iceoryx_plugin::IceoryxPlugin::Options;

  static Node encode(const Options &rhs) {
    Node node;

    node["shm_init_size"] = rhs.shm_init_size;
    node["runtime_id"] = rhs.runtime_id;

    return node;
  }

  static bool decode(const Node &node, Options &rhs) {
    if (!node.IsMap()) return false;

    if (node["shm_init_size"])
      rhs.shm_init_size = node["shm_init_size"].as<uint64_t>();

    if (node["runtime_id"].IsDefined()) {
      rhs.runtime_id = node["runtime_id"].as<std::string>();
    }

    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::iceoryx_plugin {

bool IceoryxPlugin::Initialize(runtime::core::AimRTCore *core_ptr) noexcept {
  try {
    core_ptr_ = core_ptr;

    YAML::Node plugin_options_node = core_ptr_->GetPluginManager().GetPluginOptionsNode(Name());

    if (plugin_options_node && !plugin_options_node.IsNull()) {
      options_ = plugin_options_node.as<Options>();
    }

#if defined(_WIN32)
    std::string runtime_id = "iceoryx" + std::to_string(GetProcessId(GetCurrentProcess()));
#else
    std::string runtime_id = "iceoryx" + std::to_string(getpid());
#endif

    if (!options_.runtime_id.empty()) {
      runtime_id = options_.runtime_id;
    }

    init_flag_ = true;
    iceoryx_manager_.Initialize(options_.shm_init_size, runtime_id);

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPostInitLog,
                                [this] { SetPluginLogger(); });

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreInitChannel,
                                [this] { RegisterIceoryxChannelBackend(); });

    core_ptr_->GetPluginManager().UpdatePluginOptionsNode(Name(), plugin_options_node);

    plugin_options_node = options_;

    return true;
  } catch (const std::exception &e) {
    AIMRT_ERROR("Initialize failed, {}", e.what());
  }

  return false;
}

void IceoryxPlugin::Shutdown() noexcept {
  try {
    if (!init_flag_) return;

    stop_flag_ = true;

    iceoryx_manager_.Shutdown();

  } catch (const std::exception &e) {
    AIMRT_ERROR("Shutdown failed, {}", e.what());
  }
}

void IceoryxPlugin::SetPluginLogger() {
  SetLogger(aimrt::logger::LoggerRef(
      core_ptr_->GetLoggerManager().GetLoggerProxy().NativeHandle()));
}

void IceoryxPlugin::RegisterIceoryxChannelBackend() {
  std::unique_ptr<runtime::core::channel::ChannelBackendBase> iceoryx_channel_backend_ptr =
      std::make_unique<IceoryxChannelBackend>(iceoryx_manager_);

  core_ptr_->GetChannelManager().RegisterChannelBackend(std::move(iceoryx_channel_backend_ptr));
}

}  // namespace aimrt::plugins::iceoryx_plugin