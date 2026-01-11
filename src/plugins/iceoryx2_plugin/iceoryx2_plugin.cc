// Copyright (c) 2024, SmartCar Project
// Iceoryx2 Plugin for AimRT - Zero-copy IPC using Iceoryx2

#include "iceoryx2_plugin/iceoryx2_plugin.h"
#include "iceoryx2_plugin/global.h"
#include "iceoryx2_plugin/iceoryx2_channel_backend.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::iceoryx2_plugin::Iceoryx2Plugin::Options> {
  using Options = aimrt::plugins::iceoryx2_plugin::Iceoryx2Plugin::Options;

  static Node encode(const Options& rhs) {
    Node node;
    node["shm_init_size"] = rhs.shm_init_size;
    node["max_slice_len"] = rhs.max_slice_len;
    node["allocation_strategy"] = rhs.allocation_strategy;
    node["listener_thread_name"] = rhs.listener_thread_name;
    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    if (node["shm_init_size"])
      rhs.shm_init_size = node["shm_init_size"].as<uint64_t>();
    if (node["max_slice_len"])
      rhs.max_slice_len = node["max_slice_len"].as<uint64_t>();
    if (node["allocation_strategy"])
      rhs.allocation_strategy = node["allocation_strategy"].as<std::string>();
    if (node["listener_thread_name"])
      rhs.listener_thread_name = node["listener_thread_name"].as<std::string>();

    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::iceoryx2_plugin {

bool Iceoryx2Plugin::Initialize(runtime::core::AimRTCore* core_ptr) noexcept {
  try {
    core_ptr_ = core_ptr;

    YAML::Node plugin_options_node = core_ptr_->GetPluginManager().GetPluginOptionsNode(Name());

    if (plugin_options_node && !plugin_options_node.IsNull()) {
      options_ = plugin_options_node.as<Options>();
    }

    init_flag_ = true;

    // Register logger hook (with exception guard)
    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPostInitLog,
                                [this] {
                                  try {
                                    SetPluginLogger();
                                  } catch (const std::exception& e) {
                                    AIMRT_ERROR("SetPluginLogger failed: {}", e.what());
                                  }
                                });

    // Register channel backend hook (with exception guard)
    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreInitChannel,
                                [this] {
                                  try {
                                    RegisterIceoryx2ChannelBackend();
                                  } catch (const std::exception& e) {
                                    AIMRT_ERROR("RegisterIceoryx2ChannelBackend failed: {}", e.what());
                                  }
                                });

    core_ptr_->GetPluginManager().UpdatePluginOptionsNode(Name(), plugin_options_node);

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("Iceoryx2Plugin Initialize failed, {}", e.what());
  }

  return false;
}

void Iceoryx2Plugin::Shutdown() noexcept {
  try {
    if (!init_flag_) return;

    stop_flag_ = true;

  } catch (const std::exception& e) {
    AIMRT_ERROR("Iceoryx2Plugin Shutdown failed, {}", e.what());
  }
}

void Iceoryx2Plugin::SetPluginLogger() {
  SetLogger(aimrt::logger::LoggerRef(
      core_ptr_->GetLoggerManager().GetLoggerProxy().NativeHandle()));
}

void Iceoryx2Plugin::RegisterIceoryx2ChannelBackend() {
  // Create channel backend with plugin options
  auto iceoryx2_channel_backend_ptr = std::make_unique<Iceoryx2ChannelBackend>();

  // Configure backend with plugin options
  Iceoryx2ChannelBackend::Options backend_options;
  backend_options.shm_init_size = options_.shm_init_size;
  backend_options.max_slice_len = options_.max_slice_len;
  backend_options.allocation_strategy = options_.allocation_strategy;
  backend_options.listener_thread_name = options_.listener_thread_name;

  iceoryx2_channel_backend_ptr->SetOptions(backend_options);

  AIMRT_INFO("Iceoryx2 config: shm={}MB, max_slice={}MB, strategy={}",
             options_.shm_init_size / (1024 * 1024),
             options_.max_slice_len / (1024 * 1024),
             options_.allocation_strategy);

  core_ptr_->GetChannelManager().RegisterChannelBackend(std::move(iceoryx2_channel_backend_ptr));
}

}  // namespace aimrt::plugins::iceoryx2_plugin
