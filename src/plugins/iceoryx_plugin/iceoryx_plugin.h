// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <memory>
#include "aimrt_core_plugin_interface/aimrt_core_plugin_base.h"
#include "core/aimrt_core.h"
#include "iceoryx_plugin/iceoryx_channel_backend.h"
#include "iceoryx_plugin/iceoryx_manager.h"

namespace aimrt::plugins::iceoryx_plugin {

class IceoryxPlugin : public AimRTCorePluginBase {
 public:
  struct Options {
    uint64_t shm_init_size;
  };

 public:
  IceoryxPlugin() = default;

  ~IceoryxPlugin() override = default;
  std::string_view Name() const noexcept override { return "iceoryx_plugin"; }

  bool Initialize(runtime::core::AimRTCore *core_ptr) noexcept override;

  void Shutdown() noexcept override;

 private:
  void SetPluginLogger();

  void RegisterIceoryxChannelBackend();

  void RegisterIceoryxRpcBackend() {}

 private:
  runtime::core::AimRTCore *core_ptr_ = nullptr;

  Options options_;

  bool init_flag_ = false;

  std::atomic_bool stop_flag_ = false;

  std::shared_ptr<IceoryxManager> iceoryx_manager_ptr_ = std::make_shared<IceoryxManager>();
};
}  // namespace aimrt::plugins::iceoryx_plugin