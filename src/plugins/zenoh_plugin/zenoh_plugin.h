// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <memory>

#include "aimrt_core_plugin_interface/aimrt_core_plugin_base.h"
#include "core/aimrt_core.h"
#include "zenoh_plugin/zenoh_channel_backend.h"
#include "zenoh_plugin/zenoh_rpc_backend.h"

namespace aimrt::plugins::zenoh_plugin {
class ZenohPlugin : public AimRTCorePluginBase {
 public:
  struct Options {
    std::string native_cfg_path;
    std::string limit_domain;
  };

 public:
  ZenohPlugin() = default;
  ~ZenohPlugin() override = default;

  std::string_view Name() const noexcept override { return "zenoh_plugin"; }

  bool Initialize(runtime::core::AimRTCore *core_ptr) noexcept override;
  void Shutdown() noexcept override;

 private:
  void SetPluginLogger();
  void RegisterZenohChannelBackend();
  void RegisterZenohRpcBackend();

 private:
  runtime::core::AimRTCore *core_ptr_ = nullptr;

  Options options_;

  bool init_flag_ = false;
  std::atomic_bool stop_flag_ = false;

  std::shared_ptr<ZenohManager> zenoh_manager_ptr_ = std::make_shared<ZenohManager>();
};

}  // namespace aimrt::plugins::zenoh_plugin